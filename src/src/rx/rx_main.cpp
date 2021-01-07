#include <Arduino.h>
#include "targets.h"
#include "utils.h"
#include "common.h"
#include "LowPassFilter.h"
#if RADIO_SX128x
#include "SX1280.h"
#else
#include "LoRa_SX127x.h"
#endif
#if RX_GHST_ENABLED
#include "GHST.h"
#define RX_CLASS GHST
#else
#include "CRSF_RX.h"
#define RX_CLASS CRSF_RX
#endif
#include "FHSS.h"
#include "rx_LinkQuality.h"
#include "HwTimer.h"
#include "HwSerial.h"
#include "debug_elrs.h"
#include "helpers.h"
#include "rc_channels.h"
#include "rx_servo_out.h"

static void SetRFLinkRate(uint8_t rate);
void ICACHE_RAM_ATTR LostConnection();

//// CONSTANTS ////
#define SEND_LINK_STATS_TO_FC_INTERVAL 100
#define PRINT_FREQ_ERROR               0
//#define NUM_FAILS_TO_RESYNC            100
#define PRINT_RATE 1
#define PRINT_TIMING 0

#if PRINT_RATE && NO_DATA_TO_FC
uint32_t print_rate_cnt;
uint32_t print_rate_cnt_fail;
uint32_t print_Rate_cnt_time;
#endif
#if PRINT_TIMING
static volatile uint32_t DRAM_ATTR print_rx_isr_end_time;
#endif

///////////////////

#if RADIO_SX128x
SX1280Driver DRAM_FORCE_ATTR Radio(OTA_PACKET_SIZE);
#else
SX127xDriver DRAM_FORCE_ATTR Radio(OTA_PACKET_SIZE);
#endif
RX_CLASS DRAM_FORCE_ATTR crsf(CrsfSerial); //pass a serial port object to the class for it to use

connectionState_e DRAM_ATTR connectionState;
static volatile uint8_t DRAM_ATTR NonceRXlocal; // nonce that we THINK we are up to.
static uint8_t DRAM_ATTR TLMinterval;
static volatile uint32_t DRAM_ATTR tlm_check_ratio;
static volatile uint32_t DRAM_ATTR rx_last_valid_us; //Time the last valid packet was recv
static volatile int32_t DRAM_ATTR rx_freqerror;
#if NUM_FAILS_TO_RESYNC
static uint32_t rx_lost_packages;
#endif
static volatile int32_t DRAM_ATTR rx_hw_isr_running;

static uint16_t DRAM_ATTR CRCCaesarCipher;

#if SERVO_OUTPUTS_ENABLED
#if !SERVO_WRITE_FROM_ISR
static uint8_t DRAM_ATTR update_servos;
#endif
#endif
static crsf_channels_t DRAM_ATTR CrsfChannels;

///////////////////////////////////////////////
////////////////  Filters  ////////////////////
static LPF DRAM_FORCE_ATTR LPF_FreqError(5);
static LPF DRAM_FORCE_ATTR LPF_UplinkRSSI(5);
static LPF DRAM_FORCE_ATTR LPF_UplinkSNR(5);

//////////////////////////////////////////////////////////////
////// Variables for Telemetry and Link Quality //////////////

static uint32_t DRAM_ATTR LastValidPacket; //Time the last valid packet was recv
#if !SERVO_OUTPUTS_ENABLED
static uint32_t DRAM_ATTR SendLinkStatstoFCintervalNextSend;
#endif
static mspPacket_t DRAM_FORCE_ATTR msp_packet_rx;
static uint32_t DRAM_ATTR msp_packet_rx_sent;
static mspPacket_t DRAM_FORCE_ATTR msp_packet_tx;
static uint8_t DRAM_ATTR tlm_msp_send, tlm_msp_rcvd;
static uint8_t DRAM_ATTR uplink_Link_quality;

///////////////////////////////////////////////////////////////
///////////// Variables for Sync Behaviour ////////////////////
static uint32_t DRAM_ATTR RFmodeNextCycle; // set from isr
static uint8_t DRAM_ATTR scanIndex;
static uint8_t DRAM_ATTR tentative_cnt;
#if RX_UPDATE_AIR_RATE
static uint8_t DRAM_ATTR updatedAirRate = 0xff;
#endif

///////////////////////////////////////
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
struct gpio_out dbg_pin_tmr;
#endif
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
struct gpio_out dbg_pin_rx;
#endif

///////////////////////////////////////

static bool DRAM_ATTR ledState;
inline void led_set_state(bool state)
{
    ledState = state;
    platform_set_led(state);
}

inline void led_toggle(void)
{
    led_set_state(!ledState);
}

static void ICACHE_RAM_ATTR handle_tlm_ratio(uint8_t interval)
{
    if (TLMinterval == interval) return;

    if ((TLM_RATIO_NO_TLM < interval) && (TLM_RATIO_MAX > interval))
    {
        tlm_check_ratio = TLMratioEnumToValue(interval) - 1;
    }
    else
    {
        tlm_check_ratio = 0;
    }
    TLMinterval = interval;
    DEBUG_PRINTF("TLM: %u\n", interval);
}
///////////////////////////////////////

void ICACHE_RAM_ATTR FillLinkStats()
{
    int32_t rssiDBM = Radio.LastPacketRSSI;
    rssiDBM = LPF_UplinkRSSI.update(rssiDBM);
    // our rssiDBM is currently in the range -128 to 98, but BF wants a value in the range
    // 0 to 255 that maps to -1 * the negative part of the rssiDBM, so cap at 0.
    if (0 < rssiDBM) rssiDBM = 0;
    else if (rssiDBM < INT8_MIN) rssiDBM = INT8_MIN;
    //CrsfChannels.ch15 = UINT10_to_CRSF(MAP(rssiDBM, -100, -50, 0, 1023));
    //CrsfChannels.ch14 = UINT10_to_CRSF(MAP_U16(crsf.LinkStatistics.uplink_Link_quality, 0, 100, 0, 1023));
    crsf.LinkStatistics.uplink_RSSI_1 = -1 * rssiDBM; // to match BF
    crsf.LinkStatistics.uplink_SNR = LPF_UplinkSNR.update(Radio.LastPacketSNR * 10);
}

uint8_t ICACHE_RAM_ATTR RadioFreqErrorCorr(void)
{
    // Do freq correction before FHSS
    /* Freq tunin ~84us */
    int32_t freqerror = rx_freqerror;
    uint8_t retval = 0;
    rx_freqerror = 0;
    if (!freqerror)
        return 0;

#if PRINT_FREQ_ERROR
    DEBUG_PRINTF(" > freqerror:%u", freqerror);
#endif

#define MAX_ERROR 0 //3000
#if MAX_ERROR
    if (freqerror < -MAX_ERROR)
        freqerror = -MAX_ERROR;
    else if (freqerror > MAX_ERROR)
        freqerror = MAX_ERROR;
#endif
    freqerror = LPF_FreqError.update(freqerror);

#if PRINT_FREQ_ERROR
    DEBUG_PRINTF(" smooth:%u", freqerror);
#endif

    if (abs(freqerror) > 100) // 120
    {
        FHSSfreqCorrectionSet(freqerror);
        Radio.setPPMoffsetReg(freqerror, 0);
        retval = 1;
    }
#if PRINT_FREQ_ERROR
    //extern int_fast32_t FreqCorrection;
    //DEBUG_PRINTF(" local:%u", FreqCorrection);
#endif

    return retval;
}

uint8_t ICACHE_RAM_ATTR HandleFHSS(uint_fast8_t & nonce)
{
    uint8_t fhss = ((nonce % ExpressLRS_currAirRate->FHSShopInterval) == 0);
    if (fhss) {
        FHSSincCurrIndex();
    }
    ++nonce;
    return fhss;
}

void ICACHE_RAM_ATTR HandleSendTelemetryResponse(uint_fast8_t lq) // total ~79us
{
    DEBUG_PRINTF(" X");
    // esp requires word aligned buffer
    uint32_t __tx_buffer[(OTA_PACKET_SIZE + sizeof(uint32_t) - 1) / sizeof(uint32_t)];
    uint8_t *tx_buffer = (uint8_t *)__tx_buffer;
    uint8_t index = OTA_PACKET_DATA;

    if ((tlm_msp_send == 1) && (msp_packet_tx.type == MSP_PACKET_TLM_OTA))
    {
        if (RcChannels_tlm_ota_send(tx_buffer, msp_packet_tx, 0) || msp_packet_tx.error) {
            msp_packet_tx.reset();
            tlm_msp_send = 0;
        }
    }
    else if (crsf.GpsStatsPack(tx_buffer))
    {
        RcChannels_packetTypeSet(tx_buffer, DL_PACKET_GPS);
    }
    else
    {
        crsf.LinkStatisticsPack(tx_buffer, lq);
        RcChannels_packetTypeSet(tx_buffer, DL_PACKET_TLM_LINK);
    }

#if OTA_PACKET_10B
    tx_buffer[index++] = 0;
    tx_buffer[index++] = 0;
#endif // OTA_PACKET_10B

    uint16_t crc = CalcCRC16(tx_buffer, index, CRCCaesarCipher);
    tx_buffer[index++] = (crc >> 8);
    tx_buffer[index++] = (crc & 0xFF);
    Radio.TXnb(tx_buffer, index, FHSSgetCurrFreq());

    // Adds packet to LQ otherwise an artificial drop in LQ is seen due to sending TLM.
    LQ_packetAck();
}

void tx_done_cb(void)
{
    // Configure RX only next is not hopping time
    //if (((NonceRXlocal + 1) % ExpressLRS_currAirRate->FHSShopInterval) != 0)
    //    Radio.RXnb(FHSSgetCurrFreq());
}

void ICACHE_RAM_ATTR HWtimerCallback(uint32_t const us)
{
    //DEBUG_PRINTF("H");
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_tmr, 1);
#endif

    rx_hw_isr_running = 1;

    int32_t diff_us = 0;
    const uint32_t last_rx_us = rx_last_valid_us;
    const uint_fast8_t tlm_ratio = tlm_check_ratio;
    uint_fast8_t fhss_config_rx = 0;
    uint_fast8_t nonce = NonceRXlocal;
    rx_last_valid_us = 0;

    /* do adjustment */
    if (unlikely(last_rx_us != 0))
    {
#if 1 //!USE_TIMER_KICK
        diff_us = (int32_t)((uint32_t)(us - last_rx_us));

#if 0
        int32_t const interval = ExpressLRS_currAirRate->interval;
        if (diff_us > (interval >> 1))
        {
            /* the timer was called too early */
            diff_us %= interval;
            diff_us -= interval;
        }
#else
        if (diff_us < -TIMER_OFFSET) diff_us = -TIMER_OFFSET;
        else if (diff_us > TIMER_OFFSET) diff_us = TIMER_OFFSET;
#endif
        /* Adjust the timer */
        //if ((TIMER_OFFSET_LIMIT < diff_us) || (diff_us < 0))
            TxTimer.reset(diff_us - TIMER_OFFSET);
#else // USE_TIMER_KICK
        TxTimer.reset(-TIMER_OFFSET_KICK);
#endif // USE_TIMER_KICK
#if PRINT_RATE && NO_DATA_TO_FC
        print_rate_cnt++;
#endif
    }
    else
    {
#if PRINT_RATE && NO_DATA_TO_FC
        print_rate_cnt_fail++;
#endif
#if !USE_TIMER_KICK
        TxTimer.setTime(); // Reset timer interval
#else
        TxTimer.reset(0); // Reset timer interval
#endif
    }

    fhss_config_rx |= RadioFreqErrorCorr();
    fhss_config_rx |= HandleFHSS(nonce);

    uint_fast8_t lq = LQ_getlinkQuality();
    uplink_Link_quality = lq;
    LQ_nextPacket();

    if ((0 < tlm_ratio) && ((nonce & tlm_ratio) == 0))
    {
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
        gpio_out_write(dbg_pin_tmr, 0);
#endif
        HandleSendTelemetryResponse(lq);
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
        gpio_out_write(dbg_pin_tmr, 1);
#endif
        goto hw_tmr_isr_exit;
    }
#if NUM_FAILS_TO_RESYNC
    else if (!last_rx_us)
    {
        if (NUM_FAILS_TO_RESYNC < (++rx_lost_packages)) {
            // consecutive losts => trigger connection lost to resync
            LostConnection();
            DEBUG_PRINTF("RESYNC!");
            goto hw_tmr_isr_exit;
        }
    }
#endif

    /* Configure next reception if needed */
    if (fhss_config_rx) {
        Radio.RXnb(FHSSgetCurrFreq());
    }

hw_tmr_isr_exit:
    NonceRXlocal = nonce;

#if (PRINT_TIMING)
    uint32_t now = micros();
    DEBUG_PRINTF("RX:%u (t:%u) HW:%u diff:%d (t:%u)\n",
                 last_rx_us, (last_rx_us - print_rx_isr_end_time),
                 us, diff_us, (uint32_t)(now - us));
#endif

    rx_hw_isr_running = 0;

#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_tmr, 0);
#endif
    return;
}

void ICACHE_RAM_ATTR LostConnection()
{
    if (connectionState <= STATE_lost)
    {
        return; // Already disconnected
    }

#if SERVO_OUTPUTS_ENABLED
    servo_out_fail_safe();
#endif

    uint32_t irq = _SAVE_IRQ();
    TxTimer.stop(); // Stop sync timer
    _RESTORE_IRQ(irq);

    // Reset FHSS
    FHSSfreqCorrectionReset();
    FHSSsetCurrIndex(0);
    LPF_FreqError.init(0);

    connectionState = STATE_lost; //set lost connection

    led_set_state(0);             // turn off led
    Radio.RXnb(GetInitialFreq()); // in conn lost state we always want to listen on freq index 0
    DEBUG_PRINTF("lost conn\n");

    platform_connection_state(connectionState);
}

void ICACHE_RAM_ATTR TentativeConnection(int32_t freqerror)
{
    /* Do initial freq correction */
    FHSSfreqCorrectionSet(freqerror);
    Radio.setPPMoffsetReg(freqerror, 0);
    LPF_FreqError.init(freqerror);
    rx_last_valid_us = 0;

    tentative_cnt = 0;
    connectionState = STATE_tentative;
    DEBUG_PRINTF("tentative\n");
    TxTimer.start();     // Start local sync timer
#if !USE_TIMER_KICK
    TxTimer.setTime(80); // Trigger isr right after reception
#endif
    led_set_state(1); // turn on led
}

void ICACHE_RAM_ATTR GotConnection()
{
    connectionState = STATE_connected; //we got a packet, therefore no lost connection

    led_set_state(1); // turn on led
    DEBUG_PRINTF("connected\n");

    platform_connection_state(connectionState);
}

void ICACHE_RAM_ATTR ProcessRFPacketCallback(uint8_t *rx_buffer, const uint32_t current_us)
{
    /* Processing takes:
        R9MM: ~160us
    */
    if (rx_hw_isr_running)
        // Skip if hw isr is triggered already (e.g. TX has some weird latency)
        return;

    /* Error in reception (CRC etc), kick hw timer */
    if (!rx_buffer) {
#if USE_TIMER_KICK
        TxTimer.triggerSoon(); // Trigger FHSS ISR
#endif
        return;
    }

#if (DBG_PIN_RX_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_rx, 1);
#endif

    //DEBUG_PRINTF("I");
    uint32_t freq_err;
    const connectionState_e _conn_state = connectionState;
    const uint16_t crc = CalcCRC16(rx_buffer, OTA_PACKET_PAYLOAD, CRCCaesarCipher);
    const uint16_t crc_in = ((uint16_t)rx_buffer[OTA_PACKET_PAYLOAD] << 8) + rx_buffer[OTA_PACKET_PAYLOAD+1];
    const uint8_t type = RcChannels_packetTypeGet(rx_buffer);

    if (crc_in != crc)
    {
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
        gpio_out_write(dbg_pin_rx, 0);
#endif
#if USE_TIMER_KICK
        TxTimer.triggerSoon(); // Trigger FHSS ISR
#endif
        DEBUG_PRINTF(" !");
        return;
    }

    freq_err = Radio.GetFrequencyError();

    rx_last_valid_us = current_us;
    LastValidPacket = millis();

    switch (type)
    {
        case UL_PACKET_SYNC:
        {
            //DEBUG_PRINTF(" S");
            ElrsSyncPacket_s const * const sync = (ElrsSyncPacket_s*)rx_buffer;

            if (sync->CRCCaesarCipher == CRCCaesarCipher)
            {
                if (_conn_state == STATE_disconnected || _conn_state == STATE_lost)
                {
                    TentativeConnection(freq_err);
                    freq_err = 0;
                }
                else if (_conn_state == STATE_tentative)
                {
                    if (NonceRXlocal == sync->rxtx_counter &&
                        FHSSgetCurrIndex() == sync->fhssIndex)
                    {
                        GotConnection();
                    }
                    else if (2 < (tentative_cnt++))
                    {
                        LostConnection();
                    }
                }

#if RX_UPDATE_AIR_RATE
                if (current_rate_config != sync->air_rate)
                {
                    updatedAirRate = sync->air_rate;
                }
#endif

                handle_tlm_ratio(sync->tlm_interval);
                FHSSsetCurrIndex(sync->fhssIndex);
                NonceRXlocal = sync->rxtx_counter;
            }
            break;
        }
        case UL_PACKET_RC_DATA: //Standard RC Data Packet
            //DEBUG_PRINTF(" R");
            if (STATE_lost < _conn_state)
            {
                RcChannels_channels_extract(rx_buffer, CrsfChannels);
#if SERVO_OUTPUTS_ENABLED
#if SERVO_WRITE_FROM_ISR
                servo_out_write(&CrsfChannels);
#else
                update_servos = 1;
#endif
#else // !SERVO_OUTPUTS_ENABLED
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
                gpio_out_write(dbg_pin_rx, 0);
#endif
                crsf.sendRCFrameToFC(&CrsfChannels);
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
                gpio_out_write(dbg_pin_rx, 1);
#endif
#endif // SERVO_OUTPUTS_ENABLED
            }
            break;

        case UL_PACKET_MSP: {
#if !SERVO_OUTPUTS_ENABLED
            //DEBUG_PRINTF(" M");
            if (RcChannels_tlm_ota_receive(rx_buffer, msp_packet_rx)) {
                //crsf.sendMSPFrameToFC(msp_packet_rx);
                //msp_packet_rx.reset();
                tlm_msp_rcvd = 1;
            }
#endif
            break;
        }

        case UL_PACKET_UNKNOWN:
        default:
            /* Not a valid packet, ignore it */
            rx_last_valid_us = 0;
            freq_err = 0;
            return;
            //break;
    }

#if OTA_PACKET_10B
    if (STATE_lost < _conn_state) {
        // Received in every packet to maintain sync all the time
        FHSSsetCurrIndex(rx_buffer[OTA_PACKET_DATA]);
        NonceRXlocal = rx_buffer[OTA_PACKET_DATA+1];
    }
#endif // OTA_PACKET_10B

    rx_freqerror = freq_err;

#if NUM_FAILS_TO_RESYNC
    rx_lost_packages = 0;
#endif
    LQ_packetAck();
    FillLinkStats();

#if PRINT_TIMING
    print_rx_isr_end_time = micros();
#endif
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_rx, 0);
#endif

#if USE_TIMER_KICK
    TxTimer.triggerSoon(); // Trigger FHSS ISR
#endif
}

void forced_stop(void)
{
    uint32_t irq = _SAVE_IRQ();
    TxTimer.stop();
    Radio.StopContRX();
    _RESTORE_IRQ(irq);
}

static void SetRFLinkRate(uint8_t rate) // Set speed of RF link (hz)
{
    const expresslrs_mod_settings_t *const config = get_elrs_airRateConfig(rate);
    if (config == NULL || config == ExpressLRS_currAirRate)
        return; // No need to modify, rate is same

    // Stop ongoing actions before configuring next rate
    forced_stop();

    ExpressLRS_currAirRate = config;
    current_rate_config = rate;

    DEBUG_PRINTF("Set RF rate: %u\n", config->rate);

    // Init CRC aka LQ array
    LQ_reset();
    // Reset FHSS
    FHSSfreqCorrectionReset();
    FHSSsetCurrIndex(0);

    handle_tlm_ratio(TLM_RATIO_NO_TLM);

    Radio.Config(config->bw, config->sf, config->cr, GetInitialFreq(),
                 config->PreambleLen, (OTA_PACKET_CRC == 0));

    // Measure RF noise
#ifdef DEBUG_SERIAL // TODO: Enable this when noize floor is used!
    int RFnoiseFloor = Radio.MeasureNoiseFloor(10, GetInitialFreq());
    DEBUG_PRINTF("RF noise floor: %d dBm\n", RFnoiseFloor);
    (void)RFnoiseFloor;
#endif

    TxTimer.updateInterval(config->interval);
    LPF_FreqError.init(0);
    LPF_UplinkRSSI.init(0);
    LPF_UplinkSNR.init(0);
#if !SERVO_OUTPUTS_ENABLED
    crsf.LinkStatistics.uplink_RSSI_2 = 0;
    crsf.LinkStatistics.rf_Mode = config->rate_osd_num;
#endif
    //TxTimer.setTime();
    Radio.RXnb();
}

/* FC sends v1 MSPs */
void msp_data_cb(uint8_t const *const input)
{
    if ((read_u8(&tlm_msp_send) != 0) || (tlm_check_ratio == 0))
        return;

    /* process MSP packet from flight controller
     *  [0] header: seq&0xF,
     *  [1] payload size
     *  [2] function
     *  [3...] payload + crc
     */
    mspHeaderV1_RX_t *hdr = (mspHeaderV1_RX_t *)input;
    uint16_t iter;

    if (hdr->flags & MSP_STARTFLAG) {
        msp_packet_tx.reset();
        msp_packet_tx.type = MSP_PACKET_TLM_OTA;
        msp_packet_tx.payloadSize = hdr->hdr.payloadSize + 2; // incl size+func
        msp_packet_tx.function = hdr->hdr.function;
    }
    for (iter = 0; (iter < sizeof(hdr->payload)) && !msp_packet_tx.iterated(); iter++) {
        msp_packet_tx.addByte(hdr->payload[iter]);
    }
    if (iter < sizeof(hdr->payload)) {
        if (!msp_packet_tx.error) {
            msp_packet_tx.addByte(hdr->payload[sizeof(hdr->payload)-1]); // include CRC
            msp_packet_tx.setIteratorToSize();
            write_u8(&tlm_msp_send, 1); // rdy for sending
        } else {
            msp_packet_tx.reset();
        }
    }
}

void setup()
{
    uint8_t UID[6] = {MY_UID};

#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
    dbg_pin_tmr = gpio_out_setup(DBG_PIN_TMR_ISR, 0);
#endif
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
    dbg_pin_rx = gpio_out_setup(DBG_PIN_RX_ISR, 0);
#endif

    connectionState = STATE_disconnected;
    tentative_cnt = 0;
    CRCCaesarCipher = CalcCRC16(UID, sizeof(UID), 0);

#if !SERVO_OUTPUTS_ENABLED
    CrsfSerial.Begin(CRSF_RX_BAUDRATE);
#endif

    msp_packet_tx.reset();
    msp_packet_rx.reset();

    DEBUG_PRINTF("ExpressLRS RX Module...\n");
    platform_setup();

    //FHSSrandomiseFHSSsequence();

    // Prepare radio
#if defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
    Radio.RFmodule = RFMOD_SX1278;
#elif !RADIO_SX128x
    Radio.RFmodule = RFMOD_SX1276;
#endif
    Radio.RXdoneCallback1 = ProcessRFPacketCallback;
    Radio.TXdoneCallback1 = tx_done_cb;
    Radio.SetPins(GPIO_PIN_RST, GPIO_PIN_DIO0, GPIO_PIN_DIO1, GPIO_PIN_DIO2,
                  GPIO_PIN_BUSY, GPIO_PIN_TX_ENABLE, GPIO_PIN_RX_ENABLE);
    Radio.SetSyncWord(getSyncWord());
    Radio.Begin(GPIO_PIN_SCK, GPIO_PIN_MISO, GPIO_PIN_MOSI, GPIO_PIN_NSS);
    Radio.SetOutputPower(0b1111); // default RX to max power for tlm

#if (GPIO_PIN_ANTENNA_SELECT != UNDEF_PIN)
    gpio_out_setup(GPIO_PIN_ANTENNA_SELECT, 0);
#endif

    // Set call back for timer ISR
    TxTimer.callbackTock = &HWtimerCallback;
    TxTimer.init();
    // Init first scan index
    scanIndex = RATE_DEFAULT;
#if SERVO_OUTPUTS_ENABLED
    servo_out_init();
#else
    // Initialize CRSF protocol handler
    crsf.MspCallback = msp_data_cb;
    crsf.Begin();
#endif
}

static uint32_t led_toggle_ms = 0;
void loop()
{
    uint32_t now = millis();
    const connectionState_e _conn_state = (connectionState_e)read_u32(&connectionState);

#if RX_UPDATE_AIR_RATE
    uint8_t new_air_rate = read_u8(&updatedAirRate);
    /* update air rate config, timed to FHSS index 0 */
    if (new_air_rate < get_elrs_airRateMax() && new_air_rate != current_rate_config)
    {
        write_u32(&connectionState, (uint32_t)STATE_lost); // Mark to lost to stay on received rate and force resync.
        SetRFLinkRate(new_air_rate); // configure air rate
        RFmodeNextCycle = now;
        return;
    }
#endif /* RX_UPDATE_AIR_RATE */

    if (STATE_lost < _conn_state)
    {
        // check if connection is lost or in very bad shape
        if (ExpressLRS_currAirRate->connectionLostTimeout <= (int32_t)(now - read_u32(&LastValidPacket))
            /*|| read_u8(&uplink_Link_quality) <= 10*/)
        {
            LostConnection();
        }
        else if (_conn_state == STATE_connected)
        {
#if SERVO_OUTPUTS_ENABLED
#if !SERVO_WRITE_FROM_ISR
            if (read_u8(&update_servos))
                servo_out_write(&CrsfChannels);
#endif
#else
            if (SEND_LINK_STATS_TO_FC_INTERVAL <= (uint32_t)(now - SendLinkStatstoFCintervalNextSend))
            {
                crsf.LinkStatistics.uplink_Link_quality = read_u8(&uplink_Link_quality);
                crsf.LinkStatisticsSend();
                SendLinkStatstoFCintervalNextSend = now;
            }
#endif
        }
    }
    else if (_conn_state == STATE_disconnected)
    { /* Cycle only if initial connection search */

        if (ExpressLRS_currAirRate->syncSearchTimeout < (uint32_t)(now - RFmodeNextCycle))
        {
            uint8_t max_rate = get_elrs_airRateMax();
            SetRFLinkRate((scanIndex % max_rate)); //switch between rates
            scanIndex++;
            if (max_rate <= scanIndex)
                platform_connection_state(STATE_search_iteration_done);

            RFmodeNextCycle = now;
        }
        else if (150 <= (uint32_t)(now - led_toggle_ms))
        {
            led_toggle();
            led_toggle_ms = now;
        }
    }
    else if (_conn_state == STATE_lost)
    { /* Just blink a led if connections is lost */
        if (300 <= (uint32_t)(now - led_toggle_ms))
        {
            led_toggle();
            led_toggle_ms = now;
        }
    }

#if !SERVO_OUTPUTS_ENABLED
    crsf.handleUartIn();
#endif

    platform_loop(_conn_state);

    platform_wd_feed();

    /* Send MSP in junks to FC */
    if (read_u8(&tlm_msp_rcvd) && (10 <= (now - msp_packet_rx_sent))) {
        crsf.sendMSPFrameToFC(msp_packet_rx);
        if (msp_packet_rx.iterated() || msp_packet_rx.error) {
            msp_packet_rx.reset();
            write_u8(&tlm_msp_rcvd, 0);
        }
        msp_packet_rx_sent = now;
    }

#if PRINT_RATE && NO_DATA_TO_FC
    if ((1000U <= (uint32_t)(now - print_Rate_cnt_time)) &&
        (_conn_state == STATE_connected)) {
        DEBUG_PRINTF(" Rate: -%u +%u LQ:%u RSSI:%d SNR:%d\n",
            read_u32(&print_rate_cnt_fail),
            read_u32(&print_rate_cnt),
            read_u8(&uplink_Link_quality),
            LPF_UplinkRSSI.value(),
            LPF_UplinkSNR.value());
        write_u32(&print_rate_cnt, 0);
        write_u32(&print_rate_cnt_fail, 0);
        print_Rate_cnt_time = now;
    }
#endif
}
