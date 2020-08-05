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
#include "CRSF_RX.h"
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

///////////////////

#if RADIO_SX128x
SX1280Driver Radio(RadioSpi, OTA_PACKET_SIZE);
#else
SX127xDriver Radio(RadioSpi, OTA_PACKET_SIZE);
#endif
CRSF_RX crsf(CrsfSerial); //pass a serial port object to the class for it to use

volatile connectionState_e connectionState = STATE_disconnected;
static volatile uint8_t NonceRXlocal = 0; // nonce that we THINK we are up to.
static volatile uint8_t TLMinterval = 0;
static volatile uint32_t tlm_check_ratio = 0;
static volatile uint32_t rx_last_valid_us = 0; //Time the last valid packet was recv
static volatile int32_t rx_freqerror = 0;
#if NUM_FAILS_TO_RESYNC
static volatile uint32_t rx_lost_packages = 0;
#endif
static volatile int32_t rx_hw_isr_running = 0;

static uint16_t DRAM_ATTR CRCCaesarCipher = 0;

#if SERVO_OUTPUTS_ENABLED
static volatile uint8_t update_servos = 0;
static volatile crsf_channels_t channels_servos = {0};
#endif

///////////////////////////////////////////////
////////////////  Filters  ////////////////////
static LPF LPF_FreqError(5);
static LPF LPF_UplinkRSSI(5);

//////////////////////////////////////////////////////////////
////// Variables for Telemetry and Link Quality //////////////

static volatile uint32_t LastValidPacket = 0; //Time the last valid packet was recv
static uint32_t SendLinkStatstoFCintervalNextSend = SEND_LINK_STATS_TO_FC_INTERVAL;
static mspPacket_t msp_packet_rx;
static mspPacket_t msp_packet_tx;
static volatile uint_fast8_t tlm_msp_send = 0;
static volatile uint_fast8_t uplink_Link_quality = 0;

///////////////////////////////////////////////////////////////
///////////// Variables for Sync Behaviour ////////////////////
static volatile uint32_t RFmodeNextCycle = 0; // set from isr
static uint32_t RFmodeCycleDelay = 0;
static uint8_t scanIndex = 0;
static uint8_t tentative_cnt = 0;
#if RX_UPDATE_AIR_RATE
static volatile uint32_t updatedAirRate = 0xff;
#endif

///////////////////////////////////////

static bool ledState = false;
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
    DEBUG_PRINT("TLM: ");
    DEBUG_PRINTLN(interval);
}
///////////////////////////////////////

void ICACHE_RAM_ATTR FillLinkStats()
{
    int8_t LastRSSI = Radio.LastPacketRSSI;
    //crsf.ChannelsPacked.ch15 = UINT10_to_CRSF(MAP(LastRSSI, -100, -50, 0, 1023));
    //crsf.ChannelsPacked.ch14 = UINT10_to_CRSF(MAP_U16(crsf.LinkStatistics.uplink_Link_quality, 0, 100, 0, 1023));
    int32_t rssiDBM = LPF_UplinkRSSI.update(LastRSSI);
    // our rssiDBM is currently in the range -128 to 98, but BF wants a value in the range
    // 0 to 255 that maps to -1 * the negative part of the rssiDBM, so cap at 0.
    if (rssiDBM > 0)
        rssiDBM = 0;
    crsf.LinkStatistics.uplink_RSSI_1 = -1 * rssiDBM; // to match BF
    crsf.LinkStatistics.uplink_SNR = Radio.LastPacketSNR * 10;
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
    DEBUG_PRINT(" > freqerror:");
    DEBUG_PRINT(freqerror);
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
    DEBUG_PRINT(" smooth:");
    DEBUG_PRINT(freqerror);
#endif

    if (abs(freqerror) > 100) // 120
    {
        FreqCorrection += freqerror;
        Radio.setPPMoffsetReg(freqerror, 0);
        retval = 1;
    }
#if PRINT_FREQ_ERROR
    //DEBUG_PRINT(" local:");
    //DEBUG_PRINT(FreqCorrection - (UID[4] + UID[5]));
#endif

    return retval;
}

uint8_t ICACHE_RAM_ATTR HandleFHSS()
{
    uint8_t fhss = 0, nonce = NonceRXlocal;
    if ((nonce % ExpressLRS_currAirRate->FHSShopInterval) == 0)
    {
        FHSSincCurrIndex();
        fhss = 1;
    }
    NonceRXlocal = ++nonce;
    return fhss;
}

void ICACHE_RAM_ATTR HandleSendTelemetryResponse(uint_fast8_t lq) // total ~79us
{
    DEBUG_PRINT(" X");
    // esp requires word aligned buffer
    uint32_t __tx_buffer[(OTA_PACKET_SIZE + sizeof(uint32_t) - 1) / sizeof(uint32_t)];
    uint8_t *tx_buffer = (uint8_t *)__tx_buffer;
    uint8_t index = OTA_PACKET_DATA;

    if ((tlm_msp_send == 1) && (msp_packet_tx.type == MSP_PACKET_TLM_OTA))
    {
        if (RcChannels_tlm_send(tx_buffer, msp_packet_tx, 0) || msp_packet_tx.error)
        {
            msp_packet_tx.reset();
            tlm_msp_send = 0;
        }
    }
    else
    {
        crsf.LinkStatistics.uplink_Link_quality = uplink_Link_quality;
        crsf.LinkStatisticsPack(tx_buffer);
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
}

void tx_done_cb(void)
{
    // Configure RX only next is not hopping time
    if (((NonceRXlocal + 1) % ExpressLRS_currAirRate->FHSShopInterval) != 0)
        Radio.RXnb(FHSSgetCurrFreq());
}

void ICACHE_RAM_ATTR HWtimerCallback(uint32_t us)
{
    //DEBUG_PRINT("H");
#if (DBG_PIN_TMR_ISR_FAST != UNDEF_PIN)
    digitalWriteFast(DBG_PIN_TMR_ISR_FAST, 1);
#endif
    rx_hw_isr_running = 1;
    uint_fast8_t fhss_config_rx = 0, tlm_send, tlm_ratio = tlm_check_ratio;
    uint32_t __rx_last_valid_us = rx_last_valid_us;
    rx_last_valid_us = 0;

#if PRINT_TIMER && PRINT_HW_ISR
    DEBUG_PRINT("HW us ");
    DEBUG_PRINT(us);
#endif

    /* do adjustment */
    if (__rx_last_valid_us)
    {
        int32_t diff_us = (int32_t)(us - __rx_last_valid_us);
#if PRINT_TIMER && PRINT_HW_ISR
        DEBUG_PRINT(" - rx ");
        DEBUG_PRINT(__rx_last_valid_us);
        DEBUG_PRINT(" = ");
        DEBUG_PRINT(diff_us);
#endif

        int32_t interval = ExpressLRS_currAirRate->interval;
        if (diff_us > (interval >> 1))
        {
            /* the timer was called too early */
            diff_us %= interval;
            diff_us -= interval;
        }

        /* Adjust the timer */
        if ((TIMER_OFFSET_LIMIT < diff_us) || (diff_us < 0))
            TxTimer.reset(diff_us - TIMER_OFFSET);

        //LQ_packetAck(); // RX ack
    }
    else
    {
        TxTimer.setTime(); // Reset timer interval
    }

    fhss_config_rx |= RadioFreqErrorCorr();
    fhss_config_rx |= HandleFHSS();

#if PRINT_TIMER && PRINT_HW_ISR
    //DEBUG_PRINT(" nonce ");
    //DEBUG_PRINT(NonceRXlocal);
#endif

    tlm_send = ((0 < tlm_ratio) && ((NonceRXlocal & tlm_ratio) == 0));
    uint_fast8_t lq = LQ_getlinkQuality();
    uplink_Link_quality = lq;
    LQ_nextPacket();

    if (tlm_send)
    {
#if (DBG_PIN_TMR_ISR_FAST != UNDEF_PIN)
        digitalWriteFast(DBG_PIN_TMR_ISR_FAST, 0);
#endif
        // Adds packet to LQ otherwise an artificial drop in LQ is seen due to sending TLM.
        LQ_packetAck();

        HandleSendTelemetryResponse(lq);
#if (DBG_PIN_TMR_ISR_FAST != UNDEF_PIN)
        digitalWriteFast(DBG_PIN_TMR_ISR_FAST, 1);
#endif
        fhss_config_rx = 0;
    }
#if NUM_FAILS_TO_RESYNC
    else if (!__rx_last_valid_us)
    {
        if (NUM_FAILS_TO_RESYNC < (++rx_lost_packages)) {
            // consecutive losts => trigger connection lost to resync
            LostConnection();
            DEBUG_PRINT("RESYNC!");
        }
    }
#endif

    if (fhss_config_rx) {
        Radio.RXnb(FHSSgetCurrFreq()); // 260us => 148us => ~67us
    }

#if (PRINT_TIMER && PRINT_HW_ISR) || PRINT_FREQ_ERROR
    //DEBUG_PRINTLN("");
    DEBUG_PRINT(" took ");
    DEBUG_PRINTLN(micros() - us);
#endif
    rx_hw_isr_running = 0;
#if (DBG_PIN_TMR_ISR_FAST != UNDEF_PIN)
    digitalWriteFast(DBG_PIN_TMR_ISR_FAST, 0);
#endif
}

void ICACHE_RAM_ATTR LostConnection()
{
    if (connectionState <= STATE_lost)
    {
        return; // Already disconnected
    }

    TxTimer.stop(); // Stop sync timer

    // Reset FHSS
    FHSSresetFreqCorrection();
    FHSSsetCurrIndex(0);
    LPF_FreqError.init(0);

    connectionState = STATE_lost; //set lost connection

    led_set_state(1);             // turn off led
    Radio.RXnb(GetInitialFreq()); // in conn lost state we always want to listen on freq index 0
    DEBUG_PRINTLN("lost conn");

    platform_connection_state(connectionState);
    RFmodeNextCycle = millis(); // make sure to stay on same freq during next sync search phase
}

void ICACHE_RAM_ATTR TentativeConnection(int32_t freqerror)
{
    /* Do initial freq correction */
    FreqCorrection += freqerror;
    Radio.setPPMoffsetReg(freqerror, 0);
    LPF_FreqError.init(freqerror);
    rx_freqerror = 0;
    rx_last_valid_us = 0;

    tentative_cnt = 0;
    connectionState = STATE_tentative;
    DEBUG_PRINTLN("tentative");
    TxTimer.start();     // Start local sync timer
    TxTimer.setTime(80); // Trigger isr right after reception
}

void ICACHE_RAM_ATTR GotConnection()
{
    connectionState = STATE_connected; //we got a packet, therefore no lost connection

    led_set_state(0); // turn on led
    DEBUG_PRINTLN("connected");

    platform_connection_state(connectionState);
}

void ICACHE_RAM_ATTR ProcessRFPacketCallback(uint8_t *rx_buffer)
{
    /* Processing takes:
        R9MM: ~160us
    */
    if (rx_hw_isr_running)
        // Skip if hw isr is triggered already (e.g. TX has some weird latency)
        return;

#if (DBG_PIN_RX_ISR_FAST != UNDEF_PIN)
    digitalWriteFast(DBG_PIN_RX_ISR_FAST, 1);
#endif

    //DEBUG_PRINT("I");
    const connectionState_e _conn_state = connectionState;
    const uint32_t current_us = Radio.LastPacketIsrMicros;
    const uint16_t crc = CalcCRC16(rx_buffer, OTA_PACKET_PAYLOAD, CRCCaesarCipher);
    const uint16_t crc_in = ((uint16_t)rx_buffer[OTA_PACKET_PAYLOAD] << 8) + rx_buffer[OTA_PACKET_PAYLOAD+1];
    const uint8_t type = RcChannels_packetTypeGet(rx_buffer);

#if PRINT_TIMER && PRINT_RX_ISR
    DEBUG_PRINT("RX us ");
    DEBUG_PRINT(current_us);
#endif

    if (crc_in != crc)
    {
#if (DBG_PIN_RX_ISR_FAST != UNDEF_PIN)
        digitalWriteFast(DBG_PIN_RX_ISR_FAST, 0);
#endif
        DEBUG_PRINT(" !");
        //DEBUG_PRINTLN(FHSSgetCurrFreq());
        return;
    }

    rx_freqerror = Radio.GetFrequencyError();

    rx_last_valid_us = current_us;
    LastValidPacket = millis();

    switch (type)
    {
        case UL_PACKET_SYNC:
        {
            DEBUG_PRINT(" S");
            ElrsSyncPacket_s const * const sync = (ElrsSyncPacket_s*)rx_buffer;

            if (sync->CRCCaesarCipher == CRCCaesarCipher)
            {
                if (_conn_state == STATE_disconnected || _conn_state == STATE_lost)
                {
                    TentativeConnection(rx_freqerror);
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
#if !SERVO_OUTPUTS_ENABLED && 1
                else
                {
                    // Send last command to FC if connected to keep it running
                    crsf.sendRCFrameToFC();
                }
#endif

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
            DEBUG_PRINT(" R");
            if (STATE_lost < _conn_state)
            {
#if SERVO_OUTPUTS_ENABLED
                RcChannels_channels_extract(rx_buffer, channels_servos);
                update_servos = 1;
#else // !SERVO_OUTPUTS_ENABLED
                RcChannels_channels_extract(rx_buffer, crsf.ChannelsPacked);
#if (DBG_PIN_RX_ISR_FAST != UNDEF_PIN)
                digitalWriteFast(DBG_PIN_RX_ISR_FAST, 0);
#endif
                crsf.sendRCFrameToFC();
#if (DBG_PIN_RX_ISR_FAST != UNDEF_PIN)
                digitalWriteFast(DBG_PIN_RX_ISR_FAST, 1);
#endif
#endif // SERVO_OUTPUTS_ENABLED
            }
            break;

        case UL_PACKET_MSP:
            DEBUG_PRINT(" M");
            RcChannels_tlm_receive(rx_buffer, msp_packet_rx);
            break;

        case UL_PACKET_UNKNOWN:
        default:
            /* Not a valid packet, ignore it */
            rx_last_valid_us = 0;
            rx_freqerror = 0;
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

#if NUM_FAILS_TO_RESYNC
    rx_lost_packages = 0;
#endif
    LQ_packetAck();
    FillLinkStats();

#if PRINT_TIMER && PRINT_RX_ISR
    DEBUG_PRINT(" took ");
    DEBUG_PRINTLN(micros() - current_us);
#endif
#if (DBG_PIN_RX_ISR_FAST != UNDEF_PIN)
    digitalWriteFast(DBG_PIN_RX_ISR_FAST, 0);
#endif
}

void forced_start(void)
{
    DEBUG_PRINTLN("Manual Start");
    Radio.RXnb(GetInitialFreq());
}

void forced_stop(void)
{
    Radio.StopContRX();
    TxTimer.stop();
}

static void SetRFLinkRate(uint8_t rate) // Set speed of RF link (hz)
{
    const expresslrs_mod_settings_t *const config = get_elrs_airRateConfig(rate);
    if (config == NULL || config == ExpressLRS_currAirRate)
        return; // No need to modify, rate is same

    // Stop ongoing actions before configuring next rate
    TxTimer.stop();
    Radio.StopContRX();

    ExpressLRS_currAirRate = config;
    current_rate_config = rate;

    DEBUG_PRINT("Set RF rate: ");
    DEBUG_PRINTLN(config->rate);

    // Init CRC aka LQ array
    LQ_reset();
    // Reset FHSS
    FHSSresetFreqCorrection();
    FHSSsetCurrIndex(0);

    RFmodeCycleDelay = config->syncSearchTimeout +
                       config->connectionLostTimeout;

    handle_tlm_ratio(config->TLMinterval);

    Radio.Config(config->bw, config->sf, config->cr, GetInitialFreq(),
                 config->PreambleLen, (OTA_PACKET_CRC == 0));

    // Measure RF noise
#ifdef DEBUG_SERIAL // TODO: Enable this when noize floor is used!
    int RFnoiseFloor = Radio.MeasureNoiseFloor(10, GetInitialFreq());
    DEBUG_PRINT("RF noise floor: ");
    DEBUG_PRINT(RFnoiseFloor);
    DEBUG_PRINTLN("dBm");
    (void)RFnoiseFloor;
#endif

    TxTimer.updateInterval(config->interval);
    LPF_FreqError.init(0);
    LPF_UplinkRSSI.init(0);
#if !SERVO_OUTPUTS_ENABLED
    crsf.LinkStatistics.uplink_RSSI_2 = 0;
    crsf.LinkStatistics.rf_Mode = config->rate_osd_num;
#endif
    Radio.RXnb();
    TxTimer.init();
}

/* FC sends v1 MSPs */
void msp_data_cb(uint8_t const *const input)
{
    if ((tlm_msp_send != 0) || (tlm_check_ratio == 0))
        return;

    /* process MSP packet from flight controller
     *  [0] header: seq&0xF,
     *  [1] payload size
     *  [2] function
     *  [3...] payload + crc
     */
    mspHeaderV1_t *hdr = (mspHeaderV1_t *)input;

    if (sizeof(msp_packet_tx.payload) < hdr->payloadSize)
        /* too big, ignore */
        return;

    msp_packet_tx.reset(hdr);
    msp_packet_tx.type = MSP_PACKET_TLM_OTA;
    if (0 < hdr->payloadSize)
        volatile_memcpy(msp_packet_tx.payload,
                        hdr->payload,
                        hdr->payloadSize);

    tlm_msp_send = 1; // rdy for sending
}

void setup()
{
#if (DBG_PIN_TMR_ISR_FAST != UNDEF_PIN)
    pinMode(DBG_PIN_TMR_ISR, OUTPUT);
    digitalWriteFast(DBG_PIN_TMR_ISR_FAST, 0);
#endif
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
    pinMode(DBG_PIN_RX_ISR, OUTPUT);
    digitalWriteFast(DBG_PIN_RX_ISR_FAST, 0);
#endif

    CRCCaesarCipher = CalcCRC16(UID, sizeof(UID), 0);

#if !SERVO_OUTPUTS_ENABLED
    CrsfSerial.Begin(CRSF_RX_BAUDRATE);
#endif

    msp_packet_rx.reset();
    msp_packet_tx.reset();

    DEBUG_PRINTLN("ExpressLRS RX Module...");
    platform_setup();

    //FHSSrandomiseFHSSsequence();

    // Prepare radio
#if defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
    Radio.RFmodule = RFMOD_SX1278;
#elif !RADIO_SX128x
    Radio.RFmodule = RFMOD_SX1276;
#endif
    Radio.SetPins(GPIO_PIN_RST, GPIO_PIN_DIO0, GPIO_PIN_DIO1, GPIO_PIN_DIO2,
                  GPIO_PIN_BUSY, GPIO_PIN_TX_ENABLE, GPIO_PIN_RX_ENABLE);
    Radio.SetSyncWord(getSyncWord());
    Radio.Begin();
    Radio.SetOutputPower(0b1111); // default RX to max power for tlm
    Radio.RXdoneCallback1 = ProcessRFPacketCallback;
    Radio.TXdoneCallback1 = tx_done_cb;

    // Set call back for timer ISR
    TxTimer.callbackTock = &HWtimerCallback;
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
    uint8_t rx_buffer_handle = 0;

#if RX_UPDATE_AIR_RATE
    /* update air rate config, timed to FHSS index 0 */
    if (updatedAirRate < get_elrs_airRateMax() && updatedAirRate != current_rate_config)
    {
        //connectionState = STATE_disconnected; // Force resync
        connectionState = STATE_lost; // Mark to lost to stay on received rate and force resync.
        SetRFLinkRate(updatedAirRate); // configure air rate
        RFmodeNextCycle = now;
        return;
    }
#endif /* RX_UPDATE_AIR_RATE */

    /* Cycle only if initial connection search */
    if (connectionState == STATE_disconnected)
    {
        if (RFmodeCycleDelay < (uint32_t)(now - RFmodeNextCycle))
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
    else if (connectionState == STATE_lost)
    {
        if (300 <= (uint32_t)(now - led_toggle_ms))
        {
            led_toggle();
            led_toggle_ms = now;
        }
    }
    else if (STATE_lost < connectionState)
    {
        // check if we lost conn.
        if (ExpressLRS_currAirRate->connectionLostTimeout <= (int32_t)(now - LastValidPacket))
        {
            LostConnection();
        }
        else if (connectionState == STATE_connected)
        {
#if SERVO_OUTPUTS_ENABLED
            if (update_servos)
                servo_out_write(channels_servos);
#else
            if (SEND_LINK_STATS_TO_FC_INTERVAL <= (uint32_t)(now - SendLinkStatstoFCintervalNextSend))
            {
                crsf.LinkStatistics.uplink_Link_quality = uplink_Link_quality;
                crsf.LinkStatisticsSend();
                SendLinkStatstoFCintervalNextSend = now;
            }
            else if (msp_packet_rx.iterated() || msp_packet_rx.error)
            {
                // MPS packet received, handle it
                if (!msp_packet_rx.error)
                {
                    // TODO: Check if packet is for receiver
                    crsf.sendMSPFrameToFC(msp_packet_rx);
                }
                msp_packet_rx.reset();
            }
#endif
        }
    }

#if !SERVO_OUTPUTS_ENABLED
    crsf.handleUartIn(rx_buffer_handle);
#endif

    platform_loop(connectionState);

    platform_wd_feed();
}
