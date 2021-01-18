#include <Arduino.h>
#include "utils.h"
#include "common.h"
#include "FHSS.h"
#include "targets.h"
#include "POWERMGNT.h"
#include "HwTimer.h"
#include "debug_elrs.h"
#include "LowPassFilter.h"
#include "tx_common.h"
#include <stdlib.h>

#if (GPIO_PIN_RCSIGNAL_RX != UNDEF_PIN) || (GPIO_PIN_RCSIGNAL_TX != UNDEF_PIN)
#include "CRSF_TX.h"
extern CRSF_TX crsf;
#endif

//// CONSTANTS ////
#define RX_CONNECTION_LOST_TIMEOUT  1500U // After 1500ms of no TLM response consider that slave has lost connection
#define TX_POWER_UPDATE_PERIOD      1500U

///////////////////
/// define some libs to use ///
POWERMGNT DRAM_FORCE_ATTR PowerMgmt;

static uint32_t DRAM_ATTR _rf_rxtx_counter;
static uint8_t DMA_ATTR rx_buffer[OTA_PACKET_SIZE];
static volatile uint8_t DRAM_ATTR rx_buffer_handle;
static uint8_t red_led_state;

static uint16_t DRAM_ATTR CRCCaesarCipher;

static struct platform_config pl_config;

/////////// SYNC PACKET ////////
static uint32_t DRAM_ATTR SyncPacketNextSend;
static uint32_t DRAM_ATTR sync_send_interval; // Default is send always

/////////// CONNECTION /////////
static uint32_t DRAM_ATTR LastPacketRecvMillis;
connectionState_e DRAM_ATTR connectionState;

//////////// TELEMETRY /////////
static uint32_t DRAM_ATTR TLMinterval;
static uint32_t DRAM_ATTR tlm_check_ratio;
static uint32_t DRAM_ATTR expected_tlm_counter;
static uint32_t DRAM_ATTR recv_tlm_counter;

static LPF DRAM_ATTR LPF_dyn_tx_power(3);
static uint32_t DRAM_ATTR dyn_tx_updated;

static MSP msp_packet_parser;

mspPacket_t msp_packet_tx;
mspPacket_t msp_packet_rx;
uint8_t DRAM_ATTR tlm_msp_send, tlm_msp_rcvd;

LinkStats_t DRAM_ATTR LinkStatistics;
GpsOta_t DRAM_ATTR GpsTlm;


static uint8_t SetRadioType(uint8_t type);
static void SendRCdataToRF(uint32_t const current_us);
static uint8_t SetRFLinkRate(uint8_t rate, uint8_t init=0);


///////////////////////////////////////

void tx_common_init_globals(void) {
    uint8_t UID[6] = {MY_UID};

    current_rate_config = RATE_DEFAULT;

    pl_config.key = 0;
    for (uint8_t iter = 0; iter < ARRAY_SIZE(pl_config.rf); iter++) {
        pl_config.rf[iter].mode = RATE_DEFAULT;
        pl_config.rf[iter].power = TX_POWER_DEFAULT;
        pl_config.rf[iter].tlm = TLM_RATIO_DEFAULT;
    }

    // Default to 127x if both defined
#if RADIO_SX127x
    pl_config.rf_mode = RADIO_TYPE_127x;
#elif RADIO_SX128x
    pl_config.rf_mode = RADIO_TYPE_128x;
#endif

    connectionState = STATE_disconnected;
    CRCCaesarCipher = CalcCRC16(UID, sizeof(UID), 0);

    msp_packet_tx.reset();
    msp_packet_rx.reset();
}

void tx_common_init(void)
{
    platform_config_load(pl_config);
    TxTimer.callbackTock = &SendRCdataToRF;
    SetRadioType(pl_config.rf_mode);
    SetRFLinkRate(current_rate_config, 1);

#ifdef CTRL_SERIAL
    uint8_t resp[5], outlen = sizeof(resp);
    if (0 == SettingsCommandHandle(0, resp, 0, outlen))
        msp_packet_parser.sendPacket(
            &ctrl_serial, MSP_PACKET_V1_ELRS,
            ELRS_INT_MSP_PARAMS, MSP_ELRS_INT, outlen, resp);
#endif /* CTRL_SERIAL */
}

///////////////////////////////////////

static void ICACHE_RAM_ATTR ProcessTLMpacket(uint8_t *buff, uint32_t rx_us);
static void ICACHE_RAM_ATTR HandleTLM();

uint8_t tx_tlm_change_interval(uint8_t value, uint8_t init = 0)
{
    uint32_t ratio = 0;
    if (value == TLM_RATIO_DEFAULT)
    {
        // Default requested
        value = ExpressLRS_currAirRate->TLMinterval;
    }
    else if (TLM_RATIO_MAX <= value)
    {
        DEBUG_PRINTF("TLM: Invalid value! disable tlm\n");
        value = TLM_RATIO_NO_TLM;
    }

    if (value != TLMinterval || init)
    {
        if (TLM_RATIO_NO_TLM < value) {
            Radio->RXdoneCallback1 = ProcessTLMpacket;
            Radio->TXdoneCallback1 = HandleTLM;
            connectionState = STATE_disconnected;
            ratio = TLMratioEnumToValue(value);
            DEBUG_PRINTF("TLM ratio %u\n", ratio);
            ratio -= 1;
        } else {
            Radio->RXdoneCallback1 = Radio->rx_nullCallback;
            Radio->TXdoneCallback1 = Radio->tx_nullCallback;
            // Set connected if telemetry is not used
            connectionState = STATE_connected;
            DEBUG_PRINTF("TLM disabled\n");
        }
        write_u32(&TLMinterval, value);
        write_u32(&tlm_check_ratio, ratio);
        return 1;
    }
    return 0;
}

void tx_tlm_disable_enable(uint8_t enable)
{
    if (enable)
        tx_tlm_change_interval(TLM_RATIO_DEFAULT);
    else
        tx_tlm_change_interval(TLM_RATIO_NO_TLM);
}

int8_t tx_tlm_toggle(void)
{
    /* Toggle TLM between NO_TLM and DEFAULT */
    uint8_t tlm = (TLMinterval == TLM_RATIO_NO_TLM) ? TLM_RATIO_DEFAULT : TLM_RATIO_NO_TLM;
    tx_tlm_change_interval(tlm);
    return (TLMinterval != TLM_RATIO_NO_TLM);
}

///////////////////////////////////////

static void stop_processing(void)
{
    uint32_t irq = _SAVE_IRQ();
    TxTimer.stop();
    Radio->StopContRX();
    _RESTORE_IRQ(irq);
}

void platform_radio_force_stop(void)
{
    uint32_t irq = _SAVE_IRQ();
    TxTimer.stop();
    Radio->End();
    _RESTORE_IRQ(irq);
}

static uint8_t SetRadioType(uint8_t type)
{
    /* Configure if ratio not set or its type will be changed */
    if ((type < RADIO_TYPE_MAX) && (type != pl_config.rf_mode || !Radio)) {
        /* Stop radio processing if chaning RF type */
        if (Radio)
            stop_processing();
        Radio = common_config_radio(type);

        current_rate_config =
            pl_config.rf[type].mode % get_elrs_airRateMax();
        TLMinterval = pl_config.rf[type].tlm;
        PowerLevels_e power =
            (PowerLevels_e)(pl_config.rf[type].power % PWR_UNKNOWN);
        platform_mode_notify(get_elrs_airRateMax() - current_rate_config);

#if defined(TARGET_R9M_TX) && !defined(R9M_LITE_TX)
        PowerMgmt.Begin(Radio, &r9dac);
#else
        PowerMgmt.Begin(Radio);
#endif
        PowerMgmt.setPower(power);
        pl_config.rf_mode = type;

        return 1;
    }
    return 0;
}

///////////////////////////////////////

void process_rx_buffer(void)
{
    const uint32_t ms = millis();
    const uint16_t crc = CalcCRC16((uint8_t*)rx_buffer, OTA_PACKET_DATA, CRCCaesarCipher);
    const uint16_t crc_in = ((uint16_t)rx_buffer[OTA_PACKET_DATA] << 8) + rx_buffer[OTA_PACKET_DATA+1];
    const uint8_t  type = RcChannels_packetTypeGet((uint8_t*)rx_buffer);

    if (crc_in != crc)
    {
        DEBUG_PRINTF("!C");
        return;
    }

    //DEBUG_PRINTF(" PROC_RX ");

    connectionState = STATE_connected;
    platform_connection_state(STATE_connected);
    platform_set_led(0);
    LastPacketRecvMillis = ms;
    recv_tlm_counter++;

    switch (type)
    {
        case DL_PACKET_TLM_MSP:
        {
            //DEBUG_PRINTF("DL MSP junk\n");
            if (RcChannels_tlm_ota_receive((uint8_t*)rx_buffer, msp_packet_rx))
                tlm_msp_rcvd = 1;
            break;
        }
        case DL_PACKET_TLM_LINK:
        {
            RcChannels_link_stas_extract((uint8_t*)rx_buffer, LinkStatistics,
                                         Radio->LastPacketSNR,
                                         Radio->LastPacketRSSI);

            // Check RSSI and update TX power if needed
            int8_t rssi = LPF_dyn_tx_power.update((int8_t)LinkStatistics.link.uplink_RSSI_1);
            if (TX_POWER_UPDATE_PERIOD <= (ms - dyn_tx_updated)) {
                dyn_tx_updated = ms;
                if (-75 < rssi) {
                    PowerMgmt.decPower();
                } else if (-95 > rssi) {
                    PowerMgmt.incPower();
                }
            }
            break;
        }
        case DL_PACKET_GPS:
        {
            RcChannels_gps_extract((uint8_t*)rx_buffer, GpsTlm);
            break;
        }
        case DL_PACKET_FREE1:
        default:
            break;
    }
}

static void ICACHE_RAM_ATTR ProcessTLMpacket(uint8_t *buff, uint32_t rx_us)
{
    if (buff) {
        (void)rx_us;
        memcpy(rx_buffer, buff, sizeof(rx_buffer));
        rx_buffer_handle = 1;

        //DEBUG_PRINTF(" R ");
    }
}

static void ICACHE_RAM_ATTR HandleTLM()
{
    //DEBUG_PRINTF("X ");
    uint32_t const tlm_ratio = tlm_check_ratio;
    if (tlm_ratio && (_rf_rxtx_counter & tlm_ratio) == 0)
    {
        // receive tlm package
        PowerMgmt.pa_off();
        Radio->RXnb(FHSSgetCurrFreq());
        expected_tlm_counter++;
        //DEBUG_PRINTF(" RX ");
    }
}

///////////////////////////////////////

static void ICACHE_RAM_ATTR
GenerateSyncPacketData(uint8_t *const output, uint32_t rxtx_counter)
{
    ElrsSyncPacket_s * sync = (ElrsSyncPacket_s*)output;
    sync->fhssIndex = FHSSgetCurrIndex();
    sync->rxtx_counter = rxtx_counter;
#if RX_UPDATE_AIR_RATE
    sync->air_rate = current_rate_config;
#endif
    sync->tlm_interval = TLMinterval;
    sync->CRCCaesarCipher = CRCCaesarCipher;
    sync->pkt_type = UL_PACKET_SYNC;
}

static void ICACHE_RAM_ATTR SendRCdataToRF(uint32_t const current_us)
{
    // Called by HW timer
    uint32_t freq;
    uint32_t const rxtx_counter = _rf_rxtx_counter;
    uint32_t const tlm_ratio = tlm_check_ratio;
    // esp requires word aligned buffer
    uint32_t __tx_buffer[(OTA_PACKET_SIZE + sizeof(uint32_t) - 1) / sizeof(uint32_t)];
    uint8_t * const tx_buffer = (uint8_t *)__tx_buffer;
    uint16_t crc;
    uint8_t index = OTA_PACKET_DATA, arm_state = RcChannels_get_arm_channel_state();

    // Check if telemetry RX ongoing
    if (tlm_ratio && (rxtx_counter & tlm_ratio) == 0) {
        // Skip TX because TLM RX is ongoing
        goto send_to_rf_exit;
    }

    freq = FHSSgetCurrFreq();

    //only send sync when its time and only on channel 0;
    if (!arm_state && (FHSSgetCurrSequenceIndex() == 0) &&
        (sync_send_interval <= (uint32_t)(current_us - SyncPacketNextSend)))
    {
        GenerateSyncPacketData(tx_buffer, rxtx_counter);
        SyncPacketNextSend = current_us;
    }
    else if (!arm_state && (tlm_msp_send == 1) && (msp_packet_tx.type == MSP_PACKET_TLM_OTA))
    {
        /* send tlm packet if needed */
        if (RcChannels_tlm_ota_send(tx_buffer, msp_packet_tx) || msp_packet_tx.error) {
            msp_packet_tx.reset();
            tlm_msp_send = 0;
        }
    }
    else
    {
        RcChannels_get_packed_data(tx_buffer);
    }

#if OTA_PACKET_10B
    tx_buffer[index++] = FHSSgetCurrIndex();
    tx_buffer[index++] = rxtx_counter;
#endif // OTA_PACKET_10B

    // Calculate the CRC
    crc = CalcCRC16(tx_buffer, index, CRCCaesarCipher);
    tx_buffer[index++] = (crc >> 8);
    tx_buffer[index++] = (crc & 0xFF);
    // Enable PA
    PowerMgmt.pa_on();
    // Debugging
    //delayMicroseconds(random(0, 400)); // 300 ok
    //if (random(0, 99) < 55) tx_buffer[1] = 0;
    // Send data to rf
    Radio->TXnb(tx_buffer, index, freq);

send_to_rf_exit:
    // Check if hopping is needed
    if ((rxtx_counter % ExpressLRS_currAirRate->FHSShopInterval) == 0) {
        // it is time to hop
        FHSSincCurrIndex();
    }
    // Increase TX counter
    _rf_rxtx_counter++;

    //DEBUG_PRINTF(" T");
}

///////////////////////////////////////

int8_t SettingsCommandHandle(uint8_t const *in, uint8_t *out, uint8_t const inlen, uint8_t & outlen)
{
    uint8_t const cmd = in[0];
    uint8_t value = in[1];
    uint8_t modified = 0;

    switch (cmd)
    {
        case 0: // send all params
            break;

        case 1:
            if (inlen != 2)
                return -1;

            // set air rate
            if (get_elrs_airRateMax() > value)
                modified |= (SetRFLinkRate(value) << 1);
            DEBUG_PRINTF("Rate: %u\n", ExpressLRS_currAirRate->rate);
            break;

        case 2:
            // set TLM interval
            if (inlen != 2)
                return -1;
            modified = (tx_tlm_change_interval(value) << 2);
            break;

        case 3:
            // set TX power
            if (inlen != 2)
                return -1;
            modified = PowerMgmt.currPower();
            PowerMgmt.setPower((PowerLevels_e)value);
            DEBUG_PRINTF("Power: %u\n", PowerMgmt.currPower());
            modified = (modified != PowerMgmt.currPower()) ? (1 << 3) : 0;
            break;

        case 4:
            // RFFreq
            if (inlen != 2)
                return -1;
            value = common_config_get_radio_type(value);
            modified |= (SetRadioType(value) << 1);
            break;

        case 5:
            // Start WiFi
            platform_radio_force_stop();
            platform_wifi_start();
            break;

        case 6:
            // RF power (TEST feature!)
            if (inlen != 2)
                return -1;
            DEBUG_PRINTF("RF Power: %u\n", value);
            Radio->SetOutputPower(value);
            break;

        default:
            return -1;
    }

    // Fill response
    if (out && 5 <= outlen) {
        //out[0] = get_elrs_airRateIndex((void*)ExpressLRS_currAirRate);
        out[0] = (uint8_t)current_rate_config;
        out[1] = (uint8_t)TLMinterval;
        out[2] = (uint8_t)PowerMgmt.currPower();
        out[3] = (uint8_t)PowerMgmt.maxPowerGet();
        out[4] = RADIO_RF_MODE_INVALID;
        if (RADIO_SX127x && pl_config.rf_mode == RADIO_TYPE_127x) {
#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_FCC_915)
            out[4] = RADIO_RF_MODE_915_AU_FCC;
#elif defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_EU_868_R9)
            out[4] = RADIO_RF_MODE_868_EU;
#elif defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
            out[4] = RADIO_RF_MODE_433_AU_EU;
#endif
        } else if (RADIO_SX128x && pl_config.rf_mode == RADIO_TYPE_128x) {
            out[4] = RADIO_RF_MODE_2400_ISM_500Hz;
        }
    }

    if (modified)
    {
        // Stop timer before save if not already done
        if ((modified & (1 << 1)) == 0) {
            stop_processing();
        }

        // Save modified values
        uint8_t type = pl_config.rf_mode;
        pl_config.key = ELRS_EEPROM_KEY;
        pl_config.rf[type].mode = current_rate_config;
        pl_config.rf[type].power = PowerMgmt.currPower();
        pl_config.rf[type].tlm = TLMinterval;
        platform_config_save(pl_config);

        // and restart timer
        TxTimer.start();
    }

#ifdef CTRL_SERIAL
    if (out) {
        msp_packet_parser.sendPacket(
            &ctrl_serial, MSP_PACKET_V1_ELRS, ELRS_INT_MSP_PARAMS,
            MSP_ELRS_INT, outlen, out);
    }
#endif /* CTRL_SERIAL */

    return 0;
}



///////////////////////////////////////
uint8_t SetRFLinkRate(uint8_t rate, uint8_t init) // Set speed of RF link (hz)
{
    const expresslrs_mod_settings_t *const config = get_elrs_airRateConfig(rate);
    if (config == NULL || config == ExpressLRS_currAirRate)
        return 0; // No need to modify, rate is same

    if (!init) {
        // Stop timer and put radio into sleep
        stop_processing();
    }

    LPF_dyn_tx_power.init(-55);

    current_rate_config = rate;
    ExpressLRS_currAirRate = config;
    TxTimer.updateInterval(config->interval);

    FHSSsetCurrIndex(0);
    Radio->Config(config->bw, config->sf, config->cr, GetInitialFreq(),
                  config->PreambleLen, (OTA_PACKET_CRC == 0));

    //TODO: move...
#if (GPIO_PIN_RCSIGNAL_RX != UNDEF_PIN) || (GPIO_PIN_RCSIGNAL_TX != UNDEF_PIN)
    crsf.setRcPacketRate(config->interval);
#endif
    LinkStatistics.link.rf_Mode = config->rate_osd_num;

    tx_tlm_change_interval(TLMinterval, init);

    write_u32(&sync_send_interval, config->syncInterval);

    platform_connection_state(connectionState);

    return 1;
}

void hw_timer_init(void)
{
    red_led_state = 1;
    platform_set_led(1);
    TxTimer.init();
    TxTimer.start();
}

void hw_timer_stop(void)
{
    red_led_state = 0;
    platform_set_led(0);
    platform_radio_force_stop();
}


#ifdef CTRL_SERIAL
static void MspOtaCommandsSend(mspPacket_t &packet)
{
    uint16_t iter;

    if (read_u8(&tlm_msp_send) || (TLM_RATIO_NO_TLM == TLMinterval)) {
        DEBUG_PRINTF("MSP TX packet ignored\n");
        return;
    }

    msp_packet_tx.reset();
    msp_packet_tx.type = MSP_PACKET_TLM_OTA;
    msp_packet_tx.flags = packet.flags;
    msp_packet_tx.function = packet.function;
    // Convert to CRSF encapsulated MSP message
    msp_packet_tx.addByte(packet.payloadSize);
    msp_packet_tx.addByte(packet.function);
    for (iter = 0; iter < packet.payloadSize; iter++) {
        msp_packet_tx.addByte(packet.payload[iter]);
    }
    msp_packet_tx.addByte(msp_packet_tx.crc);
    msp_packet_tx.setIteratorToSize();

    if (!msp_packet_tx.error)
        write_u8(&tlm_msp_send, 1); // rdy for sending
}
#endif

void tx_common_handle_rx_buffer(void)
{
    if (rx_buffer_handle) {
        process_rx_buffer();
        rx_buffer_handle = 0;
        platform_wd_feed();
    }
}

int tx_common_has_telemetry(void)
{
    return (TLM_RATIO_NO_TLM < TLMinterval) ? 0 : -1;
}

int tx_common_check_connection(void)
{
    if (0 <= tx_common_has_telemetry() && STATE_lost < connectionState) {
        uint32_t current_ms = millis();
        if (RX_CONNECTION_LOST_TIMEOUT < (uint32_t)(current_ms - LastPacketRecvMillis)) {
            connectionState = STATE_disconnected;
            platform_connection_state(STATE_disconnected);
            platform_set_led(red_led_state);
            return -1;
        }
    }
    return 0;
}

void tx_common_handle_ctrl_serial(void)
{
#ifdef CTRL_SERIAL
    if (ctrl_serial.available()) {
        platform_wd_feed();
        uint8_t in = ctrl_serial.read();
        if (msp_packet_parser.processReceivedByte(in)) {
            //  MSP received, check content
            mspPacket_t &packet = msp_packet_parser.getPacket();

            //DEBUG_PRINTF("MSP rcvd, type:%u\n", packet.type);

            /* Check if packet is ELRS internal */
            if ((packet.type == MSP_PACKET_V1_ELRS) && (packet.flags & MSP_ELRS_INT)) {
                switch (packet.function) {
                    case ELRS_INT_MSP_PARAMS: {
                        uint8_t * msg = (uint8_t*)packet.payload;
                        uint8_t outlen = packet.payloadSize;
                        if (0 == SettingsCommandHandle(msg, msg, packet.payloadSize, outlen)) {
                            //packet.type = MSP_PACKET_V1_ELRS;
                            packet.payloadSize = outlen;
                            msp_packet_parser.sendPacket(&packet, &ctrl_serial);
                        }
                        break;
                    }
                    default:
                        break;
                };
            } else {
                MspOtaCommandsSend(packet);
            }
            msp_packet_parser.markPacketFree();
        }
    }
#endif
}

void tx_common_update_link_stats(void)
{
    // Calc LQ based on good tlm packets and receptions done
    uint8_t const rx_cnt = read_u32(&expected_tlm_counter);
    write_u32(&expected_tlm_counter, 0);
    uint32_t const tlm_cnt = recv_tlm_counter;
    recv_tlm_counter = 0; // Clear RX counter

    if (rx_cnt)
        LinkStatistics.link.downlink_Link_quality = (tlm_cnt * 100u) / rx_cnt;
    else
        // failure value??
        LinkStatistics.link.downlink_Link_quality = 0;

    LinkStatistics.link.uplink_TX_Power = PowerMgmt.power_to_radio_enum();
}
