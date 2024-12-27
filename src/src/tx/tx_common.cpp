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
#if TARGET_HANDSET && OTA_VANILLA_ENABLED
#include "OTAvanilla.h"
#else
#define OTA_VERSION_ID 0
#endif
#include <stdlib.h>

//// CONSTANTS ////
#define RX_CONNECTION_LOST_TIMEOUT 1500U // After 1500ms of no TLM response consider that slave has lost connection
#define TX_POWER_UPDATE_PERIOD     1500U

///////////////////
/// define some libs to use ///
POWERMGNT DRAM_FORCE_ATTR PowerMgmt(GPIO_PIN_FAN_CTRL);

static uint32_t DRAM_ATTR _rf_rxtx_counter;
static uint8_t DMA_ATTR rx_buffer[OTA_PAYLOAD_MAX];
static uint8_t DRAM_ATTR rx_buffer_size;
static uint8_t red_led_state;

static uint16_t DRAM_ATTR CRCCaesarCipher;
static uint32_t DRAM_ATTR SyncCipher;

struct platform_config DRAM_ATTR pl_config;

/////////// SYNC PACKET ////////
static uint32_t DRAM_ATTR SyncPacketSent_us;
static uint32_t DRAM_ATTR SyncPacketInterval_us; // Default is send always

/////////// CONNECTION /////////
static uint32_t DRAM_ATTR LastPacketRecvMillis;
int8_t DRAM_ATTR connectionState;

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
uint8_t DRAM_ATTR tlm_msp_send;
uint32_t DRAM_ATTR tlm_msp_sent_ms;
uint8_t DRAM_ATTR tlm_msp_rcvd;

LinkStats_t DRAM_ATTR LinkStatistics;
GpsOta_t DRAM_ATTR GpsTlm;
DeviceInfo_t DRAM_ATTR DevInfo;
uint32_t tlm_updated;

static uint8_t SetRadioType(uint8_t type);
static void SendRCdataToRF(uint32_t const current_us);
static uint8_t SetRFLinkRate(uint8_t rate, uint8_t init = 0);

// static struct gpio_out debug_pin_tx;

///////////////////////////////////////

void tx_common_init_globals(void)
{
    current_rate_config = RATE_DEFAULT;

    pl_config.key = 0;
    for (uint8_t iter = 0; iter < ARRAY_SIZE(pl_config.rf); iter++) {
        pl_config.rf[iter].mode = RATE_DEFAULT;
        pl_config.rf[iter].power = TX_POWER_DEFAULT;
#if SERIAL_PASSTHROUGH && SERIAL_BIDIR
        pl_config.rf[iter].tlm = TLM_RATIO_1_2;
#else
        pl_config.rf[iter].tlm = get_elrs_default_tlm_interval(iter, RATE_DEFAULT);
#endif
    }

    // Default to 127x if both defined
#if RADIO_SX127x
    pl_config.rf_mode = RADIO_TYPE_127x;
#elif RADIO_SX128x
    pl_config.rf_mode = RADIO_TYPE_128x;
#endif

    connectionState = STATE_disconnected;

    msp_packet_tx.reset();
    msp_packet_rx.reset();

    tlm_updated = TLM_UPDATES_NA;

    // debug_pin_tx = gpio_out_setup(PC12, 0);
}

void tx_common_init(void)
{
#if defined(LATEST_COMMIT)
    uint8_t commit_sha[] = {LATEST_COMMIT};
    DEBUG_PRINTF("Current version (SHA): ");
    for (uint8_t iter = 0; iter < sizeof(commit_sha); iter++) {
        DEBUG_PRINTF("%X", commit_sha[iter]);
    }
#if LATEST_COMMIT_DIRTY
    DEBUG_PRINTF("-dirty");
#endif
    DEBUG_PRINTF("\n");
#endif
    my_uid_print();

    platform_config_load(pl_config);
    TxTimer.callbackTick = &SendRCdataToRF;
    if (!SetRadioType(pl_config.rf_mode)) {
#if RADIO_SX127x && RADIO_SX128x
        /* Roll back to other module if first failed */
        pl_config.rf_mode = (pl_config.rf_mode == RADIO_TYPE_127x) ? RADIO_TYPE_128x : RADIO_TYPE_127x;
        if (!SetRadioType(pl_config.rf_mode))
#endif
            /* Infinite loop in case of failure */
            while (1) {
                /* TODO: blink led... */
                DEBUG_PRINTF("RADIO CONFIG ERROR!\n");
                delay(1000);
            }
    }
    SetRFLinkRate(current_rate_config, 1);
    // delay(10);

#ifdef CTRL_SERIAL
    uint8_t resp[2] = {0, 0};
    SettingsCommandHandle(resp, NULL, 0, resp[1]);
#endif /* CTRL_SERIAL */
}

///////////////////////////////////////

static void PacketReceivedCallback(uint8_t * buff, uint32_t rx_us, size_t payloadSize, int32_t freq_err);
static void TransmissionCompletedCallback();

uint8_t tx_tlm_value_validate(uint8_t const value)
{
    if (TLM_RATIO_MAX <= value) {
        return TLM_RATIO_NO_TLM;
    }
    if (value == TLM_RATIO_DEFAULT) {
        // reset to default value
        return ExpressLRS_currAirRate->TLMinterval;
    }
    return value;
}

void tx_tlm_change_interval(uint8_t const value, uint8_t const init = 0)
{
    uint32_t ratio = 0;
    if (value == TLMinterval && !init) {
        return;
    }
    if (TLM_RATIO_NO_TLM < value) {
        Radio->RXdoneCallback1 = PacketReceivedCallback;
        Radio->TXdoneCallback1 = TransmissionCompletedCallback;
        connectionState = STATE_disconnected;
        ratio = TlmEnumToMask(value);
    } else {
        Radio->RXdoneCallback1 = Radio->rx_nullCallback;
        Radio->TXdoneCallback1 = Radio->tx_nullCallback;
        // Set connected if telemetry is not used
        connectionState = STATE_connected;
    }
    TLMinterval = value;
    tlm_check_ratio = ratio;
}

int8_t tx_tlm_toggle(void)
{
    /* Toggle TLM between NO_TLM and DEFAULT */
    uint8_t const tlm = (TLMinterval == TLM_RATIO_NO_TLM) ? pl_config.rf[pl_config.rf_mode].tlm : TLM_RATIO_NO_TLM;
    tx_tlm_change_interval(tlm);
    return (TLMinterval != TLM_RATIO_NO_TLM);
}

#if defined(TX_TLM_WHEN_DISARMED)
void FAST_CODE_1 tx_common_armed_state(uint8_t const armed)
{
    uint8_t next_tlm_value = (armed) ? pl_config.rf[pl_config.rf_mode].tlm : TLM_RATIO_1_4;
    tx_tlm_change_interval(next_tlm_value);
}
#endif

uint_fast8_t FORCED_INLINE tx_common_frame_frame_is_tlm(void)
{
    uint32_t const tlm_ratio = tlm_check_ratio;
    return TlmFrameCheck(_rf_rxtx_counter, tlm_ratio) && !FHSScurrSequenceIndexIsSyncChannel();
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

static uint8_t SetRadioType(uint8_t const type)
{
    uint32_t const uid_u32 = my_uid_to_u32();
    /* Configure if ratio not set or its type will be changed */
    if ((type < RADIO_TYPE_MAX) && (type != pl_config.rf_mode || !Radio)) {
        /* Stop radio processing if chaning RF type */
        if (Radio)
            platform_radio_force_stop();
        RadioInterface * new_radio = common_config_radio(type);
        if (!new_radio) {
            return 0;
        }
        Radio = new_radio;

        current_rate_config = pl_config.rf[type].mode % get_elrs_airRateMax();
        if (type == RADIO_TYPE_128x_VANILLA) {
            CRCCaesarCipher = (uid_u32 & 0xffff) ^ OTA_VERSION_ID;
            // Sync word must be set To make IQ inversion correct
            Radio->SetSyncWord(uid_u32 & 0xff);
            Radio->SetSyncWordLong(my_uid_to_u32(OTA_VERSION_ID));
        } else {
            CRCCaesarCipher = my_uid_crc16();
            SyncCipher = (CRCCaesarCipher ^ uid_u32) & SYNC_CIPHER_MASK;
            Radio->SetSyncWord(my_uid_crc8());
            Radio->SetSyncWordLong(my_uid_crc32());
        }

        PowerLevels_e const power = (PowerLevels_e)(pl_config.rf[type].power % PWR_UNKNOWN);
        platform_mode_notify(get_elrs_airRateMax() - current_rate_config);

        DEBUG_PRINTF("SetRadioType type:%u, rate:%u, tlm:%u, pwr:%u, cipher:0x%X\n", type, current_rate_config,
                     pl_config.rf[type].tlm, power, CRCCaesarCipher);

#if DAC_IN_USE
        PowerMgmt.Begin(Radio, &r9dac);
#else  /* !DAC_IN_USE */
        PowerMgmt.Begin(Radio);
#endif /* DAC_IN_USE */
        PowerMgmt.setPower(power);
        pl_config.rf_mode = type;

        return 1;
    }
    DEBUG_PRINTF("SetRadioType skipped (type:%u)\n", type);
    return 0;
}

///////////////////////////////////////

void process_rx_buffer(uint8_t payloadSize)
{
    const uint32_t ms = millis();
    if (ExpressLRS_currAirRate->hwCrc != HWCRC_EN) {
        payloadSize -= OTA_PACKET_CRC;

        const uint16_t crc = CalcCRC16((uint8_t *)rx_buffer, payloadSize, CRCCaesarCipher);
        const uint16_t crc_in = ((uint16_t)rx_buffer[payloadSize] << 8) + rx_buffer[payloadSize + 1];

        if (crc_in != crc) {
            DEBUG_PRINTF("!C");
            return;
        }
    }

    // DEBUG_PRINTF(" PROC_RX ");

    connectionState = STATE_connected;
    platform_connection_state(STATE_connected);
    platform_set_led(0);
    LastPacketRecvMillis = ms;
    recv_tlm_counter++;

#if !SERIAL_PASSTHROUGH
    switch (RcChannels_packetTypeGet((uint8_t *)rx_buffer, payloadSize)) {
        case DL_PACKET_TLM_MSP: {
            if (RcChannels_tlm_ota_receive((uint8_t *)rx_buffer, msp_packet_rx))
                tlm_msp_rcvd = 1;
            break;
        }
        case DL_PACKET_TLM_LINK: {
            RcChannels_link_stas_extract((uint8_t *)rx_buffer, LinkStatistics, (int8_t)read_u8(&Radio->LastPacketSNR),
                                         (int16_t)read_u16(&Radio->LastPacketRSSI));
            tlm_updated |= (TLM_UPDATES_LNK_STATS | TLM_UPDATES_BATTERY);

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
        case DL_PACKET_GPS: {
            RcChannels_gps_extract((uint8_t *)rx_buffer, GpsTlm);
            tlm_updated |= TLM_UPDATES_GPS;
            break;
        }
        case DL_PACKET_DEV_INFO: {
            if (RcChannels_dev_info_extract((uint8_t *)rx_buffer, DevInfo))
                tlm_updated |= TLM_UPDATES_DEV_INFO;
            break;
        }
        default:
            break;
    }
#else
#warning "TODO: handle tlm data and push to serial"
#endif
}

static void FAST_CODE_1 PacketReceivedCallback(uint8_t const * buff,
                                               uint32_t const rx_us,
                                               size_t const payloadSize,
                                               int32_t const freq_err)
{
    (void)rx_us;
    (void)freq_err;

    if (buff) {
        memcpy(rx_buffer, buff, payloadSize);
        rx_buffer_size = payloadSize;
        // DEBUG_PRINTF(" R ");
    }
}

static void FAST_CODE_1 TransmissionCompletedCallback()
{
    // DEBUG_PRINTF("X ");
    if (tx_common_frame_frame_is_tlm()) {
        // receive tlm package
        PowerMgmt.pa_off();
        Radio->RXnb(FHSSgetCurrFreq());
        expected_tlm_counter++;
        // DEBUG_PRINTF(" RX ");
    }
}

///////////////////////////////////////
// Internal OTA implementation

static void FAST_CODE_1 GenerateSyncPacketData(uint8_t * const output,
                                               uint32_t const rxtx_counter,
                                               uint_fast8_t const numOfTxPerRc)
{
    ElrsSyncPacket_s * sync = (ElrsSyncPacket_s *)output;
    sync->cipher = SyncCipher;
    sync->fhssIndex = FHSSgetCurrIndex();
    sync->rxtx_counter = rxtx_counter;
    sync->tlm_interval = TLMinterval;
    sync->rate_index = current_rate_config;
    sync->radio_mode = ExpressLRS_currAirRate->pkt_type;
    sync->no_sync_armed = (1 < numOfTxPerRc) ? 0 : TX_SKIP_SYNC;
    sync->pkt_type = UL_PACKET_SYNC;
}

///////////////////////////////////////
// Internal OTA implementation

uint_fast8_t FAST_CODE_1 ota_packet_generate_internal(uint8_t * const tx_buffer,
                                                      uint32_t const rxtx_counter,
                                                      uint32_t const current_us,
                                                      uint_fast8_t const hopInterval)
{
    uint16_t crc_or_type;
    uint_fast8_t payloadSize = RcChannels_payloadSizeGet();
    const uint_fast8_t numOfTxPerRc = ExpressLRS_currAirRate->numOfTxPerRc;
    const uint_fast8_t arm_state = RcChannels_get_arm_channel_state() && TX_SKIP_SYNC;
    const uint32_t sync_interval_us = SyncPacketInterval_us / (!arm_state + 1);

    // only send sync when its time and only on sync channel;
    if ((!arm_state || 1 < numOfTxPerRc) && FHSScurrSequenceIndexIsSyncChannel() &&
        ((rxtx_counter % hopInterval) == 0) && ((rxtx_counter % numOfTxPerRc) == 0) &&
        (sync_interval_us <= (uint32_t)(current_us - SyncPacketSent_us))) {
        GenerateSyncPacketData(tx_buffer, rxtx_counter, numOfTxPerRc);
        SyncPacketSent_us = current_us;
        crc_or_type = UL_PACKET_SYNC;
    } else if (!arm_state && (rxtx_counter % numOfTxPerRc) == 0 && (TLM_MSP_STATE_SEND == tlm_msp_send) &&
               (msp_packet_tx.type == MSP_PACKET_TLM_OTA)) {
        /* send tlm packet if needed */
        if (RcChannels_tlm_ota_send(tx_buffer, msp_packet_tx)) {
            msp_packet_tx.sequence_nbr = 0;
            msp_packet_tx.payloadIterator = 0;
#if TX_TLM_WHEN_DISARMED
            tlm_msp_send = TLM_MSP_STATE_WAIT_RESP;
#else
            tlm_msp_send = TLM_MSP_STATE_FREE;
#endif
            tlm_msp_sent_ms = millis();
        }
        crc_or_type = UL_PACKET_MSP;
    } else {
#if !SERIAL_PASSTHROUGH
        RcChannels_get_packed_data(tx_buffer);
#else
        #warning "TODO: Copy data from serial input buff to OTA!"
#endif
        crc_or_type = UL_PACKET_RC_DATA;
#if CRC16_POLY_TESTING
        memcpy(tx_buffer, CRC16_POLY_PKT, sizeof(CRC16_POLY_PKT));
#endif
    }

    RcChannels_packetTypeSet(tx_buffer, payloadSize, crc_or_type);

    if (ExpressLRS_currAirRate->hwCrc != HWCRC_EN) {
        // Calculate the CRC
        crc_or_type = CalcCRC16(tx_buffer, payloadSize, CRCCaesarCipher);
        tx_buffer[payloadSize++] = (crc_or_type >> 8);
        tx_buffer[payloadSize++] = (crc_or_type & 0xFF);
    }
    return payloadSize;
}

#if TARGET_HANDSET && OTA_VANILLA_ENABLED

uint_fast8_t FAST_CODE_1 ota_packet_generate_vanilla(uint8_t * const tx_buffer,
                                                     uint32_t const rxtx_counter,
                                                     uint32_t const current_us,
                                                     uint_fast8_t const hopInterval)
{
    uint_fast8_t payloadSize = OTA_VANILLA_SIZE;
    const uint_fast8_t numOfTxPerRc = ExpressLRS_currAirRate->numOfTxPerRc;
    const uint_fast8_t arm_state = OTA_vanilla_getArmChannelState();
    const uint32_t sync_interval_us = SyncPacketInterval_us / ((!arm_state * 3) + 1);

    // only send sync when its time and only on sync channel;
    if ((!arm_state || 1 < numOfTxPerRc) && FHSScurrSequenceIndexIsSyncChannel() &&
        ((rxtx_counter % hopInterval) == 0) && ((rxtx_counter % numOfTxPerRc) == 0) &&
        (sync_interval_us <= (uint32_t)(current_us - SyncPacketSent_us))) {
        OTA_vanilla_SyncPacketData(tx_buffer, rxtx_counter, TLMinterval);
        SyncPacketSent_us = current_us;
    } else {
        uint8_t NonceFHSSresult = rxtx_counter % hopInterval;
        OTA_vanilla_PackChannelData(tx_buffer, rxtx_counter, NonceFHSSresult);
    }

    OTA_vanilla_calculateCrc(tx_buffer, CRCCaesarCipher);

    return payloadSize;
}
#endif // TARGET_HANDSET

///////////////////////////////////////
// Generic TX ISR handler

static void FAST_CODE_1 SendRCdataToRF(uint32_t const current_us)
{
    // gpio_out_write(debug_pin_tx, 1);
    //  Called by HW timer
    uint32_t freq;
    uint32_t const rxtx_counter = _rf_rxtx_counter;
    // esp requires word aligned buffer
    uint32_t __tx_buffer[(OTA_PAYLOAD_MAX + sizeof(uint32_t) - 1) / sizeof(uint32_t)];
    uint8_t * const tx_buffer = (uint8_t *)__tx_buffer;
    uint_fast8_t const hopInterval = ExpressLRS_currAirRate->FHSShopInterval;
    uint_fast8_t payloadSize;

    // Check if telemetry RX ongoing
    if (tx_common_frame_frame_is_tlm()) {
        // Skip TX because TLM RX is ongoing
        goto send_to_rf_exit;
    }

    freq = FHSSgetCurrFreq();

    // ------------------------------------------------------------------

#if TARGET_HANDSET && OTA_VANILLA_ENABLED
    if (pl_config.rf_mode == RADIO_TYPE_128x_VANILLA)
        payloadSize = ota_packet_generate_vanilla(tx_buffer, (rxtx_counter - 1), current_us, hopInterval);
    else
#endif // TARGET_HANDSET
        payloadSize = ota_packet_generate_internal(tx_buffer, rxtx_counter, current_us, hopInterval);

    // ------------------------------------------------------------------

    // Enable PA
    PowerMgmt.pa_on();
    // Debugging
    // delayMicroseconds(random(0, 400)); // 300 ok
    // if (random(0, 99) < 55) tx_buffer[1] = 0;
    // Send data to rf
    Radio->TXnb(tx_buffer, payloadSize, freq);

send_to_rf_exit:
    // Check if hopping is needed
    if ((rxtx_counter % hopInterval) == 0) {
        // it is time to hop
        FHSSincCurrIndex();
    }
    // Increase TX counter
    _rf_rxtx_counter = rxtx_counter + 1;

    // DEBUG_PRINTF(" T");
    // gpio_out_write(debug_pin_tx, 0);
}

///////////////////////////////////////
int8_t SettingsCommandHandle(uint8_t const * in, uint8_t * out, uint8_t const inlen, uint8_t & outlen)
{
#if defined(LATEST_COMMIT)
    uint8_t commit_sha[] = {LATEST_COMMIT};
#endif
    uint8_t settings_buff[5 + sizeof(commit_sha) + 1];
    uint8_t * buff = settings_buff;
    uint8_t const cmd = in[0];
    uint8_t value = in[1];
    uint8_t modified = 0;

    switch (cmd) {
        case ELRS_CMD_GET: // send all params
            break;

        case ELRS_CMD_RATE:
            if (inlen != 2)
                return -1;
            // set air rate
            if (get_elrs_airRateMax() > value)
                modified |= (SetRFLinkRate(value) << 1);
            break;

        case ELRS_CMD_TLM:
            // set TLM interval
            if (inlen != 2)
                return -1;
            value = tx_tlm_value_validate(value);
            modified = (pl_config.rf[pl_config.rf_mode].tlm != value) ? (1 << 2) : 0;
            pl_config.rf[pl_config.rf_mode].tlm = value;
            DEBUG_PRINTF("Telemetry index: %u\n", value);
#if !defined(TX_TLM_WHEN_DISARMED)
            tx_tlm_change_interval(value);
#endif
            break;

        case ELRS_CMD_POWER:
            // set TX power
            if (inlen != 2)
                return -1;
            modified = PowerMgmt.setPower((PowerLevels_e)value) << 3;
            DEBUG_PRINTF("Power: %u\n", PowerMgmt.currPower());
            break;

        case ELRS_CMD_DOMAIN:
            // RFFreq
            if (inlen != 2)
                return -1;
            value = common_config_get_radio_type(value);
            if (SetRadioType(value)) {
                modified |= (SetRFLinkRate(current_rate_config) << 1);
            }
            break;

        case ELRS_CMD_WIFI_START:
            // Start WiFi
            platform_radio_force_stop();
            platform_wifi_start();
            break;

        case ELRS_CMD_RF_POWER_TEST:
            // RF power (TEST feature!)
            if (inlen != 2)
                return -1;
            DEBUG_PRINTF("RF Power: %u\n", value);
            Radio->SetOutputPower(value);
            break;

        default:
            return -1;
    }
    uint8_t const rf_mode = pl_config.rf_mode;

    buff[0] = (uint8_t)current_rate_config;
    buff[1] = (uint8_t)pl_config.rf[rf_mode].tlm;
    buff[2] = (uint8_t)PowerMgmt.currPower();
    buff[3] = (uint8_t)PowerMgmt.maxPowerGet();
    buff[4] = RADIO_RF_MODE_INVALID;
    if (RADIO_SX127x && rf_mode == RADIO_TYPE_127x) {
#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_FCC_915)
        buff[4] = RADIO_RF_MODE_915_AU_FCC;
#elif defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_EU_868_R9)
        buff[4] = RADIO_RF_MODE_868_EU;
#elif defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
        buff[4] = RADIO_RF_MODE_433_AU_EU;
#endif
    } else if (RADIO_SX128x) {
        if (rf_mode == RADIO_TYPE_128x)
            buff[4] = RADIO_RF_MODE_2400_ISM_500Hz;
        else if (rf_mode == RADIO_TYPE_128x_FLRC)
            buff[4] = RADIO_RF_MODE_2400_ISM_FLRC;
        else if (rf_mode == RADIO_TYPE_128x_VANILLA)
            buff[4] = RADIO_RF_MODE_2400_ISM_VANILLA;
    }
#if DOMAIN_BOTH
    buff[4] |= ExLRS_RF_MODE_DUAL;
#endif
#if TARGET_HANDSET
    buff[4] |= ExLRS_RF_MODE_HANDSET;
#endif
#if RADIO_SX128x && RADIO_SX128x_FLRC
    buff[4] |= ExLRS_RF_MODE_FLRC;
#endif
    buff += 5;

#if defined(LATEST_COMMIT)
    for (uint8_t iter = 0; iter < sizeof(commit_sha); iter++) {
        *buff++ = commit_sha[iter];
    }
#if LATEST_COMMIT_DIRTY
    *buff++ = '!';
#endif
#endif

#ifdef CTRL_SERIAL
    msp_packet_parser.sendPacket(&ctrl_serial, MSP_PACKET_V1_ELRS, ELRS_INT_MSP_PARAMS, MSP_ELRS_INT,
                                 (buff - settings_buff), settings_buff);
#endif /* CTRL_SERIAL */

    // Fill response
    if (out && (buff - settings_buff) <= outlen) {
        outlen = (buff - settings_buff);
        memcpy(out, settings_buff, outlen);
    }

    if (cmd != ELRS_CMD_GET && modified) {
        // Stop timer before save if not already done
        if ((modified & (1 << 1)) == 0) {
            stop_processing();
        }

        // Save modified values
        pl_config.key = ELRS_EEPROM_KEY;
        pl_config.rf[rf_mode].mode = current_rate_config;
        pl_config.rf[rf_mode].power = PowerMgmt.currPower();
        // pl_config.rf[rf_mode].tlm = ; // Set already
        platform_config_save(pl_config);

        // and restart timer
        TxTimer.start();
    }

    return 0;
}

///////////////////////////////////////
uint8_t SetRFLinkRate(uint8_t const rate, uint8_t const init) // Set speed of RF link (hz)
{
    const expresslrs_mod_settings_t * const config = get_elrs_airRateConfig(rate);
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

    FHSSresetCurrIndex();
    RcChannels_initRcPacket(config->payloadSize);
    Radio->SetRxBufferSize(config->payloadSize + (config->hwCrc == HWCRC_DIS ? OTA_PACKET_CRC : 0));
    Radio->SetPacketInterval(config->interval);
    Radio->SetCaesarCipher(CRCCaesarCipher);
    Radio->Config(config->bw, config->sf, config->cr, FHSSgetCurrFreq(), config->PreambleLen,
                  (config->hwCrc == HWCRC_EN), config->pkt_type);

    tx_handle_set_link_rate(config->interval * config->numOfTxPerRc);

    LinkStatistics.link.rf_Mode = config->rate_osd_num;

    uint8_t telemtery = pl_config.rf[pl_config.rf_mode].tlm;
    tx_tlm_change_interval(telemtery, init);

    write_u32(&SyncPacketInterval_us, config->syncInterval);

    platform_connection_state(connectionState);

    DEBUG_PRINTF("RF params configured, rate: %u (idx:%u)\n", config->rate, current_rate_config);

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
static void MspOtaCommandsSend(mspPacket_t & packet)
{
    // TODO,FIXME: add a retry!!
    if (TLM_MSP_STATE_FREE != read_u8(&tlm_msp_send)) {
        DEBUG_PRINTF("UL MSP ignored. func: %x, size: %u\n", packet.function, packet.payloadSize);
        return;
    }

    msp_packet_tx.reset();
    msp_packet_tx.type = MSP_PACKET_TLM_OTA;
    msp_packet_tx.flags = packet.flags;
    msp_packet_tx.function = packet.function;
    msp_packet_tx.payloadSize = packet.payloadSize;
    memcpy(msp_packet_tx.payload, packet.payload, packet.payloadSize);
    write_u8(&tlm_msp_send, TLM_MSP_STATE_SEND); // rdy for sending
    tlm_msp_sent_ms = 0;
}
#endif

void tx_common_handle_rx_buffer(void)
{
    uint_fast8_t size = rx_buffer_size;
    if (!size) {
        return;
    }
    process_rx_buffer(size);
    rx_buffer_size = 0;
    platform_wd_feed();
}

bool tx_common_has_telemetry(void)
{
    return !!tlm_check_ratio;
}

bool tx_common_check_connection(void)
{
    if (STATE_lost < connectionState) {
        uint32_t const current_ms = millis();
        if (RX_CONNECTION_LOST_TIMEOUT < (int32_t)(current_ms - LastPacketRecvMillis)) {
            connectionState = STATE_disconnected;
            platform_connection_state(STATE_disconnected);
            platform_set_led(red_led_state);
            return false;
        }
    }
    return true;
}

void tx_common_handle_ctrl_serial(void)
{
#ifdef CTRL_SERIAL
    if (ctrl_serial.available()) {
        platform_wd_feed();
        uint8_t in = ctrl_serial.read();
        if (msp_packet_parser.processReceivedByte(in)) {
            //  MSP received, check content
            mspPacket_t & packet = msp_packet_parser.getPacket();

            // DEBUG_PRINTF("MSP rcvd, type:%u\n", packet.type);

            /* Check if packet is ELRS internal */
            if ((packet.type == MSP_PACKET_V1_ELRS) && (packet.flags & MSP_ELRS_INT)) {
                switch (packet.function) {
                    case ELRS_INT_MSP_PARAMS: {
                        uint8_t * msg = (uint8_t *)packet.payload;
                        uint8_t outlen = packet.payloadSize;
                        SettingsCommandHandle(msg, NULL, packet.payloadSize, outlen);
                        break;
                    }
                    default: {
                        if (0 <= tx_handle_msp_input(packet)) {
                            /* Send resp */
                            msp_packet_parser.sendPacket(&packet, &ctrl_serial);
                        }
                        break;
                    }
                };
            } else {
                MspOtaCommandsSend(packet);
            }
            msp_packet_parser.markPacketFree();
        }
    }
#endif // CTRL_SERIAL
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

bool tx_common_tlm_rx_handle(uint32_t const now)
{
    // Send MSP resp if allowed and packet ready
    static uint8_t retrx_count = 0;
    uint8_t const tlm_msp_tx_state = read_u8(&tlm_msp_send);
    bool const wait_resp = TLM_MSP_STATE_WAIT_RESP == tlm_msp_tx_state;
    if (tlm_msp_rcvd) {
        // Check that resp match
        if (wait_resp && msp_packet_rx.sequence_nbr == 0 && msp_packet_rx.type == msp_packet_tx.type &&
            msp_packet_rx.function == msp_packet_tx.function) {
            msp_packet_rx.type = MSP_PACKET_V1_RESP; // Only v1 over OTA
            // Resp ok, free TX buffer to send next one
            write_u8(&tlm_msp_send, TLM_MSP_STATE_FREE);
            retrx_count = 0;
            // DEBUG_PRINTF("Expected MSP response\n");
        } else {
            // DEBUG_PRINTF("DL MSP rcvd. func: %x, size: %u\n",
            //              msp_packet_rx.function, msp_packet_rx.payloadSize);
        }
        return true; // Allow forwarding to logger
    } else if (wait_resp) {
        int32_t const timeout = (MSP_EEPROM_WRITE == msp_packet_tx.function) ? 200 : 1000;
        if (timeout <= (int32_t)(now - tlm_msp_sent_ms)) {
            if (2 < retrx_count++) {
                // Give up...
                write_u8(&tlm_msp_send, TLM_MSP_STATE_FREE);
                retrx_count = 0;
                DEBUG_PRINTF("MSP ignored after 3 retries\n");
            } else {
                // retransmit if no resp received
                write_u8(&tlm_msp_send, TLM_MSP_STATE_SEND);
            }
        }
    }
    return false;
}
