#include <Arduino.h>
#include "utils.h"
#include "common.h"
#include "targets.h"
#include "POWERMGNT.h"
#include "debug_elrs.h"
#include "tx_common.h"
#include "HwTimer.h"
#include "handset.h"
#include "OTAvanilla.h"
#include <stdlib.h>

#define RC_CH_PRINT_INTERVAL    2000

//#define DBG_PIN SWITCH_6_2
#ifdef DBG_PIN
static struct gpio_out debug;
#endif

static rc_channels_internal_u DRAM_ATTR rc_data;

///////////////////////////////////////

void LinkStatisticsAndBatterySend(void);
void GpsSensorSend(void);

///////////////////////////////////////

static void FAST_CODE_1
rc_data_collect(uint32_t const current_us)
{
    uint8_t num_channels = switches_get_available();
#ifdef DBG_PIN
    gpio_out_write(debug, 1);
#endif
    uint16_t const channelMax =
#if OTA_VANILLA_ENABLED
        (pl_config.rf_mode == RADIO_TYPE_128x_VANILLA) ?
            OTA_vanilla_get_channelMaxValue():
#endif
            RcChannels_channelMaxValueGet();
    uint16_t const channelMin =
#if OTA_VANILLA_ENABLED
        (pl_config.rf_mode == RADIO_TYPE_128x_VANILLA) ?
            OTA_vanilla_get_channelMinValue():
#endif
            ANALOG_MIN_VAL;
    uint16_t gimbals[TX_NUM_ANALOGS];
    uint16_t aux[num_channels] = {0};
    uint16_t scale;
    uint8_t iter, index;
    gimbals_timer_adjust(current_us);
    gimbals_get(gimbals, channelMin, channelMax);
    for (iter = 0; iter < TX_NUM_ANALOGS; iter++) {
        index = pl_config.mixer[iter].index;
        if (pl_config.mixer[iter].inv) {
            gimbals[index] = channelMin +
                (channelMax - gimbals[index]);
        }
        scale = pl_config.mixer[iter].scale;
        if (scale) {
            scale = ((channelMax * scale) / 100U);
            gimbals[index] =
                MAP_U16(gimbals[index],
                    channelMin, channelMax,
                    channelMin, scale);
        }
    }
    // mix analog channels into output buffer
    for (iter = 0; iter < TX_NUM_ANALOGS; iter++) {
        rc_data.ch[iter] = gimbals[pl_config.mixer[iter].index];
    }
    // get AUX channel positions
    switches_collect(aux);
    for (iter = 0; iter < num_channels; iter++) {
        if (pl_config.mixer[(iter + 4)].inv) {
            index = pl_config.mixer[(iter + 4)].index;
            aux[index] = SWITCH_MAX - aux[index];
        }
    }
    num_channels += TX_NUM_ANALOGS;
    // mix AUX channels into output buffer
    for (iter = TX_NUM_ANALOGS; iter < num_channels; iter++) {
        rc_data.ch[iter] = aux[pl_config.mixer[iter].index];
    }
    // process channel data into OTA packet
#if OTA_VANILLA_ENABLED
    if (pl_config.rf_mode == RADIO_TYPE_128x_VANILLA) {
        // map range for vanilla code
        for (iter = TX_NUM_ANALOGS; iter < num_channels; iter++) {
            rc_data.ch[iter] = MAP_U16(rc_data.ch[iter],
                    SWITCH_MIN, SWITCH_MAX,
                    channelMin, channelMax);
        }
        OTA_vanilla_processChannels(&rc_data.ota_pkt);
    } else
#endif
    {
        RcChannels_processChannels(&rc_data.ota_pkt);
        tx_common_armed_state(RcChannels_get_arm_channel_state());
    }
#ifdef DBG_PIN
    gpio_out_write(debug, 0);
#endif
}

void send_config_mixer(void)
{
    /* Send config data to controller */
    constexpr uint8_t mixer_size = (sizeof(struct mixer) + 1);
    uint8_t buffer[mixer_size * ARRAY_SIZE(pl_config.mixer) + 2];
    uint8_t *data = buffer, iter;
    for (iter = 0; iter < ARRAY_SIZE(pl_config.mixer); iter++) {
        data[0] = iter;
        data[1] = pl_config.mixer[iter].index;
        data[2] = pl_config.mixer[iter].inv;
        data[3] = pl_config.mixer[iter].scale;
        data += mixer_size;
    }
    *data++ = switches_get_available();
    *data++ = N_SWITCHES;
    MSP::sendPacket(
        &ctrl_serial, MSP_PACKET_V1_ELRS, ELRS_HANDSET_MIXER,
        MSP_ELRS_INT, (data - buffer), buffer);
}

void send_configs_gimbals(void)
{
    uint8_t buffer[sizeof(pl_config.gimbals)];
    memcpy(buffer, pl_config.gimbals, sizeof(buffer));
    MSP::sendPacket(
        &ctrl_serial, MSP_PACKET_V1_ELRS, ELRS_HANDSET_ADJUST,
        MSP_ELRS_INT, sizeof(buffer), buffer);
}

void send_configs(void)
{
    //delay(2);
    send_config_mixer();
    delay(2);
    send_configs_gimbals();
}

void save_configs(void)
{
    uint32_t irq = _SAVE_IRQ();
    // Stop TX processing
    TxTimer.stop();
    Radio->StopContRX();
    _RESTORE_IRQ(irq);
    // save config
    platform_config_save(pl_config);
    // restart processing
    TxTimer.start();
    // Update configs just in case
    send_configs();
}

///////////////////////////////////////

void setup()
{
    uint8_t iter;
    uint8_t num_of_switches = switches_get_available();
    tx_common_init_globals();
    /** Default mixer setup (Mode2)
     * CH0 = Thr (throttle, inv)
     * CH1 = Ail (roll, inv)
     * CH2 = Ele (pitch)
     * CH3 = Rud (yaw)
     *
     * CH4..CH15 = AUX0...AUX11
    */
    memset(pl_config.mixer, 0, sizeof(pl_config.mixer));
    pl_config.mixer[0].index = GIMBAL_IDX_L1;
    pl_config.mixer[0].inv = 1;
    pl_config.mixer[1].index = GIMBAL_IDX_R2;
    pl_config.mixer[1].inv = 1;
    pl_config.mixer[2].index = GIMBAL_IDX_R1;
    pl_config.mixer[2].inv = 0;
    pl_config.mixer[3].index = GIMBAL_IDX_L2;
    pl_config.mixer[3].inv = 0;
    for (iter = 4; iter < ARRAY_SIZE(pl_config.mixer); iter++) {
        uint8_t aux = iter - 4;
        if (aux < N_SWITCHES && aux < num_of_switches)
            pl_config.mixer[iter].index = aux;
        else
            pl_config.mixer[iter].index = 16;
    }
    /** Set default gimbal ranges */
    struct gimbal_limit gimbal_limit[TX_NUM_ANALOGS] = {
        {900, 2196, 3536}, // L1
        {194, 2023, 3796}, // L2
        {183, 1860, 3628}, // R1
        {490, 2094, 3738}, // R2
    };
    memcpy(pl_config.gimbals, gimbal_limit, sizeof(gimbal_limit));

    platform_setup();
    DEBUG_PRINTF("ExpressLRS HANDSET\n");

#ifdef DBG_PIN
    debug = gpio_out_setup(DBG_PIN, 0);
#endif

    switches_init();
    gimbals_init();

    TxTimer.callbackTickPre = rc_data_collect;

#if 0
    TxTimer.updateInterval(5000);
    TxTimer.init();
    TxTimer.start();
    while (1) {
        DEBUG_PRINTF("RC: %u, %u, %u, %u -- %u, %u, %u\n",
            rc_data.ch[0], rc_data.ch[1], rc_data.ch[2], rc_data.ch[3],
            rc_data.ch[4], rc_data.ch[5], rc_data.ch[6]);
        delay(100);
    }
#endif
    /* Initialize common TX procedures */
    tx_common_init();
    /* Send config data to controller */
    send_configs();
    /* Start TX */
    hw_timer_init();
}


void loop()
{
    uint32_t const now = millis();
    uint32_t const _tlm_updated = read_u32(&tlm_updated);
    write_u8(&tlm_updated, TLM_UPDATES_NA);

    tx_common_handle_rx_buffer();

    if (0 <= tx_common_has_telemetry() && _tlm_updated) {
        (void)tx_common_check_connection();

        // TODO: limit tlm output speed for higher ratios?

        if (_tlm_updated & TLM_UPDATES_DEV_INFO) {
            uint8_t buff[] = {DevInfo.state, DevInfo.count};
            MSP::sendPacket(
                &ctrl_serial, MSP_PACKET_V1_ELRS, ELRS_INT_MSP_DEV_INFO,
                MSP_ELRS_INT, sizeof(buff), buff);
        } else if (_tlm_updated & TLM_UPDATES_GPS) {
            GpsSensorSend();
        } else if (_tlm_updated & (TLM_UPDATES_LNK_STATS | TLM_UPDATES_BATTERY)) {
            static uint32_t last_tlm_sent;
            if (100U <= (now - last_tlm_sent)) {
                tx_common_update_link_stats();
                LinkStatisticsAndBatterySend();
                last_tlm_sent = now;
            }
        }
    }
    bool need_delay = false;

#if RC_CH_PRINT_INTERVAL
    static uint32_t last_rc_info;
    if (RC_CH_PRINT_INTERVAL <= (now - last_rc_info) && !_tlm_updated) {
        last_rc_info = now;
        need_delay = true;

        // Push RC data to logger
        MSP::sendPacket(
            &ctrl_serial, MSP_PACKET_V1_ELRS, ELRS_HANDSET_RC_DATA,
            MSP_ELRS_INT, sizeof(rc_data), (uint8_t*)&rc_data);
        /*
        DEBUG_PRINTF("RC: %u|%u|%u|%u -- %u|%u|%u|%u|%u|%u\n",
            rc_data.ch[0], rc_data.ch[1], rc_data.ch[2], rc_data.ch[3],
            rc_data.ch[4], rc_data.ch[5], rc_data.ch[6], rc_data.ch[7],
            rc_data.ch[8], rc_data.ch[9]);
        */
    }
#endif // RC_CH_PRINT_INTERVAL

    // Send MSP resp if allowed and packet ready
    if (tx_common_tlm_rx_handle(now)) {
        if (need_delay)
            delay(2);
        if (!MSP::sendPacket(&msp_packet_rx, &ctrl_serial)) {
            DEBUG_PRINTF("unable to send MSP packet to Logger!");
        }
        msp_packet_rx.reset();
        tlm_msp_rcvd = 0;
    }

    tx_common_handle_ctrl_serial();

    platform_loop(connectionState);
    platform_wd_feed();
}


uint8_t handle_mixer(uint8_t * data, int32_t len)
{
    uint8_t * ptr = data, index, scale;
    while ((ptr - data) < len) {
        index = ptr[0];
        scale = 0;
        if (index < ARRAY_SIZE(pl_config.mixer)) {
            pl_config.mixer[index].index = ptr[1];
            pl_config.mixer[index].inv = ptr[2];
            if (index < 4) {
                scale = ptr[3];
                DEBUG_PRINTF("Scale %u\n", scale);
                ptr++;
            }
            pl_config.mixer[index].scale = scale;
        }
        ptr += 3;
    }
    return 0;
}


int8_t tx_handle_msp_input(mspPacket_t &packet)
{
    uint32_t val = 0;
    int8_t ret = -1;
    switch (packet.function) {
        case ELRS_HANDSET_CALIBRATE: {
            if (gimbals_calibrate(packet.payload)) {
                send_configs_gimbals();
            } else {
                /* Send error */
                packet.payloadSize = 1;
                ret = 0;
            }
            break;
        }
        case ELRS_HANDSET_MIXER: {
            handle_mixer(packet.payload, packet.payloadSize);
            send_config_mixer();
            break;
        }
        case ELRS_HANDSET_ADJUST_MIN: {
            val = packet.payload[1];
            val <<= 8;
            val += packet.payload[2];
            gimbals_adjust_min(val, packet.payload[0]);
            break;
        }
        case ELRS_HANDSET_ADJUST_MID: {
            val = packet.payload[1];
            val <<= 8;
            val += packet.payload[2];
            gimbals_adjust_mid(val, packet.payload[0]);
            break;
        }
        case ELRS_HANDSET_ADJUST_MAX: {
            val = packet.payload[1];
            val <<= 8;
            val += packet.payload[2];
            gimbals_adjust_max(val, packet.payload[0]);
            break;
        }
        case ELRS_HANDSET_CONFIGS_LOAD: {
            send_configs();
            break;
        }
        case ELRS_HANDSET_CONFIGS_SAVE: {
            save_configs();
            break;
        }
        default: {
            break;
        }
    }
    return ret;
}


void tx_handle_set_link_rate(uint32_t const interval)
{
    (void)interval;
    // TODO: should the gimbal read timing be updated as well? Runs @ 1kHz atm
}

/***********************/

void LinkStatisticsAndBatterySend(void)
{
    MSP::sendPacket(
        &ctrl_serial, MSP_PACKET_V1_ELRS, ELRS_HANDSET_TLM_LINK_STATS,
        MSP_ELRS_INT, sizeof(LinkStatistics), (uint8_t*)&LinkStatistics);
}

void GpsSensorSend(void)
{
    if (!GpsTlm.pkt_cnt)
        return;
    MSP::sendPacket(
        &ctrl_serial, MSP_PACKET_V1_ELRS, ELRS_HANDSET_TLM_GPS,
        MSP_ELRS_INT, sizeof(GpsTlm), (uint8_t*)&GpsTlm);
    GpsTlm.pkt_cnt = 0;
}
