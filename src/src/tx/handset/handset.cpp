#include <Arduino.h>
#include "utils.h"
#include "common.h"
#include "targets.h"
#include "POWERMGNT.h"
#include "debug_elrs.h"
#include "tx_common.h"
#include "HwTimer.h"
#include "gimbals.h"
#include "switches.h"
#include "rc_channels.h"
#include <stdlib.h>

#define RC_CH_PRINT_INTERVAL    2000

//#define DBG_PIN SWITCH_6_2
#ifdef DBG_PIN
static struct gpio_out debug;
#endif

static rc_channels_t DRAM_ATTR rc_data;
#if RC_CH_PRINT_INTERVAL
static uint32_t last_rc_info;
#endif

///////////////////////////////////////

void LinkStatisticsSend(void);
void BatterySensorSend(void);
void GpsSensorSend(void);

///////////////////////////////////////

static void FAST_CODE_1
rc_data_collect(uint32_t const current_us)
{
#ifdef DBG_PIN
    gpio_out_write(debug, 1);
#endif
    uint16_t gimbals[NUM_ANALOGS];
    uint16_t aux[NUM_SWITCHES] = {0};
    uint16_t scale;
    uint8_t iter, index;
    gimbals_timer_adjust(current_us);
    gimbals_get(gimbals);
    for (iter = 0; iter < NUM_ANALOGS; iter++) {
        index = pl_config.mixer[iter].index;
        if (pl_config.mixer[iter].inv) {
            gimbals[index] = ANALOG_MIN_VAL +
                (ANALOG_MAX_VAL - gimbals[index]);
        }
        scale = pl_config.mixer[iter].scale;
        if (scale) {
            scale = (((uint32_t)ANALOG_MAX_VAL * scale) / 100U);
            gimbals[index] =
                MAP_U16(gimbals[index],
                    ANALOG_MIN_VAL, ANALOG_MAX_VAL,
                    ANALOG_MIN_VAL, scale);
        }
    }
    rc_data.ch0 = gimbals[pl_config.mixer[0].index];
    rc_data.ch1 = gimbals[pl_config.mixer[1].index];
    rc_data.ch2 = gimbals[pl_config.mixer[2].index];
    rc_data.ch3 = gimbals[pl_config.mixer[3].index];
    switches_collect(aux);
    for (iter = 0; iter < NUM_SWITCHES; iter++) {
        if (pl_config.mixer[(iter + 4)].inv) {
            index = pl_config.mixer[(iter + 4)].index;
            aux[index] = SWITCH_MAX - aux[index];
        }
    }
    rc_data.ch4 = aux[pl_config.mixer[4].index];
    rc_data.ch5 = aux[pl_config.mixer[5].index];
    rc_data.ch6 = aux[pl_config.mixer[6].index];
    rc_data.ch7 = aux[pl_config.mixer[7].index];
    rc_data.ch8 = aux[pl_config.mixer[8].index];
    rc_data.ch9 = aux[pl_config.mixer[9].index];
#if 6 < NUM_SWITCHES
    rc_data.ch10 = aux[pl_config.mixer[10].index];
#if 7 < NUM_SWITCHES
    rc_data.ch11 = aux[pl_config.mixer[11].index];
#if 8 < NUM_SWITCHES
    rc_data.ch12 = aux[pl_config.mixer[12].index];
#if 9 < NUM_SWITCHES
    rc_data.ch13 = aux[pl_config.mixer[13].index];
#if 10 < NUM_SWITCHES
    rc_data.ch14 = aux[pl_config.mixer[14].index];
#if 11 < NUM_SWITCHES
    rc_data.ch15 = aux[pl_config.mixer[15].index];
#endif
#endif
#endif
#endif
#endif
#endif
    RcChannels_processChannels(&rc_data);
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
    delay(10);
    send_config_mixer();
    delay(10);
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
#if (HANDSET_GIMBALS == GIMBALS_NV14)
        {0, 2047, 4095}, // L1
        {0, 2047, 4095}, // L2
        {0, 2047, 4095}, // R1
        {0, 2047, 4095}, // R2
#else
        {900, 2196, 3536}, // L1
        {194, 2023, 3796}, // L2
        {183, 1860, 3628}, // R1
        {490, 2094, 3738}, // R2
#endif
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
            rc_data.ch0, rc_data.ch1, rc_data.ch2, rc_data.ch3,
            rc_data.ch4, rc_data.ch5, rc_data.ch6);
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
    uint8_t _tlm_updated = read_u8(&tlm_updated);
    write_u8(&tlm_updated, 0);

    tx_common_handle_rx_buffer();

    if (0 <= tx_common_has_telemetry() && _tlm_updated) {
        (void)tx_common_check_connection();

        if (_tlm_updated & TLM_UPDATES_LNK_STATS) {
            tx_common_update_link_stats();
            LinkStatisticsSend();
            delay(2);
        }
        if (_tlm_updated & TLM_UPDATES_BATTERY) {
            BatterySensorSend();
            delay(2);
        }
        if (_tlm_updated & TLM_UPDATES_GPS) {
            GpsSensorSend();
            delay(2);
        }
    }

#if RC_CH_PRINT_INTERVAL
    if (RC_CH_PRINT_INTERVAL <= (millis() - last_rc_info)) {
        last_rc_info = millis();
        DEBUG_PRINTF("RC: %u|%u|%u|%u -- %u|%u|%u|%u|%u\n",
            rc_data.ch0, rc_data.ch1, rc_data.ch2, rc_data.ch3,
            rc_data.ch4, rc_data.ch5, rc_data.ch6, rc_data.ch7, rc_data.ch8);
    }
#endif // RC_CH_PRINT_INTERVAL

    // Send MSP resp if allowed and packet ready
    if (tlm_msp_rcvd) {
        //DEBUG_PRINTF("DL MSP rcvd. func: %x, size: %u\n",
        //    msp_packet_rx.function, msp_packet_rx.payloadSize);

        // TODO: Send received MSP packet to CTRL_SERIAL (MSP)
        // msp_packet_rx;

        msp_packet_rx.reset();
        tlm_msp_rcvd = 0;
    } else {
        tx_common_handle_ctrl_serial();
    }
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

/***********************/

void LinkStatisticsSend(void)
{
    MSP::sendPacket(
        &ctrl_serial, MSP_PACKET_V1_ELRS, ELRS_HANDSET_TLM_LINK_STATS,
        MSP_ELRS_INT, sizeof(LinkStatistics.link),
        (uint8_t*)&LinkStatistics.link);
}

void BatterySensorSend(void)
{
    MSP::sendPacket(
        &ctrl_serial, MSP_PACKET_V1_ELRS, ELRS_HANDSET_TLM_BATTERY,
        MSP_ELRS_INT, sizeof(LinkStatistics.batt),
        (uint8_t*)&LinkStatistics.batt);
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
