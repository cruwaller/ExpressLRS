#include <Arduino.h>
#include "utils.h"
#include "common.h"
#include "CRSF_TX.h"
#include "targets.h"
#include "POWERMGNT.h"
#include "debug_elrs.h"
#include "tx_common.h"
#include "HwTimer.h"
#include <stdlib.h>

#if !SERIAL_PASSTHROUGH

CRSF_TX DRAM_FORCE_ATTR crsf(CrsfSerial);

static uint32_t DRAM_ATTR TlmSentToRadioTime;

///////////////////////////////////////

static void rc_data_cb(uint8_t const *const channels)
{
    RcChannels_processChannelsCrsf((rc_channels_module_t*)channels);
}

static void ParamWriteHandler(uint8_t const *msg, uint16_t len)
{
    // Called from UART handling loop (main loop)
#if !SERIAL_PASSTHROUGH
    uint8_t resp[ELRS_LUA_BUFF_SIZE], outlen = sizeof(resp);
    int8_t ret;
    CrsfSerial.Pause();
    ret = SettingsCommandHandle(msg, resp, len, outlen);
    CrsfSerial.Continue();
    if (0 <= ret)
        crsf.sendLUAresponseToRadio(resp, outlen);
#endif
}

/* Parse CRSF encapsulated MSP packet */
static void msp_data_cb(uint8_t const *const input)
{
     /* process MSP packet from radio
     *    Start:
     *      [0] header: seq&0xF,
     *      [1] payload size
     *      [2] function
     *      [3...7] payload / crc
     *    Next:
     *      [0] header
     *      [1...7] payload / crc
     */
    mspHeaderV1_RX_t const * const p_msp = (mspHeaderV1_RX_t *)input;
    uint8_t const * p_src = p_msp->payload;
    uint16_t iter;

#if 0
    DEBUG_PRINTF("MSP: p_msp:%u size:%u func:%u data: ",
        p_msp->flags, p_msp->hdr.payloadSize, p_msp->hdr.function);
    for (iter = 0; (iter < p_msp->hdr.payloadSize); iter++) {
        DEBUG_PRINTF("0x%X, ", p_msp->payload[iter]);
    }
    DEBUG_PRINTF("\n");
#endif

    if (TLM_MSP_STATE_FREE != read_u8(&tlm_msp_send)) {
        DEBUG_PRINTF("MSP TX packet ignored\n");
        return;
    }

    if (p_msp->flags & MSP_STARTFLAG) {
        msp_packet_tx.reset();
        msp_packet_tx.flags = p_msp->flags;
        msp_packet_tx.type = MSP_PACKET_TLM_OTA;
        msp_packet_tx.payloadSize = p_msp->hdr.payloadSize;
        msp_packet_tx.function = p_msp->hdr.function;
        p_src = p_msp->hdr.payload;
    }
    for (iter = 0; (iter < sizeof(p_msp->payload)) && !msp_packet_tx.iterated(); iter++) {
        msp_packet_tx.addByte(p_src[iter]);
    }
    if (msp_packet_tx.iterated() && !msp_packet_tx.error) {
        msp_packet_tx.setIteratorToSize();
        write_u8(&tlm_msp_send, TLM_MSP_STATE_SEND);
    } else {
        msp_packet_tx.reset();
    }
}

static void FAST_CODE_1
update_handset_sync(uint32_t const current_us)
{
    // tells the crsf that we want to send data now - this allows opentx packet syncing
    crsf.UpdateOpenTxSyncOffset(current_us);
}

#endif /* !SERIAL_PASSTHROUGH */

void setup()
{
#if !SERIAL_PASSTHROUGH
    CrsfSerial.Begin(CRSF_TX_BAUDRATE_FAST);
#else /* SERIAL_PASSTHROUGH */
#if !defined(SERIAL_BAUD)
#define SERIAL_BAUD 115200
#endif
    CrsfSerial.Begin(SERIAL_BAUD);
#endif /* SERIAL_PASSTHROUGH */

    tx_common_init_globals();
    platform_setup();
    platform_set_led(0);
    DEBUG_PRINTF("ExpressLRS TX Module...\n");

#if !SERIAL_PASSTHROUGH
    crsf.connected = hw_timer_init;
    crsf.disconnected = hw_timer_stop;
    crsf.ParamWriteCallback = ParamWriteHandler;
    crsf.RCdataCallback1 = rc_data_cb;
    crsf.MspCallback = msp_data_cb;

    TxTimer.callbackTickPre = update_handset_sync;
#endif /* SERIAL_PASSTHROUGH */
    tx_common_init();

#if !SERIAL_PASSTHROUGH
    crsf.Begin();
#else /* SERIAL_PASSTHROUGH */
    // Start TX immediately
    hw_timer_init();
#endif /* SERIAL_PASSTHROUGH */
}

void loop()
{
#if !SERIAL_PASSTHROUGH
    uint32_t const current_ms = millis();
#endif

    tx_common_handle_rx_buffer();

#if !SERIAL_PASSTHROUGH
    if (tx_common_has_telemetry())
    {
        if (tx_common_check_connection() &&
            connectionState == STATE_connected &&
            TLM_REPORT_INTERVAL <= (uint32_t)(current_ms - TlmSentToRadioTime))
        {
            TlmSentToRadioTime = current_ms;
            tx_common_update_link_stats();
            crsf.LinkStatisticsSend(LinkStatistics.link);
            crsf.BatterySensorSend(LinkStatistics.batt);
            crsf.GpsSensorSend(GpsTlm);
        }
    }

    // Process CRSF packets from TX
    bool const can_send = crsf.handleUartIn();

    // Send MSP resp if allowed and packet ready
    if (can_send && tx_common_tlm_rx_handle(current_ms))
    {
        crsf.sendMspPacketToRadio(msp_packet_rx);
        if (msp_packet_rx.iterated()) {
            msp_packet_rx.reset();
            tlm_msp_rcvd = 0;
        }
    }
#ifdef CTRL_SERIAL
    else {
        tx_common_handle_ctrl_serial();
    }
#endif /* CTRL_SERIAL */

#endif /* SERIAL_PASSTHROUGH */

    platform_loop(connectionState);
    platform_wd_feed();
}

int8_t tx_handle_msp_input(mspPacket_t &packet)
{
    (void)packet;
    return -1;
}

void tx_handle_set_link_rate(uint32_t const interval)
{
#if !SERIAL_PASSTHROUGH
    crsf.setRcPacketRate(interval);
#endif
}
