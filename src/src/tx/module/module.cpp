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


CRSF_TX DRAM_FORCE_ATTR crsf(CrsfSerial);
static uint32_t DRAM_ATTR TlmSentToRadioTime;

///////////////////////////////////////

static void rc_data_cb(uint8_t const *const channels)
{
    RcChannels_processChannels((rc_channels_t*)channels);
}

static void ParamWriteHandler(uint8_t const *msg, uint16_t len)
{
    // Called from UART handling loop (main loop)
    uint8_t resp[5], outlen = sizeof(resp);
    if (0 > SettingsCommandHandle(msg, resp, len, outlen))
        return;
    crsf.sendLUAresponseToRadio(resp, outlen);
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
    mspHeaderV1_RX_t *hdr = (mspHeaderV1_RX_t *)input;
    uint16_t iter;

    if (read_u8(&tlm_msp_send) || (0 <= tx_common_has_telemetry())) {
        DEBUG_PRINTF("MSP TX packet ignored\n");
        return;
    }

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
        // check CRC
        if ((hdr->payload[iter] == msp_packet_tx.crc) && !msp_packet_tx.error) {
            msp_packet_tx.addByte(msp_packet_tx.crc);
            msp_packet_tx.setIteratorToSize();
            write_u8(&tlm_msp_send, 1); // rdy for sending
        } else {
            msp_packet_tx.reset();
        }
    }
}

static void ICACHE_RAM_ATTR
update_handset_sync(uint32_t const current_us)
{
    // tells the crsf that we want to send data now - this allows opentx packet syncing
    crsf.UpdateOpenTxSyncOffset(current_us);
}

void setup()
{
    CrsfSerial.Begin(CRSF_TX_BAUDRATE_FAST);
    tx_common_init_globals();
    platform_setup();
    DEBUG_PRINTF("ExpressLRS TX Module...\n");

    crsf.connected = hw_timer_init;
    crsf.disconnected = hw_timer_stop;
    crsf.ParamWriteCallback = ParamWriteHandler;
    crsf.RCdataCallback1 = rc_data_cb;
    crsf.MspCallback = msp_data_cb;

    TxTimer.callbackTockPre = update_handset_sync;

    tx_common_init();

    crsf.Begin();
}

void loop()
{
    uint8_t can_send;

    tx_common_handle_rx_buffer();

    if (0 <= tx_common_has_telemetry())
    {
        uint32_t current_ms = millis();
        if (0 <= tx_common_check_connection() &&
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
    can_send = crsf.handleUartIn();

    // Send MSP resp if allowed and packet ready
    if (can_send && tlm_msp_rcvd)
    {
        DEBUG_PRINTF("DL MSP rcvd. func: %x, size: %u\n",
            msp_packet_rx.function, msp_packet_rx.payloadSize);
        crsf.sendMspPacketToRadio(msp_packet_rx);
        msp_packet_rx.reset();
        tlm_msp_rcvd = 0;
    }
#ifdef CTRL_SERIAL
    else {
        tx_common_handle_ctrl_serial();
    }
#endif /* CTRL_SERIAL */

    platform_loop(connectionState);
    platform_wd_feed();
}
