#include "msp_handler_base.h"

int MspHandlerBase::parseCommand(websoc_bin_hdr_t const * const cmd,
                                 size_t const len,
                                 AsyncWebSocketClient * const client)
{
    switch (cmd->msg_id) {
        case WSMSGID_VIDEO_FREQ: {
            if (!len) {
                return 0; // Invalid message!
            }
            uint16_t const freq = parseFreq(cmd->payload);
            if (storeVtxFreq(client, freq)) {
                vtx_params_t const params = {.freq = freq, .power = -1, .pitmode = false};
                handleVtxFrequencyCommand(params, client);
#if LOGGER_HDZERO_USE_VTX_INDEX
                int8_t const chan_index = getIndexByFreq(freq);
                if (0 <= chan_index)
                    espnow_vtxset_send(chan_index);
#else
                espnow_vtxset_send(freq);
#endif
            }
            break;
        }
        case WSMSGID_LAPTIMER_OSD_POS: {
            uint16_t const row = ((uint16_t)cmd->payload[1] << 8) + cmd->payload[0];
            uint16_t const col = ((uint16_t)cmd->payload[3] << 8) + cmd->payload[2];
            if (osdRowMax() <= row) {
                websocket_send_txt("[ERROR] Invalid row value.");
            } else if (osdColumnMax() <= col) {
                websocket_send_txt("[ERROR] Invalid column value.");
            } else {
                eeprom_storage.laptimer_osd_pos.row = row;
                eeprom_storage.laptimer_osd_pos.column = col;
                eeprom_storage.markDirty();
            }
            break;
        }
        case WSMSGID_LAPTIMER_OSD_TIMEOUT: {
            uint16_t const timeout = ((uint16_t)cmd->payload[1] << 8) + cmd->payload[0];
            eeprom_storage.laptimer_osd_timeout = timeout * 1000;
            eeprom_storage.markDirty();
            break;
        }
        default:
            return parseCommandPriv(cmd, len, client);
    }
    return 0;
}

// Handle received MSP packet
int MspHandlerBase::parseCommand(mspPacket_t & msp_in)
{
    if (msp_in.type == MSP_PACKET_V2_COMMAND) {
        if (msp_in.function == MSP_VTX_SET_CONFIG && 2 <= msp_in.payloadSize) {
            uint16_t const freq = parseFreq(msp_in.payload);
            if (storeVtxFreq(NULL, freq)) {
                vtx_params_t params = {.freq = freq, .power = -1, .pitmode = false};
                if (2 <= msp_in.payloadSize) {
                    // Read power
                    params.power = msp_in.payload[2];
                }
                if (3 <= msp_in.payloadSize) {
                    // Read pitmode status
                    params.pitmode = !!msp_in.payload[3];
                }
                handleVtxFrequencyCommand(params, NULL);
                /* Infrom web clients */
                webUiSendVtxFrequency(freq);
            }
            return 0;
        } else if (msp_in.function == MSP_ELRS_FUNC) {
            // vanilla elrs specific command... ignore
            return 0;
        }
    } else if (msp_in.type == MSP_PACKET_V2_RESPONSE) {
        if (msp_in.function == MSP_ELRS_FUNC) {
            // vanilla elrs specific command... ignore
            return 0;
        }
    }
    return parseCommandPriv(msp_in);
}

String MspHandlerBase::vtxFreqGetBandChannel(uint16_t freq)
{
    String info = "--";
    if (!freq) {
        freq = eeprom_storage.vtx_freq;
    }
    int8_t index = getIndexByFreq(freq);
    if (0 <= index) {
        uint8_t const _band = index / VTX_CHANNEL_MAX;
        uint8_t const _index = index % VTX_CHANNEL_MAX;
        char const bands[VTX_CHANNEL_MAX] = {'A', 'B', 'E', 'F', 'R', 'L'};
        info = bands[_band];
        info += (_index + 1);
    }
    return info;
}

void MspHandlerBase::webUiSendVtxFrequency(uint16_t const freq, AsyncWebSocketClient * const client) const
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_VIDEO_FREQ >> 8),
        (uint8_t)WSMSGID_VIDEO_FREQ,
        (uint8_t)(freq >> 8),
        (uint8_t)freq,
    };
    websocket_send_bin(response, sizeof(response), client);
}

void MspHandlerBase::webUiSendVRecordingState(uint8_t const state, AsyncWebSocketClient * const client) const
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_RECORDING_CTRL >> 8),
        (uint8_t)WSMSGID_RECORDING_CTRL,
        state,
    };
    websocket_send_bin(response, sizeof(response), client);
}

void MspHandlerBase::webUiSendLaptimerState(uint16_t const race_id,
                                            uint16_t const round_num,
                                            bool const state,
                                            AsyncWebSocketClient * const client) const
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_LAPTIMER_START_STOP >> 8),
        (uint8_t)WSMSGID_LAPTIMER_START_STOP,
        (uint8_t)(race_id >> 8),
        (uint8_t)race_id,
        (uint8_t)state,
        (uint8_t)(round_num >> 8),
        (uint8_t)round_num,
    };
    websocket_send_bin(response, sizeof(response), client);
}

void MspHandlerBase::webUiSendLaptimerLap(laptimer_lap_t const * lap, AsyncWebSocketClient * const client) const
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_LAPTIMER_LAPTIME >> 8),
        (uint8_t)WSMSGID_LAPTIMER_LAPTIME,
        (uint8_t)(lap->lap_time_ms >> 24),
        (uint8_t)(lap->lap_time_ms >> 16),
        (uint8_t)(lap->lap_time_ms >> 8),
        (uint8_t)lap->lap_time_ms,
        (uint8_t)(lap->race_id >> 8),
        (uint8_t)lap->race_id,
        (uint8_t)(lap->lap_index),
    };
    websocket_send_bin(response, sizeof(response), client);
}
