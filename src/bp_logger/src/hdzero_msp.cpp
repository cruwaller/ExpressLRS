#include "hdzero_msp.h"

#define RETRY_INTERVAL_MS 1000

/*
 *  Get band/channel index  0x0300
 *  Set band/channel index  0x0301
 *  Get frequency           0x0302
 *  Set frequency           0x0303
 *  Get recording state     0x0304
 *  Set recording state     0x0305
 *  Get VRx mode            0x0306
 *  Set VRx mode            0x0307
 *  Get RSSI                0x0308
 *  Get battery voltage     0x0309
 *  Get firmware            0x030A
 *  Set buzzer              0x030B
 *  Set OSD Element         0x030C
 *  Set Head Tracking       0x030D
 *      - enable/disable head-tracking forwarding packets to the TX
 *  Set RTC                 0x030E
 */
#define HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_GET 0x0300
#define HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_SET 0x0301
#define HDZ_MSP_FUNC_FREQUENCY_GET          0x0302
#define HDZ_MSP_FUNC_FREQUENCY_SET          0x0303
#define HDZ_MSP_FUNC_RECORDING_STATE_GET    0x0304
#define HDZ_MSP_FUNC_RECORDING_STATE_SET    0x0305
#define HDZ_MSP_FUNC_VRX_MODE_GET           0x0306
#define HDZ_MSP_FUNC_VRX_MODE_SET           0x0307
#define HDZ_MSP_FUNC_RSSI_GET               0x0308
#define HDZ_MSP_FUNC_BATTERY_VOLTAGE_GET    0x0309
#define HDZ_MSP_FUNC_FIRMWARE_GET           0x030A
#define HDZ_MSP_FUNC_BUZZER_SET             0x030B
#define HDZ_MSP_FUNC_OSD_ELEMENT_SET        0x030C
#define HDZ_MSP_FUNC_SET_HEAD_TRACKING      0x030D
#define HDZ_MSP_FUNC_SET_RTC                0x030E

#define MSP_ELRS_SET_OSD 0x00B6

// Incoming packets originating from the VRx
#define MSP_ELRS_BACKPACK_SET_MODE    0x0380 // enable wifi/binding mode
#define MSP_ELRS_BACKPACK_GET_VERSION 0x0381 // get the bacpack firmware version
#define MSP_ELRS_BACKPACK_GET_STATUS  0x0382 // get the status of the backpack
#define MSP_ELRS_BACKPACK_SET_PTR     0x0383 // forwarded back to TX backpack

enum {
    OSD_CMD_HEARTBEAT = 0,
    OSD_CMD_RELEASE_PORT = 1,
    OSD_CMD_SCREEN_CLEAR = 2,
    OSD_CMD_SCREEN_WRITE = 3,
    OSD_CMD_SCREEN_DRAW = 4,
};

static bool check_retry(uint32_t const ms_now, uint32_t const timeout_ms)
{
    static uint32_t wait_started_ms;
    if (!ms_now || !timeout_ms || (int32_t)timeout_ms <= (int32_t)(ms_now - wait_started_ms)) {
        wait_started_ms = ms_now;
        return true;
    }
    return false;
}

void HDZeroMsp::init(void)
{
    current_state = STATE_GET_FW_VER;
    osd_timeout = eeprom_storage.laptimer_osd_timeout;
}

void HDZeroMsp::syncSettings(void)
{
}

void HDZeroMsp::syncSettings(AsyncWebSocketClient * const client)
{
    // Send settings
    webUiSendVtxFrequency(eeprom_storage.vtx_freq, client);
}

void HDZeroMsp::syncSettings(AsyncEventSourceClient * const client)
{
    String json = "{\"vtxfreq\":";
    json += eeprom_storage.vtx_freq;
    json += ",\"recording_control\":1";
    json += ",\"osd_text\":1";
    json += ",\"laptimer\":1";
    json += ",\"vrxversion\":1";
    json += ",\"espnow\":";
    json += ESP_NOW;
    json += ",\"model\":\"HDZero\"";
    json += ",\"osd_row\":";
    json += eeprom_storage.laptimer_osd_pos.row;
    json += ",\"osd_row_max\":";
    json += osdRowMax() - 1;
    json += ",\"osd_col\":";
    json += eeprom_storage.laptimer_osd_pos.column;
    json += ",\"osd_col_max\":";
    json += osdColumnMax() - 1;
    json += ",\"osd_timeout\":";
    json += eeprom_storage.laptimer_osd_timeout / 1000;  // convert to seconds
    json += '}';
    async_event_send(json, "fea_config", client);
    async_event_send(m_version_info, "vrx_version", client);
}

int HDZeroMsp::parseSerialData(uint8_t const chr)
{
    if (_handler.processReceivedByte(chr)) {
        /* Handle the received MSP message */
        mspPacket_t & msp_in = _handler.getPacket();
        String info = "";
        bool ignore = false;

        if (msp_in.type == MSP_PACKET_V2_RESPONSE) {
            if (msp_in.function == HDZ_MSP_FUNC_FIRMWARE_GET) {
                m_version_info = "FW info not available!";
                if ((4 <= msp_in.payloadSize) && (3 == msp_in.payload[0])) {
                    m_version_info = "FW: ";
                    m_version_info += msp_in.payload[1];
                    m_version_info += '.';
                    m_version_info += msp_in.payload[2];
                    m_version_info += '.';
                    m_version_info += msp_in.payload[3];
                }
                info = m_version_info;
                if (current_state < STATE_READY)
                    current_state = STATE_GET_CH_INDEX;

            } else if (msp_in.function == HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_GET) {
                info = "Channel index received: ";
                if (1 == msp_in.payloadSize) {
                    uint16_t const freq = getFreqByIndex(msp_in.payload[0]);
                    info += msp_in.payload[0];
                    info += " -> ";
                    info += freq;
                    info += "MHz";

                    webUiSendVtxFrequency(freq);
#if LOGGER_HDZERO_USE_VTX_INDEX
                    espnow_vtxset_send(msp_in.payload[0]);
#else
                    espnow_vtxset_send(freq);
#endif
                    storeVtxFreq(NULL, freq);

                    if (current_state < STATE_READY)
                        current_state = STATE_GET_FREQ;
                }

            } else if (msp_in.function == HDZ_MSP_FUNC_FREQUENCY_GET) {
                info = "Frequency received: ";
                if (2 == msp_in.payloadSize) {
                    uint16_t const freq = parseFreq(msp_in.payload);
                    info += freq;
                    info += "MHz";

                    webUiSendVtxFrequency(freq);
#if LOGGER_HDZERO_USE_VTX_INDEX
                    espnow_vtxset_send(getIndexByFreq(freq));
#else
                    espnow_vtxset_send(freq);
#endif
                    storeVtxFreq(NULL, freq);

                    if (current_state < STATE_READY)
                        current_state = STATE_GET_RECORDING;
                }

            } else if (msp_in.function == HDZ_MSP_FUNC_RECORDING_STATE_GET) {
                info = "Recording state received: ";
                if (1 == msp_in.payloadSize) {
                    info += msp_in.payload[0] ? "ON" : "OFF";

                    webUiSendVRecordingState(msp_in.payload[0]);
                    m_recording_state = !!msp_in.payload[0];

                    if (current_state < STATE_READY) {
                        check_retry(0, 0);
                        current_state = STATE_CLEAR_OSD;
                        // current_state = STATE_READY;
                    }
                }

            } else if (msp_in.function == HDZ_MSP_FUNC_VRX_MODE_GET) {
                info += "HDZ_MSP_FUNC_VRX_MODE_GET";

            } else if (msp_in.function == HDZ_MSP_FUNC_RSSI_GET) {
                info = "RSSI: 1)";
                if ((5 == msp_in.payloadSize) && (4 == msp_in.payload[0])) {
                    info += msp_in.payload[1];
                    info += " 2)";
                    info += msp_in.payload[2];
                    info += " 3)";
                    info += msp_in.payload[3];
                    info += " 4)";
                    info += msp_in.payload[4];
                }

            } else if (msp_in.function == HDZ_MSP_FUNC_BATTERY_VOLTAGE_GET) {
                info = "Battery Voltage: ";
                if (2 == msp_in.payloadSize) {
                    uint16_t const voltage = ((uint16_t)msp_in.payload[1] << 8) + msp_in.payload[0];
                    info += voltage;
                }
            } else {
                info = "Invalid MSPv2 RESP function received! func: ";
                info += msp_in.function;
            }

        } else if (msp_in.type == MSP_PACKET_V2_COMMAND) {
            ignore = true; // don't forward

            if (msp_in.function == MSP_ELRS_BACKPACK_SET_MODE) {
                info = "BACKPACK Set Mode: ";
                if (1 == msp_in.payloadSize) {
                    uint8_t data = 'F'; // "FAILED" response by default
                    if (msp_in.payload[0] == 'B') {
                        info += "Enter binding...";
                    } else if (msp_in.payload[0] == 'W') {
                        info += "Enable WIFI...";
                        data = 'P'; // "in-progress" response
                        // 'P' = started...
                        // 'O' = success...
                    }
                    sendMspToHdzero(&data, sizeof(data), MSP_ELRS_BACKPACK_SET_MODE, true);
                } else {
                    info += "invalid size!";
                }
            } else if (msp_in.function == MSP_ELRS_BACKPACK_GET_VERSION) {
                info = "BACKPACK get version";
                // Resp max 32 char!
                sendMspToHdzero((uint8_t *)version_string, strlen(version_string), MSP_ELRS_BACKPACK_GET_VERSION, true);

            } else if (msp_in.function == MSP_ELRS_BACKPACK_GET_STATUS) {
                info = "BACKPACK get status";
                enum {
                    WIFI_UPDATE = 1 << 0,
                    BINDING_MODE = 1 << 1,
                    BOUND_OK = 1 << 2,
                };
                uint8_t response[1 + sizeof(eeprom_storage.uid)];
                response[0] = BOUND_OK;
                memcpy(&response[1], eeprom_storage.uid, sizeof(eeprom_storage.uid));
                sendMspToHdzero(response, sizeof(response), MSP_ELRS_BACKPACK_GET_STATUS, true);

            } else if (msp_in.function == MSP_ELRS_BACKPACK_SET_PTR) {
                info = "BACKPACK get status";
                // do nothing...
            } else if (msp_in.function == HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_SET) {
                info = "HDZ set band and channel to ";
                if (1 == msp_in.payloadSize) {
                    uint16_t const band_channel = msp_in.payload[0];
                    uint16_t const freq = getFreqByIndex(band_channel);
                    info += band_channel;
                    info += " => ";
                    info += freq;
                    info += "MHz";

                    webUiSendVtxFrequency(freq);
#if LOGGER_HDZERO_USE_VTX_INDEX
                    espnow_vtxset_send(band_channel);
#else
                    espnow_vtxset_send(freq);
#endif
                    storeVtxFreq(NULL, freq);
                } else {
                    info += "INVALID PAYLOAD LEN!";
                }
            } else {
                info = "Invalid MSPv2 CMD! func: ";
                info += msp_in.function;
            }

        } else if (msp_in.type == MSP_PACKET_ERROR) {
            info = "MSP ERROR! func: ";
            info += msp_in.function;
        } else {
            info = "Invalid MSP received! type: ";
            info += msp_in.type;
            info += ", func: ";
            info += msp_in.function;
        }

        if (info.length()) {
            websocket_send_txt(info);
        } else if (!ignore) {
            espnow_send_msp(msp_in);
        }

        _handler.markPacketFree();
    } else if (!_handler.mspOngoing()) {
        return -1;
    }
    return 0;
}

void HDZeroMsp::vtxFrequencySet(uint16_t const freq, int8_t const power, bool const disable_change)
{
    m_vtx_set_disabled = disable_change;
    if (storeVtxFreq(NULL, freq, disable_change)) {
        // Adjust VRx
        vtx_params_t const params = {.freq = freq, .power = power, .pitmode = false};
        handleVtxFrequencyCommand(params, NULL);
        delay(2);
        // ... GUI is updated when channel query resp is received
#if LOGGER_HDZERO_USE_VTX_INDEX
        getChannelIndex();
#else
        getFrequency();
#endif
        // TODO: forward over esp-now?
        espnow_vtxset_send(freq, (int8_t)power);
    }
}

void HDZeroMsp::OsdShowText(const char * text, uint32_t const timeout_ms)
{
    size_t const len = strlen(text);
    osd_timeout = timeout_ms;
    osdText(text, len, 0, 0);
}

int HDZeroMsp::parseCommandPriv(char const * cmd, size_t const len, AsyncWebSocketClient * const client)
{
    char * temp;
    // ExLRS setting commands
    temp = strstr(cmd, "SET_text=");
    if (temp) {
        handleUserTextCommand(&temp[9], (len - 9), client);
        return 0;
    }
    return -1;
}

int HDZeroMsp::parseCommandPriv(websoc_bin_hdr_t const * const cmd,
                                size_t const len,
                                AsyncWebSocketClient * const client)
{
    if (!len) {
        String settings_out = "[INTERNAL ERROR] something went wrong, payload size is 0!";
        websocket_send_txt(settings_out, client);
        return -1;
    }

    switch (cmd->msg_id) {
        case WSMSGID_RECORDING_CTRL: {
            handleRecordingStateCommand(!!cmd->payload[0]);
            break;
        }
        default:
            return -1;
    }
    return 0;
}

// This is received from outside (ESP-NOW)
int HDZeroMsp::parseCommandPriv(mspPacket_t & msp_in)
{
    /* Just validate the packet and let the espnow handler to forward it */

    /*  HDZero uses only MSP V2 */
    if (msp_in.type == MSP_PACKET_V2_COMMAND || msp_in.type == MSP_PACKET_V2_RESPONSE) {
        switch (msp_in.function) {
            case HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_SET: {
                uint16_t const freq = (1 == msp_in.payloadSize) ? getFreqByIndex(msp_in.payload[0]) : 0;
                if (!freq)
                    return 0;
                checkFreqFromModule(freq); // Broadcasted to other ESP-NOW peers
                break;
            }

            case HDZ_MSP_FUNC_FREQUENCY_SET: {
                uint16_t const freq = (2 <= msp_in.payloadSize) ? parseFreq(msp_in.payload) : 0;
                if (!freq || getIndexByFreq(freq) < 0)
                    return 0;              // ignore invalid freqs
                checkFreqFromModule(freq); // Broadcasted to other ESP-NOW peers
                break;
            }

            case HDZ_MSP_FUNC_RECORDING_STATE_SET: {
                webUiSendVRecordingState(msp_in.payload[0]);
            }

            case HDZ_MSP_FUNC_OSD_ELEMENT_SET: {
                // OSD element write function is diffrent so change it
                msp_in.function = MSP_ELRS_SET_OSD;
                return -1; // Forward to VRX
            }

            default:
                break;
        }

        if ((HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_GET <= msp_in.function && HDZ_MSP_FUNC_SET_RTC >= msp_in.function)) {
            /* Return -1 to get packet written to VRX */
            return -1;
        }
    }
    /* Return 0 to ignore packet */
    return 0;
}

void HDZeroMsp::loop(void)
{
    uint32_t const now = millis();
    switch (current_state) {
        /************** INIT STATES **************/
        case STATE_GET_FW_VER:
#if UART_DEBUG_EN
            check_retry(0, 0);
            current_state = STATE_CLEAR_OSD;
#else
            if (check_retry(now, RETRY_INTERVAL_MS))
                getFwVersion();
#endif
            break;
        case STATE_GET_CH_INDEX:
            if (check_retry(now, RETRY_INTERVAL_MS))
                getChannelIndex();
            break;
        case STATE_GET_FREQ:
            if (check_retry(now, RETRY_INTERVAL_MS))
                getFrequency();
            break;
        case STATE_GET_RECORDING:
            if (check_retry(now, RETRY_INTERVAL_MS))
                getRecordingState();
            break;

        /************** RUNTIME **************/
        case STATE_CLEAR_OSD: {
            if (check_retry(now, osd_timeout)) {
                osdClear();
                current_state = STATE_READY;
            }
            break;
        }
        case STATE_DRAW_OSD: {
            osdDraw();
            break;
        }
        case STATE_READY:
        default:
            check_retry(now, 0); // Reset retry time
            break;
    };
}

void HDZeroMsp::sendMspToHdzero(uint8_t const * const buff,
                                uint16_t const len,
                                uint16_t const function,
                                bool const resp) const
{
    // Send packet to HDZero
    mspPacketType_e const type = (resp ? MSP_PACKET_V2_RESPONSE : MSP_PACKET_V2_COMMAND);
    MSP::sendPacket(_serial, type, function, 0, (!buff ? 0 : len), buff);
}

void HDZeroMsp::handleUserTextCommand(const char * input, size_t const len, AsyncWebSocketClient * const client)
{
    osdText(input, len, 4, 0); // DEBUG
}

void HDZeroMsp::handleVtxFrequencyCommand(vtx_params_t const params, AsyncWebSocketClient * const client)
{
    // Send to HDZero
#if LOGGER_HDZERO_USE_VTX_INDEX
    uint8_t chan_index = (uint8_t)getIndexByFreq(freq);
    if (chan_index == (uint8_t)-1)
        return;
    sendMspToHdzero(&chan_index, 1, HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_SET);
#else
    uint8_t payload[] = {(uint8_t)(params.freq & 0xff), (uint8_t)(params.freq >> 8)};
    sendMspToHdzero(payload, sizeof(payload), HDZ_MSP_FUNC_FREQUENCY_SET);
#endif
}

void HDZeroMsp::handleRecordingStateCommand(uint8_t const start) const
{
    uint16_t const delay_s = 1;
    // Send to HDZero
    uint8_t payload[] = {start, (uint8_t)(delay_s & 0xff), (uint8_t)(delay_s >> 8)};
    sendMspToHdzero(payload, sizeof(payload), HDZ_MSP_FUNC_RECORDING_STATE_SET);
    webUiSendVRecordingState(start);
}

void HDZeroMsp::handleBuzzerCommand(uint16_t const time_ms) const
{
    uint8_t payload[] = {(uint8_t)(time_ms & 0xff), (uint8_t)(time_ms >> 8)};
    sendMspToHdzero(payload, sizeof(payload), HDZ_MSP_FUNC_BUZZER_SET);
}

void HDZeroMsp::getFwVersion(void) const
{
    sendMspToHdzero(NULL, 0, HDZ_MSP_FUNC_FIRMWARE_GET);
}

void HDZeroMsp::getChannelIndex(void) const
{
    sendMspToHdzero(NULL, 0, HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_GET);
}

void HDZeroMsp::getFrequency(void) const
{
    sendMspToHdzero(NULL, 0, HDZ_MSP_FUNC_FREQUENCY_GET);
}

void HDZeroMsp::getRecordingState(void) const
{
    sendMspToHdzero(NULL, 0, HDZ_MSP_FUNC_RECORDING_STATE_GET);
}

void HDZeroMsp::osdClear(void) const
{
    uint8_t payload = OSD_CMD_RELEASE_PORT;
    sendMspToHdzero(&payload, sizeof(payload), MSP_ELRS_SET_OSD);
}

void HDZeroMsp::osdDraw(void)
{
    uint8_t payload = OSD_CMD_SCREEN_DRAW;
    sendMspToHdzero(&payload, sizeof(payload), MSP_ELRS_SET_OSD);
    if (osd_timeout) {
        current_state = STATE_CLEAR_OSD;
    }
}

void HDZeroMsp::osdText(char const * const p_text, size_t len, uint8_t const row, uint8_t const column)
{
    enum {
        OSD_ATTR_PAGE0 = 0,
        OSD_ATTR_PAGE1 = 1,
        OSD_ATTR_BLINK = 0x80,
    };

    if (p_text == NULL || !len)
        return;
    if (64 < len)
        len = 64;
    // Fill OSD data
    uint8_t payload[len + 4];
    payload[0] = OSD_CMD_SCREEN_WRITE;
    payload[1] = row + eeprom_storage.laptimer_osd_pos.row;
    payload[2] = column + eeprom_storage.laptimer_osd_pos.column;
    payload[3] = OSD_ATTR_PAGE0;
    memcpy(&payload[4], p_text, len);
    sendMspToHdzero(payload, sizeof(payload), MSP_ELRS_SET_OSD);
    // Draw OSD
    // current_state = STATE_DRAW_OSD;
    osdDraw();
}

void HDZeroMsp::handleLaptimerState(uint16_t const race_id,
                                    uint16_t const round_num,
                                    bool const state,
                                    AsyncWebSocketClient * const client)
{
    String info = "RACE ";
    info += race_id;
    if (state) {
        info += " START";
        if (round_num) {
            info += "(ROUND ";
            info += round_num;
            info += ')';
        }
    } else {
        info += " END";
    }
    osd_timeout = eeprom_storage.laptimer_osd_timeout;
    osdText(info.c_str(), info.length(), 0, 0);
    if (state) {
        delay(2);
        handleBuzzerCommand(400);
    }
}

void HDZeroMsp::handleLaptimerLap(laptimer_lap_t const * lap, AsyncWebSocketClient * const client)
{
    lap_time_t const laptime = convert_ms_to_time(lap->lap_time_ms);

    String info = "LAP ";
    info += lap->lap_index;
    info += " ";
    info += laptime.m;
    info += ":";
    if (laptime.s < 10)
        info += "0";
    info += laptime.s;
    info += ".";
    if (laptime.ms < 10)
        info += "00";
    else if (laptime.ms < 100)
        info += "0";
    info += laptime.ms;
    osd_timeout = eeprom_storage.laptimer_osd_timeout;
    osdText(info.c_str(), info.length(), 1, 0);
}

void HDZeroMsp::checkFreqFromModule(uint16_t const freq) const
{
    webUiSendVtxFrequency(freq);
    espnow_vtxset_send(freq);
    storeVtxFreq(NULL, freq);
}
