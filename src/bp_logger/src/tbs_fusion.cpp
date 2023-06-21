#include "tbs_fusion.h"
#include "CRSF.h"

#define RETRY_INTERVAL_MS 1000

enum {
    WSMSGID_CRSF_DBG_TLM_BATT = 0x3000,
    WSMSGID_CRSF_DBG_TLM_LINK = 0x3001,
    WSMSGID_CRSF_DBG_TLM_GPS = 0x3002,
    WSMSGID_CRSF_DBG_TLM_BYTES = 0x3003,
    WSMSGID_CRSF_DBG_LAPTIME = 0x3004,
};

const char device_name[] = "ELRS Logger";

static uint32_t ping_sent_ms;
static uint8_t params_count;
static uint8_t params_read;

static CRSF crsf(NULL);

typedef void (*param_set_func_t)(struct crsf_param * const p_param, uint8_t value);

typedef struct crsf_param {
    const char * name;
    const char * params;
    const param_set_func_t callback;
    uint8_t current;
    uint8_t min;
    uint8_t max;
    uint8_t type;
    uint8_t parent; // 0xff = main
} crsf_param_t;

static crsf_param_t crsf_params[] = {
    {.name = "root",
     .params = NULL,
     .callback = NULL,
     .current = 0,
     .min = 0,
     .max = 0,
     .type = CRSF_FOLDER,
     .parent = 0},
    {.name = "Lap Timer",
     .params = NULL,
     .callback = NULL,
     .current = 0,
     .min = 0,
     .max = 0,
     .type = CRSF_FOLDER,
     .parent = 0},
    {.name = "Start",
     .params = "start;stop",
     .callback = NULL,
     .current = 0,
     .min = 0,
     .max = 1,
     .type = CRSF_TEXT_SELECTION,
     .parent = 1},
    {.name = "Reset",
     .params = NULL,
     .callback = NULL,
     .current = 0,
     .min = 0,
     .max = 0,
     .type = CRSF_COMMAND,
     .parent = 1},
};

typedef union {
    struct {
        uint16_t voltage;       // mv * 100
        uint16_t current;       // ma * 100
        uint32_t capacity : 24; // mah
        uint32_t remaining : 8; // %
    } PACKED;
    uint8_t payload[8];
} crsf_battery_info_t;
static crsf_battery_info_t lap_info;


void TbsFusion::init(void)
{
    _handler.markPacketFree();
}

void TbsFusion::syncSettings(void)
{
}

void TbsFusion::syncSettings(AsyncWebSocketClient * const client)
{
    // Send settings
    webUiSendVtxFrequency(eeprom_storage.vtx_freq);

#if 1
    // Ping main CPU
    pingSend();
    ping_sent_ms = millis() + 2000;
#endif
}

void TbsFusion::syncSettings(AsyncEventSourceClient * const client)
{
    String json = "{\"vtxfreq\":";
    json += eeprom_storage.vtx_freq;
    json += ",\"recording_control\":0"; // No support
    json += ",\"osd_text\":0";          // No support atm
    json += ",\"telemetrydebug\":1";
    json += ",\"laptimer\":1";
    json += ",\"espnow\":";
    json += ESP_NOW;
    json += ",\"model\":\"TBS Fusion\"";
    json += '}';
    async_event_send(json, "fea_config", client);
    async_event_send(m_version_info, "vrx_version", client);
}

// Data from Serial input
int TbsFusion::parseSerialData(uint8_t const chr)
{
    String info = "";
    if (_handler.processReceivedByte(chr)) {
        /* Handle the received MSP message */
        mspPacket_t & msp_in = _handler.getPacket();
        if (msp_in.type != MSP_PACKET_V2_COMMAND && msp_in.type != MSP_PACKET_V2_RESPONSE) {
            info = "Invalid MSP received! func: ";
            info += msp_in.function;
        } else {
            // TODO: handle input MSP messages!
        }

        if (info.length()) {
            websocket_send_txt(info);
        } else {
            espnow_send_msp(msp_in);
        }
        _handler.markPacketFree();

    } else if (!_handler.mspOngoing()) {
        crsf_buffer_t const * const p_message = (crsf_buffer_t *)crsf.ParseInByte(chr);
        if (p_message) {
            // Check CRSF input packet
            switch (p_message->type) {
                case CRSF_FRAMETYPE_COMMAND: {
                    if (0x08 == p_message->command.command && 0x02 == p_message->command.sub_command &&
                        /* Co-processor sends this two times */
                        CRSF_ADDRESS_CRSF_RECEIVER == p_message->command.dest_addr) {
                        // Set VTX freq
                        uint16_t freq = p_message->command.payload[0];
                        freq <<= 8;
                        freq += p_message->command.payload[1];
                        webUiSendVtxFrequency(freq);
                        espnow_vtxset_send(freq);
                        storeVtxFreq(NULL, freq);
                    }
                    break;
                }
                case CRSF_FRAMETYPE_PARAMETER_READ: {
                    // p_message->param.read;
                    if (p_message->extended.dest_addr == CRSF_ADDRESS_WIFI) {
                        info = "PARAM READ Field:0x" + String(p_message->param.read.field_id, HEX);
                        info += ", junk:" + String(p_message->param.read.junk_index);
                        param_entry_send(p_message->param.read.field_id, p_message->param.orig_addr);
                    }
                    break;
                }
                case CRSF_FRAMETYPE_PARAMETER_WRITE: {
                    if (p_message->extended.dest_addr == CRSF_ADDRESS_WIFI) {
                        uint8_t const param_index = p_message->param.write.field_id;
                        // p_message->param.write;
                        info = "PARAM WRITE Field:0x" + String(param_index, HEX);
                        info += ", value:" + String(p_message->param.write.value);

                        if (param_index < ARRAY_SIZE(crsf_params)) {
                            crsf_param_t * const p_param = &crsf_params[param_index];
                            if (p_param->callback) {
                                p_param->callback(p_param, p_message->param.write.value);
                                info += " => current:" + String(p_param->current);
                            }
                        }
                    }
                    break;
                }
                case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY: {
                    // DEBUG!
                    if (p_message->extended.orig_addr == CRSF_ADDRESS_VRX) {
                        info = "Entry: ";
                        info += p_message->param.entry.field_id;
                        info += ", parent: ";
                        info += p_message->param.entry.parent;
                        info += ", type: ";
                        info += p_message->param.entry.type;
                        info += ", name: '";
                        info += p_message->param.entry.name;
                        info += "'";
                    }
                    break;
                }
                case CRSF_FRAMETYPE_DEVICE_PING: {
                    // Send device info (CRSF_FRAMETYPE_DEVICE_INFO)
                    size_t const devinfo_size = sizeof(crsf_device_info_msg_header_t) +
                                                sizeof(crsf_device_info_msg_footer_t) + strlen(device_name);
                    crsf_device_info_msg_header_t * devinfo = (crsf_device_info_msg_header_t *)malloc(devinfo_size);
                    if (devinfo) {
                        devinfo->header.device_addr = CRSF_SYNC_BYTE;
                        devinfo->header.frame_size = devinfo_size - CRSF_FRAME_START_BYTES;
                        devinfo->header.type = CRSF_FRAMETYPE_DEVICE_INFO;
                        devinfo->header.dest_addr = p_message->extended.orig_addr;
                        devinfo->header.orig_addr = CRSF_ADDRESS_WIFI;
                        strcpy((char *)devinfo->name, device_name);
                        crsf_device_info_msg_footer_t * footer =
                            (crsf_device_info_msg_footer_t *)&devinfo->name[sizeof(device_name)];
                        footer->serialno = __builtin_bswap32(0x0000BABE);
                        footer->hardware_ver = __builtin_bswap32(0x00000102);
                        footer->software_ver = __builtin_bswap32(0x00000304);
                        footer->field_count = ARRAY_SIZE(crsf_params) - 1;
                        footer->parameter_ver = 1;
                        CrsfWrite((uint8_t *)devinfo, devinfo_size);
                        free(devinfo);
                    }
                    break;
                }
                case CRSF_FRAMETYPE_DEVICE_INFO: {
                    // Print received device info
                    const char * p_dev_name = (char *)p_message->extended.payload;
                    info = "DEVICE_INFO: ";
                    info += p_dev_name;
                    crsf_device_info_msg_footer_t const * const p_footer =
                        (crsf_device_info_msg_footer_t *)&p_dev_name[strlen(p_dev_name) + 1];
                    info += ", num params: ";
                    info += p_footer->field_count;
                    if (p_message->extended.orig_addr == CRSF_ADDRESS_VRX) {
                        params_count = p_footer->field_count + 1; // field_count is the last index
                        params_read = 0;
                    }
                    break;
                }
                default:
                    info = "Unknown CRSF packet: 0x" + String(p_message->type, HEX);
                    break;
            }
            if (info.length())
                websocket_send_txt(info);
        }
        return crsf.IsFrameActive() ? 0 : -1;
    }
    return 0;
}

// From WEB UI
int TbsFusion::parseCommandPriv(char const * cmd, size_t len, AsyncWebSocketClient * const client)
{
    // ExLRS setting commands
    const char * temp = strstr(cmd, "SET_text=");
    if (temp) {
        handleUserTextCommand(&temp[9], (len - 9), client);
        return 0;
    }
    return -1;
}

// From WEB UI
int TbsFusion::parseCommandPriv(websoc_bin_hdr_t const * const cmd, size_t len, AsyncWebSocketClient * const client)
{
    if (!len) {
        String settings_out = "[INTERNAL ERROR] something went wrong, payload size is 0!";
        websocket_send_txt(settings_out, client);
        return -1;
    }

    switch (cmd->msg_id) {
        case WSMSGID_RECORDING_CTRL: {
            break;
        }
        case WSMSGID_CRSF_DBG_TLM_BATT: {
            handleTelemetryBattery(cmd->payload);
            break;
        }
        case WSMSGID_CRSF_DBG_TLM_LINK: {
            handleTelemetryLinkStats(cmd->payload);
            break;
        }
        case WSMSGID_CRSF_DBG_TLM_GPS: {
            handleTelemetryGps(cmd->payload);
            break;
        }
        case WSMSGID_CRSF_DBG_TLM_BYTES: {
            String info = "TLM Bytes ";
            for (uint8_t iter = 0; iter < len; iter++) {
                info += String(cmd->payload[iter], HEX);
                info += ',';
            }
            websocket_send_txt(info, client);
            _serial->write((uint8_t *)cmd->payload, len);
            break;
        }
        case WSMSGID_CRSF_DBG_LAPTIME: {
            laptimer_lap_t lap;
            lap.lap_index = cmd->payload[0];
            lap.lap_time_ms = (uint32_t)cmd->payload[4] << 24;
            lap.lap_time_ms += (uint32_t)cmd->payload[3] << 16;
            lap.lap_time_ms += (uint32_t)cmd->payload[2] << 8;
            lap.lap_time_ms += (uint32_t)cmd->payload[1];
            handleLaptimerLap(&lap, client);
            break;
        }
        default:
            return -1;
    }
    return 0;
}

// This is received from outside (ESP-NOW). Return -1 to get packet written to Serial
int TbsFusion::parseCommandPriv(mspPacket_t & msp_in)
{
    /* Return 0 to ignore packet */
    return 0;
}

void TbsFusion::loop(void)
{
    uint32_t const now_ms = millis();
    if (params_read < params_count && 100 <= (int)(now_ms - ping_sent_ms)) {
        ping_sent_ms = now_ms;
        param_read_send(params_read++, 0);
    }
}

void TbsFusion::sendMspToUart(uint8_t const * const buff, uint16_t const len, uint16_t const function)
{
    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V2_COMMAND;
    msp_out.flags = 0;
    msp_out.function = function;
    msp_out.payloadSize = (!buff) ? 0 : len;
    if (buff && len)
        memcpy((void *)msp_out.payload, buff, len);
    // Push packet to serial
    MSP::sendPacket(&msp_out, _serial);
}

void TbsFusion::CrsfWrite(uint8_t * const buff, size_t size) const
{
    buff[size - 1] = crsf.CalcCRC(&buff[2], (buff[1] - 1));
    _serial->write((uint8_t *)buff, size);
}

void TbsFusion::CrsfWriteCommand(uint8_t * const buff, size_t const size) const
{
    buff[size - 2] = CalcCRC8len(&buff[2], (buff[1] - 2), 0, CRSF_CMD_POLY);
    CrsfWrite(buff, size);
}

void TbsFusion::handleUserTextCommand(const char * input, size_t const len, AsyncWebSocketClient * const client)
{
    if (input == NULL)
        return;

        // TODO: write text to OSD
#if 0
    struct crsf_freq_set {
        crsf_ext_header_t header;
        uint8_t tlm_type;

        // ??

        uint8_t crc;
    } command;
    command.header.device_addr = CRSF_SYNC_BYTE;
    command.header.frame_size = sizeof(command) - CRSF_FRAME_START_BYTES;
    command.header.type = CRSF_FRAMETYPE_ENCAPSULATED_TLM;
    command.header.dest_addr = CRSF_ADDRESS_VRX;
    command.header.orig_addr = CRSF_ADDRESS_WIFI;
    command.tlm_type = CRSF_TLM_TYPE_OSD_TEXT;

    CrsfWrite((uint8_t *)&command, sizeof(command));
#endif

    String dbg_info = "OSD Text: '";
    for (size_t iter = 0; iter < len; iter++)
        dbg_info += input[iter];
    dbg_info += "' (NOT SUPPORTED ATM)";
    websocket_send_txt(dbg_info);
}

void TbsFusion::handleVtxFrequencyCommand(vtx_params_t const params, AsyncWebSocketClient * const client)
{
    struct crsf_freq_set {
        crsf_ext_header_t header;
        uint8_t tlm_type;
        uint8_t dest_addr;
        uint8_t orig_addr;
        uint8_t freq[2]; // big endian
        uint8_t dummy;
        uint8_t crc;
    } command;
    command.header.device_addr = CRSF_SYNC_BYTE;
    command.header.frame_size = sizeof(command) - CRSF_FRAME_START_BYTES;
    command.header.type = CRSF_FRAMETYPE_ENCAPSULATED_TLM;
    command.header.dest_addr = CRSF_ADDRESS_VRX;
    command.header.orig_addr = CRSF_ADDRESS_WIFI;
    command.tlm_type = CRSF_TLM_TYPE_VTX;
    command.dest_addr = CRSF_ADDRESS_VRX;
    command.orig_addr = CRSF_ADDRESS_TBS_AGENT;
    command.freq[0] = params.freq >> 8;   // freq byte 1
    command.freq[1] = params.freq & 0xFF; // freq byte 2
    command.dummy = 1;             // What is this ???
    CrsfWrite((uint8_t *)&command, sizeof(command));
}

void TbsFusion::handleLaptimerLap(laptimer_lap_t const * lap, AsyncWebSocketClient * const client)
{
    String info = "Lap time lap:";
    info += lap->lap_index;
    info += ", ms:";
    info += lap->lap_time_ms;
    info += ", ";
    // Lap time is shown as a battery info...
#if 0
    // Capacity shows the milliseconds + single seconds
    uint32_t const cap = __builtin_bswap32(lap->lap_time_ms % 10000) >> 8;
    lap_info.capacity = cap;
    // Voltage shows minutes.(tens of the seconds)
    uint32_t const minutes = lap->lap_time_ms / 60000;
    uint32_t const seconds = (lap->lap_time_ms - (minutes * 60000)) / 10000;
    lap_info.voltage = __builtin_bswap16((minutes * 10) + seconds);
#else
    // Capacity shows milliseconds
    uint32_t const cap = __builtin_bswap32(lap->lap_time_ms % 1000) >> 8;
    lap_info.capacity = cap;
    // Voltage shows tens of the seconds
    uint32_t const seconds = lap->lap_time_ms / 100;
    lap_info.voltage = __builtin_bswap16(seconds);
#endif
    // Current is average laps of X
    lap_info.current = __builtin_bswap16(168);

    // remaining is the lap index
    lap_info.remaining = lap->lap_index;

    handleTelemetryBattery((uint8_t*)&lap_info);

    websocket_send_txt(info, client);
}

void TbsFusion::handleTelemetryGps(uint8_t const * const payload)
{
    struct crsf_gps_set_t {
        crsf_ext_header_t header;
        uint8_t tlm_type;
        union {
            struct {
                int32_t latitude;
                int32_t longitude;
                uint16_t speed;
                uint16_t heading;
                uint16_t altitude;
                uint8_t satellites;
            } PACKED;
            uint8_t payload[15];
        };
        uint8_t crc;
    } PACKED command;
    command.header.device_addr = CRSF_SYNC_BYTE;
    command.header.frame_size = sizeof(command) - CRSF_FRAME_START_BYTES;
    command.header.type = CRSF_FRAMETYPE_ENCAPSULATED_TLM;
    command.header.dest_addr = CRSF_ADDRESS_BROADCAST; // CRSF_ADDRESS_VRX;
    command.header.orig_addr = CRSF_ADDRESS_BROADCAST; // CRSF_ADDRESS_WIFI;
    command.tlm_type = CRSF_TLM_TYPE_GPS;
    memcpy(command.payload, payload, sizeof(command.payload));
    CrsfWrite((uint8_t *)&command, sizeof(command));
}

void TbsFusion::handleTelemetryLinkStats(uint8_t const * const payload)
{
    struct crsf_tlm_command_link_stat_t {
        crsf_ext_header_t header;
        uint8_t tlm_type;
        union {
            crsfLinkStatistics_t link_stat;
            uint8_t payload[10];
        };
        uint8_t crc;
    } PACKED command;
    command.header.device_addr = CRSF_SYNC_BYTE;
    command.header.frame_size = sizeof(command) - CRSF_FRAME_START_BYTES;
    command.header.type = CRSF_FRAMETYPE_ENCAPSULATED_TLM;
    command.header.dest_addr = CRSF_ADDRESS_BROADCAST; // CRSF_ADDRESS_VRX;
    command.header.orig_addr = CRSF_ADDRESS_BROADCAST; // CRSF_ADDRESS_WIFI;
    command.tlm_type = CRSF_TLM_TYPE_LINK_STATISTICS;
    memcpy(command.payload, payload, sizeof(command.payload));
    CrsfWrite((uint8_t *)&command, sizeof(command));
}

void TbsFusion::handleTelemetryBattery(uint8_t const * const payload)
{
    struct crsf_tlm_command_battery_t {
        crsf_ext_header_t header;
        uint8_t tlm_type;
        crsf_battery_info_t sensor;
        uint8_t crc;
    } PACKED command;
    command.header.device_addr = CRSF_SYNC_BYTE;
    command.header.frame_size = sizeof(command) - CRSF_FRAME_START_BYTES;
    command.header.type = CRSF_FRAMETYPE_ENCAPSULATED_TLM;
    command.header.dest_addr = CRSF_ADDRESS_VRX;
    command.header.orig_addr = CRSF_ADDRESS_WIFI;
    command.tlm_type = CRSF_TLM_TYPE_BATTERY_SENSOR;
    memcpy(command.sensor.payload, payload, sizeof(command.sensor));
    CrsfWrite((uint8_t *)&command, sizeof(command));
}

void TbsFusion::pingSend(void) const
{
    crsf_device_info_ping_msg_t ping;
    ping.header.device_addr = CRSF_SYNC_BYTE;
    ping.header.frame_size = sizeof(ping) - CRSF_FRAME_START_BYTES;
    ping.header.type = CRSF_FRAMETYPE_DEVICE_PING;
    ping.header.dest_addr = CRSF_ADDRESS_BROADCAST;
    ping.header.orig_addr = CRSF_ADDRESS_WIFI;
    CrsfWrite((uint8_t *)&ping, sizeof(ping));
}

void TbsFusion::param_read_send(uint8_t const field_id, uint8_t const junk) const
{
    crsf_param_read_t read;
    read.header.device_addr = CRSF_SYNC_BYTE;
    read.header.frame_size = sizeof(read) - CRSF_FRAME_START_BYTES;
    read.header.type = CRSF_FRAMETYPE_PARAMETER_READ;
    read.header.dest_addr = CRSF_ADDRESS_VRX;
    read.header.orig_addr = CRSF_ADDRESS_WIFI;
    read.field_id = field_id;
    read.junk_index = junk;
    CrsfWrite((uint8_t *)&read, sizeof(read));
}

bool TbsFusion::param_entry_send(uint8_t const param_index, uint8_t const dest_addr, const char * info_msg)
{
    crsf_param_entry_hdr_t * p_header = NULL;
    if (ARRAY_SIZE(crsf_params) <= param_index)
        return false;
    crsf_param_t const * const p_param = &crsf_params[param_index];
    size_t const name_len = strlen(p_param->name);
    size_t resp_size = sizeof(crsf_param_entry_hdr_t) + name_len;

    if (p_param->type == CRSF_FOLDER) {
        uint8_t children = 0;
        uint8_t child[8];
        for (uint8_t iter = 0; iter < ARRAY_SIZE(crsf_params) && iter < sizeof(child); iter++) {
            if (crsf_params[param_index].parent == param_index) {
                child[children++] = iter;
            }
        }
        resp_size += sizeof(crsf_param_entry_footer_folder_t) + children;
        p_header = (crsf_param_entry_hdr_t *)malloc(resp_size);
        if (p_header) {
            crsf_param_entry_footer_folder_t * p_footer =
                (crsf_param_entry_footer_folder_t *)&p_header->name[name_len + 1];
            memcpy(p_footer->childs, child, children);
            p_footer->childs[children] = 0xff;
        }

    } else if (p_param->type == CRSF_COMMAND) {
        resp_size += sizeof(crsf_param_entry_footer_command_t);
        resp_size += !!info_msg ? strlen(info_msg) : 0;
        p_header = (crsf_param_entry_hdr_t *)malloc(resp_size);
        if (p_header) {
            crsf_param_entry_footer_command_t * p_footer =
                (crsf_param_entry_footer_command_t *)&p_header->name[name_len + 1];
            p_footer->value = p_param->current;
            p_footer->timeout = 20;
            p_footer->info_msg[0] = 0;  // terminate string
            if (info_msg) {
                strcpy(p_footer->info_msg, info_msg);
            }
        }

    } else if (p_param->type == CRSF_TEXT_SELECTION) {
        size_t const params_len = strlen(p_param->params);
        resp_size += sizeof(crsf_param_entry_footer_text_selection_t) + params_len;
        p_header = (crsf_param_entry_hdr_t *)malloc(resp_size);
        if (p_header) {
            crsf_param_entry_footer_text_selection_t * p_footer =
                (crsf_param_entry_footer_text_selection_t *)&p_header->name[name_len + 1];
            strcpy(p_footer->options, p_param->params);
            p_footer = (crsf_param_entry_footer_text_selection_t *)&p_footer->options[params_len];
            p_footer->value = p_param->current;
            p_footer->min = p_param->min;
            p_footer->max = p_param->max;
            p_footer->def = 0;
            p_footer->endmark = 0;
        }
    }

    if (p_header) {

        // TODO: split into pieces (max size == 64B) if needed and send junks

        p_header->header.device_addr = CRSF_SYNC_BYTE;
        p_header->header.frame_size = resp_size - CRSF_FRAME_START_BYTES;
        p_header->header.type = CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY;
        p_header->header.dest_addr = dest_addr;
        p_header->header.orig_addr = CRSF_ADDRESS_WIFI;
        p_header->field_id = param_index;
        p_header->junks_remain = 0;
        p_header->parent = p_param->parent;
        p_header->type = p_param->type;
        strcpy(p_header->name, p_param->name);

        CrsfWrite((uint8_t *)p_header, resp_size);
        free((void *)p_header);
    }
    return true;
}
