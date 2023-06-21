#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#ifdef ARDUINO_ARCH_ESP32
#include <esp_wifi.h>
#include <ESPmDNS.h>
#include <Update.h>
#define U_PART U_SPIFFS
#else
// #include <Hash.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <Updater.h>
#define U_PART U_FS
#endif
#if USE_LITTLE_FS // Must be included here to make 6.1x pio happy
#include <LittleFS.h>
#else
#include <SPIFFS.h>
#endif

#include "storage.h"
#include "main.h"
#include "stm32_ota.h"
#include "stm32Updater.h"
#include "common_defs.h"
#include "html_default.h"
#include "led.h"
#include "comm_espnow.h"
#include "expresslrs_msp.h"
#include "hdzero_msp.h"
#include "tbs_fusion.h"
#include "buzzer.h"

#if CONFIG_HDZERO || CONFIG_TBS_FUSION
#include "backpack_vrx.h"
#else
#include "backpack_tx.h"
#endif

#ifndef SERIAL_INVERTED
#define SERIAL_INVERTED 0
#endif

#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "ExpressLRS AP"
#endif
#ifndef WIFI_AP_PSK
#define WIFI_AP_PSK "expresslrs"
#endif
#ifndef WIFI_AP_HIDDEN
#define WIFI_AP_HIDDEN 0
#endif
#ifndef WIFI_AP_MAX_CONN
#define WIFI_AP_MAX_CONN 2
#elif WIFI_AP_MAX_CONN < 1
#error "WIFI_AP_MAX_CONN min is 1"
#elif 8 < WIFI_AP_MAX_CONN
#error "WIFI_AP_MAX_CONN max is 8"
#endif

#if CONFIG_HANDSET
#define WIFI_AP_SUFFIX " HANDSET"
#elif CONFIG_HDZERO
#define WIFI_AP_SUFFIX " HDZero"
#undef WIFI_AP_HIDDEN
#define WIFI_AP_HIDDEN 0
#elif CONFIG_TBS_FUSION
#define WIFI_AP_SUFFIX " FusionVRX"
#else
#define WIFI_AP_SUFFIX " MODULE"
#endif

#ifndef WIFI_SEARCH_RSSI_MIN
#define WIFI_SEARCH_RSSI_MIN -100
#endif

#ifndef WIFI_TIMEOUT
#define WIFI_TIMEOUT 60 // default to 1min
#endif

#ifndef UART_DEBUG_EN
#define UART_DEBUG_EN 0
#endif

static enum state {
    STATE_RUNNING = 0,
    STATE_WIFI_SCAN_INIT,
    STATE_WIFI_SCAN_START,
    STATE_WIFI_SCANNING,
    STATE_WIFI_SCAN_LAPTIMER,
    STATE_WIFI_SELECT,
    STATE_WIFI_CONNECT,
    STATE_WIFI_WAIT,
    STATE_WIFI_START_AP,
    STATE_WIFI_RECONFIGURE_ESPNOW,
    STATE_WS_CLIENT_CONNECTED,
    STATE_UPGRADE_STM,
    STATE_LAPTIMER_REGISTER,
    STATE_LAPTIMER_WAIT,
    STATE_REBOOT,
} current_state;
static uint32_t reboot_req_ms;

typedef struct {
    uint8_t network_index;
    uint8_t channel;
} wifi_info_t;
static wifi_info_t wifi_search_results;
static uint32_t wifi_connect_started;
static uint32_t wifi_scan_count;
static uint8_t laptimer_channel;

AsyncWebServer server(80);
AsyncEventSource events("/events");
AsyncWebSocket webSocket("/ws");
AsyncWebSocketClient * g_ws_client;

#ifndef LOGGER_HOST_NAME
#define LOGGER_HOST_NAME "elrs_logger"
#endif
static const char hostname[] = LOGGER_HOST_NAME;
static const char target_name[] = STR(TARGET_NAME);
const char version_string[] = LATEST_COMMIT_STR;

String boot_log = "";

uint8_t char_to_dec(uint8_t const chr)
{
    if ('0' <= chr && chr <= '9')
        return chr - '0';
    return 0;
}

uint8_t hex_char_to_dec(uint8_t const chr)
{
    if ('A' <= chr && chr <= 'F')
        return (10 + (chr - 'A'));
    if ('a' <= chr && chr <= 'f')
        return (10 + (chr - 'a'));
    return char_to_dec(chr);
}

uint8_t wifi_ap_channel_get(void)
{
    // Calculate WiFi channel (1...12) according to UID
    uint8_t const * my_uid = eeprom_storage.uid;
    return ((my_uid[3] + my_uid[4] + my_uid[5]) % 12) + 1;
}

/*************************************************************************/

class CtrlSerialPrivate : public CtrlSerial
{
public:
    size_t available(void)
    {
        return Serial.available();
    }
    uint8_t read(void)
    {
        return Serial.read();
    }

    void write(uint8_t * buffer, size_t size)
    {
        Serial.write(buffer, size);
    }
};

CtrlSerialPrivate my_ctrl_serial;
CtrlSerial & ctrl_serial = my_ctrl_serial;

#if CONFIG_HDZERO
HDZeroMsp msp_handler_obj(&my_ctrl_serial);
#elif CONFIG_TBS_FUSION
TbsFusion msp_handler_obj(&my_ctrl_serial);
#else
ExpresslrsMsp msp_handler_obj(&my_ctrl_serial);
#endif
MspHandlerBase & msp_handler = msp_handler_obj;

/*************************************************************************/

#ifdef ARDUINO_ARCH_ESP32
#include <rom/rtc.h>
String get_reset_reason(RESET_REASON const reason)
{
    switch (reason) {
        case 1:
            return "POWERON_RESET"; /**<1, Vbat power on reset*/
        case 3:
            return "SW_RESET"; /**<3, Software reset digital core*/
        case 4:
            return "OWDT_RESET"; /**<4, Legacy watch dog reset digital core*/
        case 5:
            return "DEEPSLEEP_RESET"; /**<5, Deep Sleep reset digital core*/
        case 6:
            return "SDIO_RESET"; /**<6, Reset by SLC module, reset digital core*/
        case 7:
            return "TG0WDT_SYS_RESET"; /**<7, Timer Group0 Watch dog reset digital core*/
        case 8:
            return "TG1WDT_SYS_RESET"; /**<8, Timer Group1 Watch dog reset digital core*/
        case 9:
            return "RTCWDT_SYS_RESET"; /**<9, RTC Watch dog Reset digital core*/
        case 10:
            return "INTRUSION_RESET"; /**<10, Instrusion tested to reset CPU*/
        case 11:
            return "TGWDT_CPU_RESET"; /**<11, Time Group reset CPU*/
        case 12:
            return "SW_CPU_RESET"; /**<12, Software reset CPU*/
        case 13:
            return "RTCWDT_CPU_RESET"; /**<13, RTC Watch dog Reset CPU*/
        case 14:
            return "EXT_CPU_RESET"; /**<14, for APP CPU, reseted by PRO CPU*/
        case 15:
            return "RTCWDT_BROWN_OUT_RESET"; /**<15, Reset when the vdd voltage is not stable*/
        case 16:
            return "RTCWDT_RTC_RESET"; /**<16, RTC Watch dog reset digital core and rtc module*/
        default:
            return "NO_MEAN";
    }
}
void print_reset_reason(void)
{
    boot_log += "Reset CPU0: ";
    boot_log += get_reset_reason(rtc_get_reset_reason(0));
    boot_log += ", CPU1: ";
    boot_log += get_reset_reason(rtc_get_reset_reason(1));
}
#else
void print_reset_reason(void)
{
    rst_info * resetInfo;
    resetInfo = ESP.getResetInfoPtr();

    switch (resetInfo->reason) {
        case REASON_WDT_RST:
            /* 1 = hardware watch dog reset */
            boot_log += "Reset: HW WD";
            break;
        case REASON_EXCEPTION_RST:
            /* 2 = exception reset, GPIO status won’t change */
            boot_log += "Reset: Exception";
            break;
        case REASON_SOFT_WDT_RST:
            /* 3 = software watch dog reset, GPIO status won’t change */
            boot_log += "Reset: SW WD";
            break;
        case REASON_SOFT_RESTART:
            /* 4 = software restart ,system_restart , GPIO status won’t change */
            boot_log += "Reset: SW restart";
            break;
        case REASON_DEEP_SLEEP_AWAKE:
            /* 5 = wake up from deep-sleep */
            boot_log += "Reset: deep sleep wakeup";
            break;
        case REASON_EXT_SYS_RST:
            /* 6 = external system reset */
            boot_log += "Reset: External";
            break;
        case REASON_DEFAULT_RST:
        default:
            /* 0 = normal startup by power on */
            boot_log += "Reset: normal power on";
            break;
    }
}
#endif

void wifi_networks_report(AsyncWebSocketClient * client)
{
    if (eeprom_storage.wifi_is_valid()) {
        // CMD_WIFINETS</\/\IDXMACSSID>
        String info_str = "CMD_WIFINETS";
        for (size_t index = 0; index < ARRAY_SIZE(eeprom_storage.wifi_nets); index++) {
            wifi_networks_t const * const wifi_ptr = &eeprom_storage.wifi_nets[index];
            if (wifi_is_psk_valid(wifi_ptr) && (wifi_is_ssid_valid(wifi_ptr) || wifi_is_mac_valid(wifi_ptr))) {
                info_str += "/\\/\\";
                if (index < 10)
                    info_str += '0';
                info_str += index;
                if (wifi_is_mac_valid(wifi_ptr)) {
                    info_str += mac_addr_print(wifi_ptr->mac);
                } else {
                    info_str += "00:00:00:00:00:00";
                }
                if (wifi_is_ssid_valid(wifi_ptr)) {
                    info_str += wifi_ptr->ssid;
                }
            }
        }
        client->text(info_str);
    }

    if (wifi_is_mac_valid(&eeprom_storage.laptimer) || wifi_is_ssid_valid(&eeprom_storage.laptimer)) {
        String info_str = "CMD_LAPTIMER=";
        if (wifi_is_mac_valid(&eeprom_storage.laptimer)) {
            info_str += mac_addr_print(eeprom_storage.laptimer.mac);
        } else {
            info_str += "00:00:00:00:00:00";
        }
        if (wifi_is_ssid_valid(&eeprom_storage.laptimer)) {
            info_str += eeprom_storage.laptimer.ssid;
        }
        client->text(info_str);
    }
}

/*************************************************************************/

void laptimer_start_stop(bool const state)
{
    int8_t const node = eeprom_storage.laptimer_config.index;
    if (/*node < 0 ||*/ state == msp_handler.LaptimerStateGet()) {
        return;
    }
    if (state) {
        espnow_laptimer_start_send(node);
    } else {
        espnow_laptimer_stop_send(node);
    }
}

void recording_start_stop(bool const start)
{
    if (start != msp_handler.RecordingStateGet()) {
#if 0
        if (start) {
            espnow_laptimer_start_send();
        } else {
            espnow_laptimer_stop_send();
        }
#endif
    }
}

/*************************************************************************/

int esp_now_msp_rcvd(mspPacket_t & msp_pkt)
{
    if (msp_pkt.type == MSP_PACKET_V1_ELRS && msp_pkt.function == ELRS_INT_MSP_ESPNOW_UPDATE &&
        msp_pkt.flags == MSP_ELRS_INT) {
        struct espnow_update const * const update = (struct espnow_update *)msp_pkt.payload;
        uint8_t channel;
#ifdef ARDUINO_ARCH_ESP32
        wifi_second_chan_t secondChan;
        esp_wifi_get_channel(&channel, &secondChan);
        (void)secondChan;
#else
        channel = wifi_get_channel();
#endif
#if UART_DEBUG_EN
        Serial.printf("MSP_ESPNOW_UPDATE: current %u, new %u\r\n", channel, update->channel);
#endif
        if (update->channel != channel) {
            /* Reconfigure wifi channel and (re)start AP */
            wifi_search_results.channel = update->channel;
            current_state = STATE_WIFI_START_AP;
        }
    } else if (msp_pkt.function == MSP_LAP_TIMER) {
        if (msp_pkt.type == MSP_PACKET_V2_RESPONSE) {
            laptimer_messages_t const * const p_command = (laptimer_messages_t *)msp_pkt.payload;
            switch (p_command->subcommand) {
                case CMD_LAP_TIMER_REGISTER: {
                    uint16_t const freq = p_command->register_resp.freq;
                    String info = "LAPTIMER COMMAND: ";
                    info += msp_handler.vtxFreqGetBandChannel(freq);
                    if (p_command->register_resp.node_index) {
                        info += ", NODE: ";
                        info += p_command->register_resp.node_index;
                    }
                    websocket_send_txt(info);
#if UART_DEBUG_EN
                    Serial.printf("CMD_LAP_TIMER_REGISTER: freq %u, node_index %u\r\n", freq,
                                  p_command->register_resp.node_index);
#endif
                    eeprom_storage.laptimer_config.index = p_command->register_resp.node_index;

                    msp_handler.OsdShowText(info, 10000); // 10sec
                    msp_handler.vtxFrequencySet(freq, 0, true); // force 25mW

                    if (current_state == STATE_LAPTIMER_WAIT)
                        current_state = STATE_RUNNING;
                    break;
                }
                case CMD_LAP_TIMER_START: {
#if UART_DEBUG_EN
                    Serial.printf("CMD_LAP_TIMER_START: race_id: %u, node_index: %u, round_num: %u\r\n",
                                  p_command->start.race_id, p_command->start.node_index, p_command->start.round_id);
#endif
                    msp_handler.LaptimerStateStart(p_command->start.race_id, p_command->start.round_id);
                    break;
                }
                case CMD_LAP_TIMER_STOP: {
#if UART_DEBUG_EN
                    Serial.printf("CMD_LAP_TIMER_STOP: race_id: %u, node_index: %u, round_num: %u\r\n",
                                  p_command->stop.race_id, p_command->stop.node_index, p_command->stop.round_id);
#endif
                    msp_handler.LaptimerStateStop(p_command->stop.race_id, p_command->stop.round_id);
                    break;
                }
                case CMD_LAP_TIMER_LAP: {
#if UART_DEBUG_EN
                    Serial.printf("CMD_LAP_TIMER_LAP: lap: %u = %ums, race_id: %u, node_index: %u\r\n",
                                  p_command->lap.lap_index, p_command->lap.lap_time_ms, p_command->lap.race_id,
                                  p_command->lap.node_index);
#endif
                    msp_handler.LaptimerLap(&p_command->lap);
                    // TODO: store laps internally if client drops for some reason??
                    break;
                }
                default:
#if UART_DEBUG_EN
                    Serial.printf("MSP_LAP_TIMER: invalid subcommand %u\r\n", p_command->subcommand);
#endif
                    break;
            }
        } else {
#if UART_DEBUG_EN
            Serial.printf("MSP_LAP_TIMER: REQUEST received.. ignored\r\n");
#endif
        }
    } else if (msp_handler.parseCommand(msp_pkt) < 0) {
        // Not handler internally, pass to serial
        MSP::sendPacket(&msp_pkt, &ctrl_serial);
    }
    return 0;
}

/*************************************************************************/
String mac_addr_print(uint8_t const * const mac_addr)
{
    char macStr[18] = {0};
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
            mac_addr[5]);
    return String(macStr);
}

void websocket_send_txt(char const * data, AsyncWebSocketClient * const client)
{
    if (client)
        client->text(data);
    else
        webSocket.textAll(data);
}

void websocket_send_txt(String & data, AsyncWebSocketClient * const client)
{
    if (!data.length())
        return;
    websocket_send_txt(data.c_str(), client);
}

void websocket_send_bin(uint8_t const * data, uint8_t const len, AsyncWebSocketClient * const client)
{
    if (!len || !data)
        return;
    if (client)
        client->binary((char *)data, (size_t)len);
    else
        webSocket.binaryAll((char *)data, (size_t)len);
}

static void websocket_send_initial_data(AsyncWebSocketClient * const client)
{
    IPAddress my_ip;
    my_ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP() : WiFi.softAPIP();
    String info_str = "NA";
    info_str = "My IP address = ";
    info_str += my_ip.toString();
    info_str += " rssi: ";
    info_str += WiFi.RSSI();

    client->text(info_str);
    client->text(espnow_get_info());
#if ESP_NOW
    info_str = "ESP-NOW channel: ";
    info_str += espnow_channel();
    client->text(info_str);
#endif

    if (boot_log)
        client->text(boot_log);

    msp_handler.printConnectionInfo(client);

#if ESP_NOW
    if (eeprom_storage.espnow_initialized == LOGGER_ESPNOW_INIT_KEY) {
        uint8_t const size = eeprom_storage.espnow_clients_count * 6;
        if (size) {
            uint8_t buffer[size + 2];
            buffer[0] = (uint8_t)(WSMSGID_ESPNOW_ADDRS >> 8);
            buffer[1] = (uint8_t)(WSMSGID_ESPNOW_ADDRS);
            memcpy(&buffer[2], eeprom_storage.espnow_clients, size);
            client->binary(buffer, sizeof(buffer));
        }
    }
#endif

    uint8_t bind_addr[2 + sizeof(eeprom_storage.uid)] = {
        (WSMSGID_ESPNOW_BIND_ADDR >> 8), WSMSGID_ESPNOW_BIND_ADDR & 0xff};
    memcpy(&bind_addr[2], eeprom_storage.uid, sizeof(eeprom_storage.uid));
    client->binary(bind_addr, sizeof(bind_addr));

    wifi_networks_report(client);

    msp_handler.syncSettings(client);
}

void webSocketEvent(AsyncWebSocket * server,
                    AsyncWebSocketClient * client,
                    AwsEventType type,
                    void * arg,
                    uint8_t * payload,
                    size_t length)
{
    switch (type) {
        case WS_EVT_DISCONNECT:
            break;

        case WS_EVT_CONNECT: {
            g_ws_client = client;
            current_state = STATE_WS_CLIENT_CONNECTED;
            break;
        }

        case WS_EVT_DATA: {
            AwsFrameInfo const * const info = (AwsFrameInfo *)arg;
            if (info->final && info->index == 0 && info->len == length) {
                // the whole message is in a single frame and we got all of it's data
                if (info->opcode == WS_TEXT) {
                    size_t index;
                    char const * temp = strstr((char *)payload, "WIFIADD/");
                    if (temp) { // WiFi network add
                        temp += 8;
                        wifi_networks_t * wifi_ptr = NULL;
                        for (index = 0; index < ARRAY_SIZE(eeprom_storage.wifi_nets); index++, wifi_ptr = NULL) {
                            wifi_ptr = &eeprom_storage.wifi_nets[index];
                            if (!wifi_is_ssid_valid(wifi_ptr) && !wifi_is_psk_valid(wifi_ptr) &&
                                !wifi_is_mac_valid(wifi_ptr)) {
                                // Free slot found
                                break;
                            }
                        }
                        if (wifi_ptr) {
                            size_t len;
                            // Parse SSID
                            len = (10 * char_to_dec(*temp++));
                            len += char_to_dec(*temp++);
                            temp += 1; // Skip '/'
                            memcpy(wifi_ptr->ssid, temp, len);
                            wifi_ptr->ssid[len] = 0;
                            temp += len + 1; // Skip SSID + '/'
                            // Parse PSK
                            len = (10 * char_to_dec(*temp++));
                            len += char_to_dec(*temp++);
                            temp += 1; // Skip '/'
                            memcpy(wifi_ptr->psk, temp, len);
                            wifi_ptr->psk[len] = 0;
                            temp += len; // Skip PSK
                            // Parse MAC if included
                            if ((((uintptr_t)temp - (uintptr_t)payload) + 1 + 12) <= length) {
                                temp += 1; // Skip '/'
                                wifi_ptr->mac[0] = (char_to_dec(temp[0]) << 4) + char_to_dec(temp[1]);
                                wifi_ptr->mac[1] = (char_to_dec(temp[2]) << 4) + char_to_dec(temp[3]);
                                wifi_ptr->mac[2] = (char_to_dec(temp[4]) << 4) + char_to_dec(temp[5]);
                                wifi_ptr->mac[3] = (char_to_dec(temp[6]) << 4) + char_to_dec(temp[7]);
                                wifi_ptr->mac[4] = (char_to_dec(temp[8]) << 4) + char_to_dec(temp[9]);
                                wifi_ptr->mac[5] = (char_to_dec(temp[10]) << 4) + char_to_dec(temp[11]);
                            }
                            eeprom_storage.markDirty();
                            wifi_networks_report(client);
                        } else {
                            client->text("WIFIADD: No free slots!");
                        }
                        break;
                    }
                    temp = strstr((char *)payload, "WIFIDEL/");
                    if (temp) { // WiFi network remove
                        temp += 8;
                        if ((((uintptr_t)temp - (uintptr_t)payload) + 2) <= length) {
                            // Parse index
                            index = (10 * char_to_dec(*temp++));
                            index += char_to_dec(*temp++);
                            if (index < ARRAY_SIZE(eeprom_storage.wifi_nets)) {
                                wifi_networks_t * const wifi_ptr = &eeprom_storage.wifi_nets[index];
                                memset(wifi_ptr, 0, sizeof(*wifi_ptr));
                                eeprom_storage.markDirty();
                                wifi_networks_report(client);
                            } else {
                                client->text("WIFIDEL: Invalid index!");
                            }
                        } else {
                            client->text("WIFIDEL: Invalid msg len!");
                        }
                        break;
                    }

                    msp_handler.parseCommand((char *)payload, length, client);

                } else if (info->opcode == WS_BINARY) {
                    websoc_bin_hdr_t const * const header = (websoc_bin_hdr_t *)payload;
                    length -= sizeof(header->msg_id);

                    // ====================== LAPTIMER COMMANDS ==============
                    if (WSMSGID_LAPTIMER_START_STOP == header->msg_id) {
                        laptimer_start_stop(!!header->payload[0]);

                        // ====================== ESP-NOW COMMANDS ==============
                    } else if (WSMSGID_ESPNOW_ADDRS == header->msg_id) {
                        // ESP-Now client list
                        espnow_update_clients(header->payload, length);
                        client->text(espnow_get_info());

                    } else if (WSMSGID_ESPNOW_BIND_ADDR == header->msg_id) {
                        // Bind address update
                        if (length == sizeof(eeprom_storage.uid)) {
                            client->text("BIND ADDR will be updated and reboot...!");
                            memcpy(eeprom_storage.uid, header->payload, sizeof(eeprom_storage.uid));
                            eeprom_storage.uid[0] &= ~0x1; // Make it MAC address compatible
                            eeprom_storage.markDirty();
                            eeprom_storage.save();
                            // Forced reboot to make change permanent
                            current_state = STATE_REBOOT;
                            reboot_req_ms = millis() + 500;
                        }

#if CONFIG_STM_UPDATER
                    } else if (WSMSGID_STM32_RESET == header->msg_id) {
                        // ====================== STM COMMANDS ==================
                        // STM reset
                        reset_stm32_to_app_mode();
#endif // CONFIG_STM_UPDATER

                    } else if (msp_handler.parseCommand(header, length, client) < 0) {
                        // ====================== DEFAULT =======================
                        String error = "Invalid message: 0x";
                        error += String(header->msg_id, HEX);
                        client->text(error);
                    }
                }
            }
            break;
        }
        default:
            break;
    }
}

/***********************************************************************************/

void async_event_send(String & data, const char * event, AsyncEventSourceClient * const client)
{
    if (client) {
        client->send(data.c_str(), event);
        return;
    }
    events.send(data.c_str(), event);
}

void async_event_handler(AsyncEventSourceClient * client)
{
#if UART_DEBUG_EN
    if (client->lastId()) {
        Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
#endif
    // send event with message "hello!", id current millis
    // and set reconnect delay to 5 second
    client->send("keepalive", NULL, millis(), 5000);
    // Send settings to client
    msp_handler.syncSettings(client);

    String json = "{\"laptimer\":{\"ssid\":\"";
    if (wifi_is_ssid_valid(&eeprom_storage.laptimer)) {
        json += eeprom_storage.laptimer.ssid;
    }
    json += "\",\"mac\":\"";
    if (wifi_is_mac_valid(&eeprom_storage.laptimer)) {
        json += mac_addr_print(eeprom_storage.laptimer.mac);
    }
    json += "\",\"pilot\":\"";
    if (eeprom_storage.laptimer_config.pilot_name) {
        json += eeprom_storage.laptimer_config.pilot_name;
    }
    json += "\"}}";
    async_event_send(json, "laptimer", client);
}

/***********************************************************************************/

static String getContentType(String const filename)
{
    if (filename.endsWith(".html"))
        return "text/html";
    else if (filename.endsWith(".css"))
        return "text/css";
    else if (filename.endsWith(".js"))
        return "text/javascript";
    else if (filename.endsWith(".ico"))
        return "image/x-icon";
    else if (filename.endsWith(".svg"))
        return "image/svg+xml";
    else if (filename.endsWith(".gz"))
        return "application/x-gzip";
    else if (filename.endsWith(".json"))
        return "application/json";
    return "text/plain";
}

static void sendReturn(AsyncWebServerRequest * request)
{
    request->send_P(200, "text/html", GO_BACK);
}

static void handle_recover(AsyncWebServerRequest * request)
{
    request->send_P(200, "text/html", RECOVER_HTML);
}

static void handleUpdatePage(AsyncWebServerRequest * request)
{
    const char html[] = "<form method='POST' action='/doUpdate' enctype='multipart/form-data'>"
                        "<input type='file' name='update'><input type='submit' value='Update'></form>";
    request->send(200, "text/html", html);
}

static void handleDoUpdate(
    AsyncWebServerRequest * request, const String & filename, size_t index, uint8_t * data, size_t len, bool final)
{
    if (!index) {
#if UART_DEBUG_EN
        Serial.println("Upgrade starts: " + filename);
#endif
        // Check for the filesystem partition update
        int const cmd = (filename.indexOf("spiffs") > -1 || filename.indexOf("_fs_data") > -1) ? U_PART : U_FLASH;
#ifdef ARDUINO_ARCH_ESP8266
        Update.runAsync(true);
        if (!Update.begin(request->contentLength(), cmd))
#else
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd))
#endif
        {
            Update.printError(Serial);
        }
    }

    if (Update.write(data, len) != len) {
        Update.printError(Serial);
#if defined(ARDUINO_ARCH_ESP8266) && UART_DEBUG_EN
    } else {
        // Serial.printf("Progress: %d%%\n", (Update.progress()*100)/Update.size());
#endif
    }

    if (final) {
        AsyncWebServerResponse * response = request->beginResponse(302, "text/plain", "Upload ready");
        response->addHeader("Refresh", "20");
        response->addHeader("Location", "/");
        response->addHeader("Connection", "close");
        request->send(response);
        request->client()->close();

        delay(100);
        bool const success = Update.end(true);

        if (!success) {
            Update.printError(Serial);
        } else {
#if UART_DEBUG_EN
            Serial.println("Update complete");
#endif
            current_state = STATE_REBOOT;
            reboot_req_ms = millis() + 500;
        }
    }
}

#if CONFIG_STM_UPDATER
// handles uploads to the filserver
static void
handleUploads(AsyncWebServerRequest * request, String filename, size_t index, uint8_t * data, size_t len, bool final)
{
    String logmessage = "";
    static bool stm32_upgrade;

    if (!index) {
        logmessage = "Uploading file: " + filename;
        stm32_upgrade = stm32_ota_check_filename(filename);
        if (stm32_upgrade) {
            // STM32 firmwware update
            stm32_ota_parse_args(request);
            logmessage += " -> STM32 upgrade!";
        }
        // open the file on first call and store the file handle in the request object
        request->_tempFile = FILESYSTEM.open("/" + filename, "w");
        if (!request->_tempFile)
            request->send(500, "text/plain", "500: couldn't create file");
        websocket_send_txt(logmessage);
#if UART_DEBUG_EN
        Serial.println(logmessage);
#endif
    }

    if (len) {
        // stream the incoming chunk to the opened file
        request->_tempFile.write(data, len);
        // logmessage = "  ** Writing index:" + String(index) + " len:" + String(len);
        // websocket_send_txt(logmessage);
#if UART_DEBUG_EN
        // Serial.println(logmessage);
#endif
    }

    if (final) {
        logmessage = "  ** Upload Completed! size: ";
        logmessage += (index + len);
        // close the file handle as the upload is now done
        request->_tempFile.close();
        websocket_send_txt(logmessage);

#if UART_DEBUG_EN
        Serial.println(logmessage);
#endif
        request->redirect("/");
        if (stm32_upgrade)
            current_state = STATE_UPGRADE_STM;
    }
}
#endif // CONFIG_STM_UPDATER

static void handle_reset_elrs_cmd(AsyncWebServerRequest * request)
{
#if CONFIG_STM_UPDATER
    // STM reset
    reset_stm32_to_app_mode();
    request->send(200);
#else
    request->send(404, "text/plain", "Not supported");
#endif
}

static void handleMacAddress(AsyncWebServerRequest * request)
{
    uint8_t channel;
#ifdef ARDUINO_ARCH_ESP32
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&channel, &secondChan);
    (void)secondChan;
#else
    channel = wifi_get_channel();
#endif

    String message = "WiFi STA MAC: ";
    message += WiFi.macAddress();
    message += "\n  - channel in use: ";
    message += channel;
    message += "\n  - mode: ";
    message += (uint8_t)WiFi.getMode();
    message += "\n\nWiFi SoftAP MAC: ";
    message += WiFi.softAPmacAddress();
    message += "\n  - IP: ";
    message += WiFi.softAPIP().toString();
    message += "\n";
    request->send(200, "text/plain", message);
}

static void handleApInfo(AsyncWebServerRequest * request)
{
    String message = "== WiFi AP ==\n  - MAC: ";
    message += WiFi.softAPmacAddress();
    message += "\n  - IP: 192.168.4.1 / 24";
    message += "\n  - AP channel: ";
    message += wifi_ap_channel_get();
    message += "\n  - SSID: ";
    message += WIFI_AP_SSID;
    message += WIFI_AP_SUFFIX;
    message += "\n  - PSK: ";
    message += WIFI_AP_PSK;
    message += "\n  - Hidden: ";
    message += !!WIFI_AP_HIDDEN;
    message += "\n  - Conn max: ";
    message += WIFI_AP_MAX_CONN;
    message += "\n\n  - MY UID: ";
    uint8_t const * uid = eeprom_storage.uid;
    message += mac_addr_print(uid);
    request->send(200, "text/plain", message);
}

static void handle_fs(AsyncWebServerRequest * request)
{
    String message = "FS info: used ";
#ifdef ARDUINO_ARCH_ESP32
    message += SPIFFS.usedBytes();
    message += "/";
    message += SPIFFS.totalBytes();
    message += "\n**** FS files ****\n";

    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        message += file.name();
        if (!file.isDirectory()) {
            message += " - ";
            message += file.size();
            message += "B";
        }
        message += "\n";
        file = root.openNextFile();
    }
#else
    FSInfo fs_info;
    FILESYSTEM.info(fs_info);
    message += fs_info.usedBytes;
    message += "/";
    message += fs_info.totalBytes;
    message += "\n**** FS files ****\n";

    Dir dir = FILESYSTEM.openDir("/");
    while (dir.next()) {
        message += dir.fileName();
        if (dir.fileSize()) {
            File f = dir.openFile("r");
            message += " - ";
            message += f.size();
            message += "B";
        }
        message += "\n";
    }
#endif
    request->send(200, "text/plain", message);
}

static void handle_fs_format(AsyncWebServerRequest * request)
{
    FILESYSTEM.format();
    request->send(200, "text/plain", "Filesystem formatted");
}

static void handleFileRead(AsyncWebServerRequest * request)
{
    bool gzipped = false;
    String path = request->url();
    if (path.endsWith("/"))
        // Send the index file if a folder is requested
        path = "/index.html";
    static struct {
        const char * url;
        const char * contentType;
        const uint8_t * content;
        const size_t size;
    } files[] = {
        {"/index.html",  "text/html",       (uint8_t *)INDEX_HTML, sizeof(INDEX_HTML)},
        {"/style.css",   "text/css",        (uint8_t *)STYLE_CSS,  sizeof(STYLE_CSS) },
#if ELRS_JS_IMPL
        {"/elrs.js",     "text/javascript", (uint8_t *)ELRS_JS,    sizeof(ELRS_JS)   },
#endif
#if VRX_JS_IMPL
        {"/vrx.js",      "text/javascript", (uint8_t *)VRX_JS,     sizeof(VRX_JS)    },
#endif
        {"/common.js",   "text/javascript", (uint8_t *)COMMON_JS,  sizeof(COMMON_JS) },
#if FAVICON_IMPL
        {"/favicon.ico", "image/x-icon",    (uint8_t *)FAVICON,    sizeof(FAVICON)   },
#endif
    };
    // Check if matching to builtin files
    for (size_t i = 0; i < ARRAY_SIZE(files); i++) {
        if (path.equals(files[i].url)) {
            AsyncWebServerResponse * response =
                request->beginResponse_P(200, files[i].contentType, files[i].content, files[i].size);
            response->addHeader("Content-Encoding", "gzip");
            request->send(response);
            return;
        }
    }
    // Get the MIME type
    String contentType = getContentType(path);
    if (FILESYSTEM.exists(path + ".gz")) {
        path += ".gz";
        gzipped = true;
    } else if (!FILESYSTEM.exists(path)) {
        handle_recover(request);
        return;
    }
    /* file found - send it */
    AsyncWebServerResponse * response = request->beginResponse(FILESYSTEM, path, contentType);
    if (gzipped)
        response->addHeader("Content-Encoding", "gzip");
    request->send(response);
}

static void handleLaptimerConfig(AsyncWebServerRequest * request)
{
    for (size_t i = 0; i < request->args(); i++) {
        String name = request->argName(i);
        String value = request->arg(i);
        uint32_t const len = value.length();
        const char * p_value = value.c_str();
        // Serial.printf(" arg: %s = %s\r\n", name.c_str(), p_value);
        if (name == "ssid") {
            if (len < sizeof(eeprom_storage.laptimer.ssid)) {
                memcpy(eeprom_storage.laptimer.ssid, p_value, len);
                eeprom_storage.laptimer.ssid[len] = 0;
            }
        } else if (name == "macaddr" && len == 17) {
            for (uint8_t iter = 0; iter < 6; iter++) {
                eeprom_storage.laptimer.mac[iter] = strtol(&p_value[iter * 3], NULL, 16);
            }
        } else if (name == "pilot" && len < sizeof(eeprom_storage.laptimer_config.pilot_name)) {
            memcpy(eeprom_storage.laptimer_config.pilot_name, p_value, len);
            eeprom_storage.laptimer_config.pilot_name[len] = 0;
            eeprom_storage.laptimer_config.index = -1;
        }
    }

#if UART_DEBUG_EN
    Serial.print("=== LapTimer ===\r\n");
    Serial.print("   SSID: ");
    Serial.println(eeprom_storage.laptimer.ssid);
    Serial.print("   MAC: ");
    Serial.println(mac_addr_print(eeprom_storage.laptimer.mac));
    Serial.print("   PILOT: ");
    Serial.println(eeprom_storage.laptimer_config.pilot_name);
#endif

    eeprom_storage.markDirty();

    // Trigger scan for LapTimer's SSID
#ifdef ARDUINO_ARCH_ESP32
    WiFi.scanNetworks(true, true, 0, 300, 0, eeprom_storage.laptimer.ssid);
#else
    WiFi.scanNetworks(true, true, 0, (uint8_t *)eeprom_storage.laptimer.ssid);
#endif
    current_state = STATE_WIFI_SCAN_LAPTIMER;

    sendReturn(request);
}

static void handle_reboot_cmd(AsyncWebServerRequest * request)
{
    (void)request;
    current_state = STATE_REBOOT;
    reboot_req_ms = millis() + 50;
}

/*************************************************/

#ifdef ARDUINO_ARCH_ESP32
void onStationConnected(arduino_event_id_t event)
#else
void onStationConnected(const WiFiEventStationModeConnected & evt)
#endif
{
    // Don't let AP to be triggered while still connecting to STA
    wifi_connect_started = millis();
#if UART_DEBUG_EN
    Serial.printf("Wifi_STA_connect - RSSI=%ddBm\r\n", WiFi.RSSI());
#endif
}

#ifdef ARDUINO_ARCH_ESP32
void onStationDisconnected(arduino_event_id_t event)
#else
void onStationDisconnected(const WiFiEventStationModeDisconnected & evt)
#endif
{
#if UART_DEBUG_EN
    Serial.printf("Wifi_STA_diconnect with reason: %d\r\n",
#ifdef ARDUINO_ARCH_ESP32
                  event
#else
                  evt.reason
#endif
    );
#endif
    if (STATE_WIFI_WAIT != current_state
#ifdef ARDUINO_ARCH_ESP32
    /* && SYSTEM_EVENT_STA_DISCONNECTED == event */
#else
        && WIFI_DISCONNECT_REASON_ASSOC_LEAVE != evt.reason
#endif
    ) {
#if UART_DEBUG_EN
        Serial.println("  CONNECTION LOST! Start new scan...");
#endif
        /* Start scan again */
        current_state = STATE_WIFI_SCAN_INIT;
    }
    MDNS.end();
    WiFi.reconnect(); // Force reconnect
}

#ifdef ARDUINO_ARCH_ESP32
void onStationGotIP(arduino_event_id_t event)
#else
void onStationGotIP(const WiFiEventStationModeGotIP & evt)
#endif
{
#if UART_DEBUG_EN
    Serial.print("WiFi STA got IP: ");
    Serial.println(WiFi.localIP().toString());
#endif

    current_state = STATE_RUNNING;

    String instance = String(hostname) + "_" + WiFi.macAddress();
    instance.replace(":", "");

    MDNS.end();
#ifdef ARDUINO_ARCH_ESP32
    if (MDNS.begin(hostname)) {
        MDNS.setInstanceName(instance);
        MDNS.addService("http", "tcp", 80);
        MDNS.addServiceTxt("http", "tcp", "vendor", "elrs");
        MDNS.addServiceTxt("http", "tcp", "target", (const char *)target_name);
        MDNS.addServiceTxt("http", "tcp", "version", LATEST_COMMIT_STR);
        MDNS.addServiceTxt("http", "tcp", "type", "bp_logger");
    }
#else
    MDNS.setHostname(hostname);
    if (MDNS.begin(hostname, WiFi.localIP())) {
        MDNS.setInstanceName(hostname);
        MDNSResponder::hMDNSService service = MDNS.addService(instance.c_str(), "http", "tcp", 80);
        MDNS.addServiceTxt(service, "vendor", "elrs");
        MDNS.addServiceTxt(service, "target", target_name);
        MDNS.addServiceTxt(service, "version", LATEST_COMMIT_STR);
        MDNS.addServiceTxt(service, "type", "bp_logger");

        MDNS.addService(instance.c_str(), "ws", "tcp", 81);

        // If the probe result fails because there is another device on the network with the same name
        // use our unique instance name as the hostname. A better way to do this would be to use
        // MDNSResponder::indexDomain and change wifi_hostname as well.
        MDNS.setHostProbeResultCallback([instance](const char * p_pcDomainName, bool p_bProbeResult) {
            if (!p_bProbeResult) {
                WiFi.hostname(instance);
                MDNS.setInstanceName(instance);
            }
        });
    }
#endif

    led_set(LED_WIFI_STA);
    /*buzzer_beep(440, 30);
    delay(200);
    buzzer_beep(440, 30);*/

    // Configure ESP-NOW
    uint8_t channel;
#ifdef ARDUINO_ARCH_ESP32
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&channel, &secondChan);
    (void)secondChan;
#else
    channel = wifi_get_channel();
#endif
    espnow_init(channel, esp_now_msp_rcvd);
    if (laptimer_channel) {
        current_state = STATE_LAPTIMER_REGISTER;
    }
}

void onStationDhcpTimeout(void)
{
    /* Start scan again */
    current_state = STATE_WIFI_SCAN_INIT;
    MDNS.end();
#if UART_DEBUG_EN
    Serial.println("WiFi STA DHCP timeout");
#endif
}

static void wifi_set_ap_mac(void)
{
    /* Set AP MAC to UID for ESP-NOW messaging */
    uint8_t * ap_mac = eeprom_storage.uid;
#ifdef ARDUINO_ARCH_ESP32
    if (ESP_OK != esp_wifi_set_mac(WIFI_IF_AP, ap_mac)) {
#if UART_DEBUG_EN
        Serial.println("ERROR! Unable to set MAC address!");
#endif
    }
#else
    wifi_set_macaddr(SOFTAP_IF, ap_mac);
#endif
}

static void wifi_config_ap(uint8_t const channel)
{
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);

    // esp_wifi_set_channel(chan,WIFI_SECOND_CHAN_NONE);

    // WiFi not connected, Start access point
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    if (WiFi.softAP(WIFI_AP_SSID WIFI_AP_SUFFIX, WIFI_AP_PSK, channel, !!WIFI_AP_HIDDEN, WIFI_AP_MAX_CONN)) {
        current_state = STATE_RUNNING;
        led_set(LED_WIFI_AP);
        buzzer_beep(400, 20);
#if UART_DEBUG_EN
        Serial.printf("WiFi AP '%s' on channel:%u hidden:%u (PSK:'%s')\r\n", WIFI_AP_SSID WIFI_AP_SUFFIX, channel,
                      !!WIFI_AP_HIDDEN, WIFI_AP_PSK);
#endif
#ifdef ARDUINO_ARCH_ESP32
        wifi_set_ap_mac();
#endif

        // Configure ESP-NOW
        espnow_init(channel, esp_now_msp_rcvd);

        if (laptimer_channel) {
            current_state = STATE_LAPTIMER_REGISTER;
        }
    } else {
        current_state = STATE_WIFI_SCAN_INIT;
        WiFi.mode(WIFI_OFF);
#if UART_DEBUG_EN
        Serial.println("WiFi AP start fail");
#endif
    }
}

static void wifi_connect(wifi_info_t & results)
{
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPdisconnect();
    WiFi.disconnect();

    wifi_networks_t const * const wifi_ptr = &eeprom_storage.wifi_nets[results.network_index];
#if UART_DEBUG_EN
    Serial.print("WiFi connecting: '");
    Serial.print(wifi_ptr->ssid);
    Serial.println("'");
#endif
    WiFi.begin(wifi_ptr->ssid, wifi_ptr->psk, results.channel);
    wifi_connect_started = millis();

#ifdef ARDUINO_ARCH_ESP32
    wifi_set_ap_mac();
#endif

    // Initial ESP-NOW configuration
    espnow_init(results.channel, esp_now_msp_rcvd);
}

static void wifi_config(void)
{
#ifdef ARDUINO_ARCH_ESP32
    WiFi.setHostname(hostname);
#else
    wifi_station_set_hostname(hostname);
#endif

#ifdef ARDUINO_ARCH_ESP8266
    wifi_set_ap_mac();
#endif

    /* Force WIFI off until it is realy needed */
    WiFi.mode(WIFI_OFF);
#ifdef ARDUINO_ARCH_ESP8266
    WiFi.forceSleepBegin();
#endif
    delay(10);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    WiFi.disconnect(true);
    WiFi.softAPdisconnect(false);
#ifdef ARDUINO_ARCH_ESP32
    WiFi.setTxPower(WIFI_POWER_13dBm);
#else
    WiFi.setOutputPower(13);
    // WiFi.setPhyMode(WIFI_PHY_MODE_11N);
#endif
    /* STA mode callbacks */
#ifdef ARDUINO_ARCH_ESP32
    WiFi.onEvent(onStationConnected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
    WiFi.onEvent(onStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.onEvent(onStationGotIP, ARDUINO_EVENT_WIFI_STA_GOT_IP);
    // WiFi.onEvent(onStationDhcpTimeout, ARDUINO_EVENT_WIFI_STA_LOST_IP);
#else
    static WiFiEventHandler stationConnectedHandler = WiFi.onStationModeConnected(&onStationConnected);
    static WiFiEventHandler stationDisconnectedHandler = WiFi.onStationModeDisconnected(&onStationDisconnected);
    static WiFiEventHandler stationGotIpAddress = WiFi.onStationModeGotIP(&onStationGotIP);
    static WiFiEventHandler stationDhcpTimeout = WiFi.onStationModeDHCPTimeout(&onStationDhcpTimeout);
#endif
#ifdef ARDUINO_ARCH_ESP8266
    WiFi.forceSleepWake();
#endif
}

static void wifi_check(void)
{
#define WIFI_LOOP_TIMEOUT (WIFI_TIMEOUT * 1000)
    static uint32_t wifi_last_check, led_state;

    uint32_t const now = millis();
    if (WIFI_LOOP_TIMEOUT < (now - wifi_connect_started)) {
#if UART_DEBUG_EN
        Serial.println("WIFI config timeout... start AP");
#endif
        current_state = STATE_WIFI_START_AP;
    } else if (500 <= (now - wifi_last_check)) {
        wifi_last_check = now;
        /* Blink led */
        led_set(led_state ? LED_WIFI_STA : LED_OFF);
        led_state ^= 1;
    }
}

static void wifi_config_server(void)
{
#ifdef ARDUINO_ARCH_ESP32
    // ESP32 crashes if the WiFi is not up while configuring services
    WiFi.mode(WIFI_AP_STA);
#endif
    server.on("/fs", handle_fs);
    server.on("/fs-format", handle_fs_format);
    server.on("/return", sendReturn);
    server.on("/mac", handleMacAddress);
    server.on("/ap", handleApInfo);
    server.on("/laptimer_config", HTTP_POST, handleLaptimerConfig);
    server.on("/reboot", handle_reboot_cmd);

#if CONFIG_STM_UPDATER
    // STM32 OTA upgrade
    server.on(
        "/upload", HTTP_POST, [](AsyncWebServerRequest * request) {}, handleUploads);
#endif
    server.on("/reset", handle_reset_elrs_cmd); // reset ELRS TX co-processor

    /* ESP OTA firmware upgrade */
    server.on("/update", HTTP_GET, handleUpdatePage);
    server.on(
        "/doUpdate", HTTP_POST, [](AsyncWebServerRequest * request) {}, handleDoUpdate);

    server.onNotFound(handleFileRead);

    // Config events
    events.onConnect(async_event_handler);
    server.addHandler(&events);

    // Config web socket
    webSocket.onEvent(webSocketEvent);
    server.addHandler(&webSocket);

    server.begin();
}

#if ARDUINO_ARCH_ESP32
#define ENC_TYPE_NONE WIFI_AUTH_OPEN
#endif

static void wifi_scan_ready(int const numberOfNetworks)
{
    int rssi_best = WIFI_SEARCH_RSSI_MIN;
    const char * own_ap_ssid = WIFI_AP_SSID;
    const size_t own_ap_ssid_len = strlen(own_ap_ssid);

#if UART_DEBUG_EN
    Serial.println("-------------------------");
    Serial.println(" Available WiFi networks");
    Serial.println("-------------------------");
#endif

    for (int iter = 0; iter < numberOfNetworks; iter++) {
        String ssid;
        uint8_t encryptionType;
        int32_t rssi;
        uint8_t * mac;
        int32_t channel;
        bool hidden = false;

#ifdef ARDUINO_ARCH_ESP32
        WiFi.getNetworkInfo(iter, ssid, encryptionType, rssi, mac, channel);
#else
        WiFi.getNetworkInfo(iter, ssid, encryptionType, rssi, mac, channel, hidden);
#endif

#if UART_DEBUG_EN
        Serial.printf("  %d: '%s', Ch:%d (%ddBm) BSSID:%s %s %s\r\n", (iter + 1), ssid.c_str(), channel, rssi,
                      WiFi.BSSIDstr(iter).c_str(), encryptionType == ENC_TYPE_NONE ? "open" : "",
                      hidden ? "hidden" : "");
#endif
        (void)hidden;

        if ((wifi_is_mac_valid(&eeprom_storage.laptimer) &&
             memcmp(eeprom_storage.laptimer.mac, mac, sizeof(eeprom_storage.laptimer.mac)) == 0) ||
            (wifi_is_ssid_valid(&eeprom_storage.laptimer) &&
             strncmp(eeprom_storage.laptimer.ssid, ssid.c_str(), sizeof(eeprom_storage.laptimer.ssid)) == 0)) {
#if UART_DEBUG_EN
            Serial.println("    ** LapTimer found!");
#endif
            laptimer_channel = channel;

            // Store Laptimer's MAC address for ESP-NOW configuration
            if (!wifi_is_mac_valid(&eeprom_storage.laptimer)) {
#if UART_DEBUG_EN
                Serial.println("    ** Laptimer MAC updated...");
#endif
                memcpy(eeprom_storage.laptimer.mac, mac, sizeof(eeprom_storage.laptimer.mac));
                eeprom_storage.markDirty();
            }
            continue;
        }

        if (rssi < WIFI_SEARCH_RSSI_MIN)
            continue; // Ignore very bad networks

        if ((strncmp(ssid.c_str(), own_ap_ssid, own_ap_ssid_len) == 0) ||
            (memcmp(mac, eeprom_storage.uid, sizeof(eeprom_storage.uid)) == 0)) {
            // Match to own access points -> use this since EPS-NOW channel must match
            if (wifi_search_results.network_index == UINT8_MAX) {
                wifi_search_results.channel = channel;
            }
#if UART_DEBUG_EN
            Serial.println("    ** My logger AP");
#endif
            continue;
        }

        for (size_t jter = 0; jter < ARRAY_SIZE(eeprom_storage.wifi_nets); jter++) {
            wifi_networks_t const * const wifi_ptr = &eeprom_storage.wifi_nets[jter];
            if ((wifi_is_mac_valid(wifi_ptr) && memcmp(wifi_ptr->mac, mac, sizeof(wifi_ptr->mac)) == 0) ||
                (wifi_is_ssid_valid(wifi_ptr) && strncmp(wifi_ptr->ssid, ssid.c_str(), sizeof(wifi_ptr->ssid)) == 0)) {
#if UART_DEBUG_EN
                Serial.print("    ** Configured network found! ");
#endif
                if (encryptionType != ENC_TYPE_NONE && !wifi_is_psk_valid(wifi_ptr)) {
                    /* No PSK and encryption is enabled -> ignore */
#if UART_DEBUG_EN
                    Serial.println(" - ignore: No PSK");
#endif
                    continue;
                }
                // Select based on the best RSSI
                if (rssi_best < rssi) {
                    rssi_best = rssi;
                    wifi_search_results.network_index = jter;
                    wifi_search_results.channel = channel;
#if UART_DEBUG_EN
                    Serial.print(" << selected! idx:");
                    Serial.print(jter);
#endif
                }
#if UART_DEBUG_EN
                Serial.println();
#endif
                break;
            }
        }
    }

    // Cleanup scan results
    WiFi.scanDelete();
    // Stop scan if home network found or count exeeded
    if ((wifi_search_results.network_index != UINT8_MAX) || (3 <= ++wifi_scan_count)) {
        current_state = STATE_WIFI_SELECT;
    }
}

void setup()
{
    // ESP.eraseConfig();

    boot_log = "";
    print_reset_reason();

    Serial.setRxBufferSize(512);
#ifdef ARDUINO_ARCH_ESP32
    Serial.begin(SERIAL_BAUD, SERIAL_8N1, -1, -1, SERIAL_INVERTED);
#else
    Serial.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_FULL, 1, SERIAL_INVERTED);
#endif
    delay(500);

    eeprom_storage.setup();

    msp_handler.init();

    buzzer_init();

    led_init();
    led_set(LED_INIT);

#if CONFIG_STM_UPDATER && (BOOT0_PIN == 2 || BOOT0_PIN == 0)
    reset_stm32_to_app_mode();
#endif

#ifdef ARDUINO_ARCH_ESP32
    SPIFFS.begin(true);
#else
    FILESYSTEM.begin();
    // FILESYSTEM.format();
#endif

#if UART_DEBUG_EN
#warning "Serial debugging enabled!"
    Serial.println();
    Serial.println("========================");
    Serial.println("  ELRS Logger started");
    Serial.println("========================");
#endif

    wifi_config();
    wifi_config_server();

    msp_handler.syncSettings();

#if UART_DEBUG_EN && 0
    for (size_t jter = 0; jter < ARRAY_SIZE(eeprom_storage.wifi_nets); jter++) {
        wifi_networks_t const * const wifi_ptr = &eeprom_storage.wifi_nets[jter];
        Serial.printf("=== [%u] ===\r\n", jter);
        Serial.print("   SSID: ");
        Serial.println(wifi_ptr->ssid);
        Serial.print("   MAC: ");
        Serial.println(mac_addr_print(wifi_ptr->mac));
        Serial.print("   PSK: ");
        Serial.println(wifi_ptr->psk);
    }
#endif

    wifi_scan_count = 0;
    current_state = STATE_WIFI_SCAN_INIT;
}

int serialEvent(String & log_string)
{
    int temp;
    uint8_t inChar;
    while (Serial.available()) {
        temp = Serial.read();
        if (temp < 0)
            break;

        inChar = (uint8_t)temp;

        if (msp_handler.parseSerialData(inChar) < 0) {
            if (inChar == '\r')
                continue;
            else if (inChar == '\n')
                return 0;
            // add if printable char
            if (isprint(inChar))
                log_string += (char)inChar;
            // for safety... send in pieces
            if (70 <= log_string.length())
                return 0;
        }
    }
    return -1;
}

void loop()
{
    static uint32_t laptimer_wait_ms;
    static String input_log_string = "";

    if (0 <= serialEvent(input_log_string)) {
#if CONFIG_HDZERO
        if (!strstr(input_log_string.c_str(), "Resource\\_000.bmp") &&
            !strstr(input_log_string.c_str(), "fc_variant:  ...")) // IGNORE dummy dbg print
#endif
            websocket_send_txt(input_log_string);
        input_log_string = "";
    }
#ifdef ARDUINO_ARCH_ESP8266
    ESP.wdtFeed();
#endif

#if defined(ARDUINO_ARCH_ESP8266)
    MDNS.update();
#endif

    msp_handler.loop();

    eeprom_storage.update();

#ifdef ARDUINO_ARCH_ESP8266
    ESP.wdtFeed();
#endif

    uint32_t const now = millis();

    switch (current_state) {
        case STATE_WIFI_SCAN_INIT: {
            wifi_scan_count = 0;
            laptimer_channel = 0;
            wifi_search_results.network_index = UINT8_MAX;
            wifi_search_results.channel = wifi_ap_channel_get();
            current_state = STATE_WIFI_SCAN_START;
            break;
        }
        case STATE_WIFI_SCAN_START: {
            // Start async scan and wait for results
            WiFi.scanNetworks(true, true);
            current_state = STATE_WIFI_SCANNING;
#if UART_DEBUG_EN
            Serial.println("WiFi scan started");
#endif
            break;
        }
        case STATE_WIFI_SCAN_LAPTIMER:
        case STATE_WIFI_SCANNING: {
            static uint32_t scan_led_last_ms;
            static bool scan_led_state;
            scan_led_state ^= 1;
            if (400 <= (now - scan_led_last_ms)) {
                scan_led_last_ms = now;
                /* Blink led */
                led_set(scan_led_state ? LED_INIT : LED_OFF);
            }
            int const numNetworks = WiFi.scanComplete();
            if (0 <= numNetworks) {
                wifi_scan_ready(numNetworks);

            } else if (-2 == numNetworks) {
#if UART_DEBUG_EN
                Serial.println("WiFi scan end - start AP");
#endif
                wifi_search_results.channel = wifi_ap_channel_get();
                current_state = STATE_WIFI_START_AP;
            }
            break;
        }
        case STATE_WIFI_SELECT: {
#if UART_DEBUG_EN
            Serial.println("...Select WiFi mode...");
#endif
            if (laptimer_channel && laptimer_channel != wifi_search_results.channel) {
#if UART_DEBUG_EN
                Serial.println("  ---laptimer on different channel, jump on it");
#endif
                /* Force AP mode and set onto same channel with the laptimer */
                wifi_search_results.network_index = UINT8_MAX;
                wifi_search_results.channel = laptimer_channel;
            }

            if (wifi_search_results.network_index == UINT8_MAX) {
                current_state = STATE_WIFI_START_AP;
            } else {
                current_state = STATE_WIFI_CONNECT;
            }
#if UART_DEBUG_EN
            Serial.printf("Change state to %u\r\n", current_state);
#endif
            break;
        }
        case STATE_WIFI_CONNECT: {
            if (!WiFi.isConnected()) {
                wifi_connect(wifi_search_results);
                current_state = STATE_WIFI_WAIT;
            } else {
                current_state = STATE_RUNNING;
            }
            break;
        }
        case STATE_WIFI_WAIT: {
            wifi_check();
            break;
        }
        case STATE_WIFI_START_AP: {
            wifi_config_ap(wifi_search_results.channel);
            break;
        }
        case STATE_REBOOT: {
            if (0 <= (int32_t)(now - reboot_req_ms)) {
                WiFi.mode(WIFI_OFF);
                Serial.flush();
                delay(100);
                ESP.restart();
            }
            break;
        }
        case STATE_WIFI_RECONFIGURE_ESPNOW: {
            espnow_send_update_channel(wifi_search_results.channel);
            current_state = STATE_WIFI_START_AP;
            break;
        }
        case STATE_WS_CLIENT_CONNECTED: {
            // Send initial config data to just connected WS client
            if (g_ws_client) {
                websocket_send_initial_data(g_ws_client);
            }
            g_ws_client = NULL;
            current_state = STATE_RUNNING;
            break;
        }
        case STATE_UPGRADE_STM: {
            stm32_ota_do_flash();
            current_state = STATE_RUNNING;
            break;
        }

        case STATE_LAPTIMER_REGISTER: {
            espnow_laptimer_register_send();
            current_state = STATE_LAPTIMER_WAIT;
            laptimer_wait_ms = now;
            break;
        }
        case STATE_LAPTIMER_WAIT: {
            // Wait couple of seconds and resend if no response received
            if (2000 <= (now - laptimer_wait_ms)) {
                // give up...
#if UART_DEBUG_EN
                Serial.println("No response from Laptimer!");
#endif
                current_state = STATE_RUNNING;
            }
            break;
        }
        default:
            break;
    }
}
