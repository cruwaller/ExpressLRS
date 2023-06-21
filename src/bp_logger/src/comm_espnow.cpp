#include "comm_espnow.h"
#include "main.h"
#include "storage.h"
#include "platform.h"
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#include <esp_now.h>
#else
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif

#define MAC_ADDR_LEN      6
#define DEBUG_TX_CALLBACK 0
#define DEBUG_LOG         0
#define BIND_EN           0

static String espnow_init_info = "";
#if DEBUG_LOG
#define LOG_SET(...) espnow_init_info = __VA_ARGS__
#define LOG_ADD(...) espnow_init_info += __VA_ARGS__
#else
#define LOG_SET(...)
#define LOG_ADD(...)
#endif

#if ESP_NOW
static uint8_t msp_tx_buffer[128];
static uint32_t esp_now_channel;
static esp_now_msp_rcvd_cb_t msp_handler;
static bool esp_now_initialized = false;

#if BIND_EN
static uint8_t msp_bind_key[MAC_ADDR_LEN + 1] = {0x0}; // UID[6] + channel

static void send_binding_message(bool const req, uint8_t receiver[MAC_ADDR_LEN])
{
    size_t const len = MSP::bufferPacket(msp_tx_buffer, (req ? SP_PACKET_V2_COMMAND : MSP_PACKET_V2_RESPONSE),
                                         MSP_ESPNOW_BIND_FUNC, 0, sizeof(msp_bind_key), msp_bind_key);
    if (len) {
        esp_now_send(receiver, msp_tx_buffer, len);
    }
}

static bool validate_bind_key(uint8_t * key)
{
    return (memcmp(key, msp_bind_key, sizeof(MAC_ADDR_LEN)) == 0);
}
#endif

static void add_peer(uint8_t const * const mac_addr, uint32_t const channel)
{
    LOG_ADD("[PEER ");
    LOG_ADD(mac_addr_print(mac_addr));
#ifdef ARDUINO_ARCH_ESP32
    esp_now_peer_info_t peer_info = {
        .peer_addr = {0}, .lmk = {0}, .channel = (uint8_t)channel, .ifidx = WIFI_IF_AP, .encrypt = false, .priv = NULL};
    memcpy(peer_info.peer_addr, mac_addr, sizeof(peer_info.peer_addr));
    if (esp_now_add_peer(&peer_info) != ESP_OK)
#else
    if (esp_now_add_peer((u8 *)mac_addr, ESP_NOW_ROLE_COMBO, channel, NULL, 0) != 0)
#endif
    {
        LOG_ADD(" !FAILED!");
    }
    LOG_ADD("] ");
#ifdef ARDUINO_ARCH_ESP8266
    ESP.wdtFeed();
#endif
}

static void FAST_CODE_2 esp_now_recv_cb(uint8_t * mac_addr, uint8_t * data, uint8_t const data_len)
{
    static MSP esp_now_msp_rx;
    uint8_t iter;
    bool const peer_exists = esp_now_is_peer_exist(mac_addr);

    if (!data_len)
        return;

#define ESP_NOW_RX 1
#if ESP_NOW_RX
    String temp = "ESPNOW RX ";
    temp += mac_addr_print(mac_addr);

    if (data[0] == 'E' && data[1] == 'S') {
        // Chorus32 command...
        temp += " - Chorus32 EXT node:";
        temp += (char)data[2];
        temp += ", cmd:";
        temp += (char)data[3];
        websocket_send_txt(temp);
#if UART_DEBUG_EN
        Serial.printf("%s\r\n", temp.c_str());
#endif
        return;
    } else if (data[0] == 'S' && (data[1] == '*' || ('0' <= data[1] && data[1] <= '9'))) {
        // Chorus32 command...
        temp += " - Chorus32 node:";
        temp += (char)data[1];
        temp += ", cmd:";
        temp += (char)data[2];
        websocket_send_txt(temp);
#if UART_DEBUG_EN
        Serial.printf("%s\r\n", temp.c_str());
#endif
        return;
    }
#endif // ESP_NOW_RX

    esp_now_msp_rx.markPacketFree();

    for (iter = 0; iter < data_len; iter++) {
        if (esp_now_msp_rx.processReceivedByte(data[iter])) {
            //  MSP received, check content
            mspPacket_t & packet = esp_now_msp_rx.getPacket();
#if BIND_EN
            if (packet.function == MSP_ESPNOW_BIND_FUNC) {
                bool const valid_key = validate_bind_key(packet.payload);
                if (packet.type == MSP_PACKET_V2_RESPONSE) {
                    if (!peer_exists && valid_key) {
                        esp_now_add_peer((u8 *)mac_addr, ESP_NOW_ROLE_COMBO, packet.payload[MAC_ADDR_LEN], NULL, 0);
                    }
                } else if (packet.type == MSP_PACKET_V2_COMMAND) {
                    if (valid_key) {
                        send_binding_message(false, msp_bind_key, mac_addr);
                    }
                }
            } else
#endif // BIND_EN
                if (peer_exists) {
                    if (msp_handler)
                        msp_handler(packet);
                } else {
#if ESP_NOW_RX
                    if (packet.function == MSP_LAP_TIMER) {
                        laptimer_messages_t const * const p_msg = (laptimer_messages_t *)packet.payload;
                        temp += " !! - Laptimer cmd: ";
                        if (p_msg->subcommand == CMD_LAP_TIMER_REGISTER) {
                            temp += "LAP_TIMER_REGISTER";
                            if (packet.type == MSP_PACKET_V2_RESPONSE) {
                                // Laptimer found. Store its MAC address...
                                memcpy(eeprom_storage.laptimer.mac, mac_addr, sizeof(eeprom_storage.laptimer.mac));
                                eeprom_storage.markDirty();
                                add_peer(mac_addr, esp_now_channel);

                                msp_handler(packet);
                                temp += " RESP OK";
                            } else {
                                temp += " COMMAND ignored";
                            }
                        } else {
                            temp += p_msg->subcommand;
                        }
                    } else {
                        temp += " !! unknown MSP func: 0x";
                        temp += String(packet.function, HEX);
                    }
#endif // ESP_NOW_RX
                    /* Handle broadcast messages */
                    if (packet.type == MSP_PACKET_V2_COMMAND /*||
                    packet.type == MSP_PACKET_V2_RESPONSE*/) {
                    }
                }
            /* Done, clear the packet and proceed */
            esp_now_msp_rx.markPacketFree();
        }
    }
#if ESP_NOW_RX
    if (temp.length()) {
        websocket_send_txt(temp);
#if UART_DEBUG_EN
        Serial.printf("%s\r\n", temp.c_str());
#endif
    }
#endif // ESP_NOW_RX
}

#if DEBUG_TX_CALLBACK
static void esp_now_send_cb(uint8_t * mac_addr, u8 status)
{
    String temp = "ESPNOW Sent: ";
    temp += mac_addr_print(mac_addr);
    temp += (status ? " FAIL" : " SUCCESS");
    websocket_send_txt(temp);
}
#endif
#endif // ESP_NOW

void espnow_init(uint32_t const channel, esp_now_msp_rcvd_cb_t const cb)
{
#ifdef ARDUINO_ARCH_ESP8266
    ESP.wdtFeed();
#endif
#if ESP_NOW

    uint8_t my_uid[MAC_ADDR_LEN];
    uint8_t iter;

    WiFi.softAPmacAddress(&my_uid[0]);
#if BIND_EN
    memcpy(msp_bind_key, my_uid, MAC_ADDR_LEN);
    msp_bind_key[MAC_ADDR_LEN] = channel;
#endif

    if (esp_now_initialized) {
        if (channel != esp_now_channel) {
            // Already initialized, just update wifi channel
            iter = eeprom_storage.espnow_clients_count;
#ifdef ARDUINO_ARCH_ESP32
            esp_now_peer_info_t peer_info = {.peer_addr = {0},
                                             .lmk = {0},
                                             .channel = (uint8_t)channel,
                                             .ifidx = WIFI_IF_AP,
                                             .encrypt = false,
                                             .priv = NULL};
            // Own peers with same UID
            memcpy(peer_info.peer_addr, my_uid, sizeof(peer_info.peer_addr));
            esp_now_mod_peer(&peer_info);
            // Laptimer
            if (wifi_is_mac_valid(&eeprom_storage.laptimer)) {
                memcpy(peer_info.peer_addr, eeprom_storage.laptimer.mac, sizeof(peer_info.peer_addr));
                if (ESP_OK != esp_now_mod_peer(&peer_info)) {
                    esp_now_add_peer(&peer_info);
                }
            }
            while (iter--) {
                memcpy(peer_info.peer_addr, eeprom_storage.espnow_clients[iter].mac_addr, sizeof(peer_info.peer_addr));
                esp_now_mod_peer(&peer_info);
            }
#else
            esp_now_set_peer_channel(my_uid, channel);
            while (iter--) {
                esp_now_set_peer_channel(eeprom_storage.espnow_clients[iter].mac_addr, channel);
            }
#endif
        }
        esp_now_channel = channel;
        return;
    }

    esp_now_initialized = true;
    msp_handler = cb;
    esp_now_channel = channel;

    LOG_SET("ESP NOW init: ");

    if (eeprom_storage.espnow_initialized != LOGGER_ESPNOW_INIT_KEY) {
        eeprom_storage.espnow_clients_count = 0;
        eeprom_storage.espnow_initialized = LOGGER_ESPNOW_INIT_KEY;
        // init storage
        memset(eeprom_storage.espnow_clients, 0, sizeof(eeprom_storage.espnow_clients));
        eeprom_storage.markDirty();
    }

    if (esp_now_init() != 0) {
        LOG_ADD("esp_now_init() failed!");
        return;
    }
#ifdef ARDUINO_ARCH_ESP8266
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
#endif
#if DEBUG_TX_CALLBACK
    esp_now_register_send_cb(esp_now_send_cb);
#endif
    esp_now_register_recv_cb((esp_now_recv_cb_t)esp_now_recv_cb);

    add_peer(my_uid, channel);
    // Laptimer
    if (wifi_is_mac_valid(&eeprom_storage.laptimer)) {
        add_peer(eeprom_storage.laptimer.mac, channel);
    }
    for (iter = 0; iter < eeprom_storage.espnow_clients_count; iter++) {
        add_peer(eeprom_storage.espnow_clients[iter].mac_addr, channel);
    }

    LOG_ADD(" - DONE!");
#else
    (void)channel;
    (void)cb;
#endif // ESP_NOW
}

uint8_t espnow_channel(void)
{
#if ESP_NOW
    return esp_now_channel;
#else
    return 0;
#endif
}

void espnow_update_clients(uint8_t const * const data, uint8_t const len, int const wsnum)
{
#if ESP_NOW
    espnow_clients_t const * const client = (espnow_clients_t *)data;
    uint8_t const count_new = (len / MAC_ADDR_LEN);
    uint8_t iter;

    LOG_SET("ESP NOW update: ");

    if (!data) {
        // No valid data or not initilized yet
        LOG_ADD(" - FAILED!");
        return;
    }

    // Remove existing peers
    iter = eeprom_storage.espnow_clients_count;
    while (iter--) {
        esp_now_del_peer(eeprom_storage.espnow_clients[iter].mac_addr);
    }
    // Add new peers
    eeprom_storage.espnow_initialized = LOGGER_ESPNOW_INIT_KEY;
    eeprom_storage.espnow_clients_count = count_new;
    for (iter = 0; iter < count_new; iter++) {
        memcpy(eeprom_storage.espnow_clients[iter].mac_addr, client[iter].mac_addr, MAC_ADDR_LEN);
        add_peer(client[iter].mac_addr, esp_now_channel);
    }
    LOG_ADD(" - DONE!");
    eeprom_storage.markDirty();
#else
    espnow_init_info = "ESP NOW is not configured!";
    (void)data;
    (void)len;
    (void)wsnum;
#endif // ESP_NOW
}

String & espnow_get_info(void)
{
    return espnow_init_info;
}

void FAST_CODE_2 espnow_send_msp(mspPacket_t & msp)
{
#if ESP_NOW
    size_t const len = MSP::bufferPacket(msp_tx_buffer, &msp);
    if (len) {
        esp_now_send(NULL, msp_tx_buffer, len);
    }
#else
    (void)msp;
#endif // ESP_NOW
}

void espnow_vtxset_send(uint16_t const freq, int8_t const power, bool const pitmode)
{
#if ESP_NOW
    uint8_t size = 2; // Default is only freq
    uint8_t command[4] = {(uint8_t)freq, (uint8_t)(freq >> 8)};
    if (0 <= power) {
        command[2] = power;
        command[3] = pitmode;
        size = 4;
    }
    size_t const len = MSP::bufferPacket(msp_tx_buffer, MSP_PACKET_V2_COMMAND, MSP_VTX_SET_CONFIG,
                                         (uint8_t)eeprom_storage.laptimer_config.index, size, command);
    if (len) {
        esp_now_send(NULL, msp_tx_buffer, len);
    }
#endif
}

void espnow_send_update_channel(uint8_t const channel)
{
#if ESP_NOW
    struct espnow_update update = {.channel = channel};

    size_t const len = MSP::bufferPacket(msp_tx_buffer, MSP_PACKET_V1_ELRS, ELRS_INT_MSP_ESPNOW_UPDATE, MSP_ELRS_INT,
                                         sizeof(update), (uint8_t *)&update);
    if (len) {
        esp_now_send(NULL, msp_tx_buffer, len);
    }
#else
    (void)channel;
#endif
}

#if ESP_NOW
static void FAST_CODE_2 espnow_laptimer_send(size_t const len, uint8_t const * buffer)
{
    if (!wifi_is_mac_valid(&eeprom_storage.laptimer)) {
        return;
    }
    size_t const size = MSP::bufferPacket(msp_tx_buffer, MSP_PACKET_V2_COMMAND, MSP_LAP_TIMER, 0, len, buffer);
    if (size)
        esp_now_send(eeprom_storage.laptimer.mac, msp_tx_buffer, size);
}
#endif

void FAST_CODE_2 espnow_laptimer_register_send(void)
{
#if ESP_NOW
    laptimer_register_req_t command = {.subcommand = CMD_LAP_TIMER_REGISTER, .pilot = {0}};
    size_t const name_len = strlen(eeprom_storage.laptimer_config.pilot_name);
    if (!name_len)
        return;
    strcpy(command.pilot, eeprom_storage.laptimer_config.pilot_name);
    espnow_laptimer_send((sizeof(command.subcommand) + name_len + 1), (uint8_t *)&command);
#endif
}

void FAST_CODE_2 espnow_laptimer_start_send(uint16_t const node_id)
{
#if ESP_NOW
    laptimer_start_t start = {.subcommand = CMD_LAP_TIMER_START, .node_index = node_id, .race_id = 0};
    espnow_laptimer_send(sizeof(start), (uint8_t *)&start);
#else
    (void)node_id;
#endif
}

void FAST_CODE_2 espnow_laptimer_stop_send(uint16_t const node_id)
{
#if ESP_NOW
    laptimer_stop_t stop = {.subcommand = CMD_LAP_TIMER_STOP, .node_index = node_id, .race_id = 0};
    espnow_laptimer_send(sizeof(stop), (uint8_t *)&stop);
#else
    (void)node_id;
#endif
}
