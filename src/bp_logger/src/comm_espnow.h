#pragma once

#include <Arduino.h>
#include "msp.h"

#ifndef ESP_NOW
#define ESP_NOW 0
#endif

struct espnow_update {
    uint8_t channel;
};

typedef int (*esp_now_msp_rcvd_cb_t)(mspPacket_t & msp_pkt);

void espnow_init(uint32_t channel, esp_now_msp_rcvd_cb_t cb);
uint8_t espnow_channel(void);
void espnow_update_clients(uint8_t const * const data, uint8_t len, int wsnum = -1);
String & espnow_get_info(void);
void espnow_send_msp(mspPacket_t & msp);
void espnow_send_update_channel(uint8_t const channel);

void espnow_vtxset_send(uint16_t const freq, int8_t const power = -1, bool const pitmode = false);

void espnow_laptimer_register_send(void);
void espnow_laptimer_start_send(uint16_t node_id);
void espnow_laptimer_stop_send(uint16_t node_id);
