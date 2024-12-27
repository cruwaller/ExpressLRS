#pragma once

#include "platform.h"
#include "rc_channels.h"
#include "msp.h"
#include <stdint.h>
#include <stdbool.h>


//// CONSTANTS ////
#ifndef TLM_REPORT_INTERVAL
#define TLM_REPORT_INTERVAL               300u
#endif

enum {
    TLM_UPDATES_NA = 0,
    TLM_UPDATES_LNK_STATS = 1 << 0,
    TLM_UPDATES_BATTERY = 1 << 1,
    TLM_UPDATES_GPS = 1 << 2,
    TLM_UPDATES_DEV_INFO = 1 << 3,
};

//////////// TELEMETRY /////////
enum tlm_msp_state {
    TLM_MSP_STATE_FREE,
    TLM_MSP_STATE_SEND,
    TLM_MSP_STATE_WAIT_RESP,
};

extern mspPacket_t msp_packet_tx;
extern mspPacket_t msp_packet_rx;
extern uint8_t tlm_msp_send, tlm_msp_rcvd;
extern uint32_t tlm_msp_sent_ms;

extern LinkStats_t LinkStatistics;
extern GpsOta_t GpsTlm;
extern DeviceInfo_t DevInfo;
extern uint32_t tlm_updated;


void tx_common_init_globals(void);
void tx_common_init(void);
#if defined(TX_TLM_WHEN_DISARMED)
void tx_common_armed_state(uint8_t armed);
#else
#define tx_common_armed_state(...) // ignore
#endif
void tx_common_handle_rx_buffer(void);
bool tx_common_has_telemetry(void);
bool tx_common_check_connection(void);
void tx_common_handle_ctrl_serial(void);
void tx_common_update_link_stats(void);

bool tx_common_tlm_rx_handle(uint32_t const now);

int8_t SettingsCommandHandle(uint8_t const *in, uint8_t *out,
                             uint8_t inlen, uint8_t &outlen);

void hw_timer_init(void);
void hw_timer_stop(void);

int8_t tx_handle_msp_input(mspPacket_t &packet);
void tx_handle_set_link_rate(uint32_t interval);
