#pragma once

#include <stdint.h>

#define TX_NUM_ANALOGS  4
#define TX_NUM_MIXER    16 // 4 analogs (TX_NUM_ANALOGS) + 12 AUX channels

enum {
    GIMBAL_IDX_L1 = 0,  // L1
    GIMBAL_IDX_L2,      // L2
    GIMBAL_IDX_R1,      // R1
    GIMBAL_IDX_R2,      // R2
    GIMBAL_IDX_MAX
};

enum {
    GIMBAL_CALIB_L_V = 0,
    GIMBAL_CALIB_L_H,
    GIMBAL_CALIB_R_V,
    GIMBAL_CALIB_R_H,
    GIMBAL_CALIB_MAX = 0x0F,

    GIMBAL_CALIB_LOW  = 0x10,
    GIMBAL_CALIB_MID  = 0x20,
    GIMBAL_CALIB_HIGH = 0x40,
};

struct gimbal_limit {
    uint16_t low;
    uint16_t mid;
    uint16_t high;
};

struct mixer {
    uint8_t index;
    uint8_t inv;
    uint8_t scale;
};

enum {
    ELRS_CMD_GET = 0,
    ELRS_CMD_RATE,
    ELRS_CMD_TLM,
    ELRS_CMD_POWER,
    ELRS_CMD_DOMAIN, // RF Freq
    ELRS_CMD_WIFI_START,
    ELRS_CMD_RF_POWER_TEST,
};
