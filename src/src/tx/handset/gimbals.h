#pragma once

#include <stdint.h>

#define NUM_ANALOGS   4

#define GIMBALS_ANALOG  0
#define GIMBALS_NV14    1

#ifndef HANDSET_GIMBALS
#define HANDSET_GIMBALS GIMBALS_ANALOG
#endif /* HANDSET_GIMBALS */

// Specific [gimbals_xxxx.cpp]
void gimbals_init(void);
void gimbals_timer_adjust(uint32_t us);
void gimbals_get(uint16_t * const out);
void gimbals_process_loop(void);
uint8_t gimbals_calibrate(uint8_t * data);

// Common [gimbals.cpp]
uint8_t gimbals_adjust_min(uint16_t val, uint8_t idx);
uint8_t gimbals_adjust_mid(uint16_t val, uint8_t idx);
uint8_t gimbals_adjust_max(uint16_t val, uint8_t idx);
