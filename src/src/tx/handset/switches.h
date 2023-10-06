#pragma once

#include <stdint.h>

/* Calc max switches according to target config */
constexpr uint8_t MAX_NUM_SWITCHES = 0
#ifdef SWITCH_CH_1
    +1
#endif
#ifdef SWITCH_CH_2
    +1
#endif
#ifdef SWITCH_CH_3
    +1
#endif
#ifdef SWITCH_CH_4
    +1
#endif
#ifdef SWITCH_CH_5
    +1
#endif
#ifdef SWITCH_CH_6
    +1
#endif
#ifdef SWITCH_CH_7
    +1
#endif
#ifdef SWITCH_CH_8
    +1
#endif
#ifdef SWITCH_CH_9
    +1
#endif
#ifdef SWITCH_CH_10
    +1
#endif
#ifdef SWITCH_CH_11
    +1
#endif
#ifdef SWITCH_CH_12
    +1
#endif
;

void switches_init(void);
void switches_collect(uint16_t * const out);
uint8_t switches_get_available(void);
