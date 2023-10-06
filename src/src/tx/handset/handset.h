#pragma once

#include "switches.h"
#include "gimbals.h"
#include "rc_channels.h"

typedef union {
    rc_channels_handset_t ota_pkt;
    uint16_t ch[TX_NUM_ANALOGS + MAX_NUM_SWITCHES];
} rc_channels_internal_u;
