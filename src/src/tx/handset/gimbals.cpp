#include "gimbals.h"
#include "platform.h"

uint8_t gimbals_adjust_min(uint16_t val, uint8_t idx)
{
    if (idx < GIMBAL_IDX_MAX)
        pl_config.gimbals[idx].low = val;
    return 0;
}
uint8_t gimbals_adjust_mid(uint16_t val, uint8_t idx)
{
    if (idx < GIMBAL_IDX_MAX)
        pl_config.gimbals[idx].mid = val;
    return 0;
}
uint8_t gimbals_adjust_max(uint16_t val, uint8_t idx)
{
    if (idx < GIMBAL_IDX_MAX)
        pl_config.gimbals[idx].high = val;
    return 0;
}
