#include "switches.h"
#include "internal.h"
#include "stm32_def.h"
#include "gpio.h"
#include "Arduino.h"
#include "targets.h"
#include "rc_channels.h"

#define CONCAT_helper(x, y) x ## y
#define CONCAT(S, N) CONCAT_helper(S, _##N)

struct switch_init {
    int32_t low;
    int32_t high;
};

struct switch_init switch_gpios[] = {
#ifdef SWITCH_CH_1
    {
#if CONCAT(SWITCH_CH_1, 1)
        .low = CONCAT(SWITCH_CH_1, 1),
#else
        .low = UNDEF_PIN,
#endif
#if CONCAT(SWITCH_CH_1, 2)
        .high = CONCAT(SWITCH_CH_1, 2)
#else
        .high = UNDEF_PIN,
#endif
    },
#endif

#ifdef SWITCH_CH_2
    {
#if CONCAT(SWITCH_CH_2, 1)
        .low = CONCAT(SWITCH_CH_2, 1),
#else
        .low = UNDEF_PIN,
#endif
#if CONCAT(SWITCH_CH_2, 2)
        .high = CONCAT(SWITCH_CH_2, 2)
#else
        .high = UNDEF_PIN,
#endif
    },
#endif

#ifdef SWITCH_CH_3
    {
#if CONCAT(SWITCH_CH_3, 1)
        .low = CONCAT(SWITCH_CH_3, 1),
#else
        .low = UNDEF_PIN,
#endif
#if CONCAT(SWITCH_CH_3, 2)
        .high = CONCAT(SWITCH_CH_3, 2)
#else
        .high = UNDEF_PIN,
#endif
    },
#endif

#ifdef SWITCH_CH_4
    {
#if CONCAT(SWITCH_CH_4, 1)
        .low = CONCAT(SWITCH_CH_4, 1),
#else
        .low = UNDEF_PIN,
#endif
#if CONCAT(SWITCH_CH_4, 2)
        .high = CONCAT(SWITCH_CH_4, 2)
#else
        .high = UNDEF_PIN,
#endif
    },
#endif

#ifdef SWITCH_CH_5
    {
#if CONCAT(SWITCH_CH_5, 1)
        .low = CONCAT(SWITCH_CH_5, 1),
#else
        .low = UNDEF_PIN,
#endif
#if CONCAT(SWITCH_CH_5, 2)
        .high = CONCAT(SWITCH_CH_5, 2)
#else
        .high = UNDEF_PIN,
#endif
    },
#endif

#ifdef SWITCH_CH_6
    {
#if CONCAT(SWITCH_CH_6, 1)
        .low = CONCAT(SWITCH_CH_6, 1),
#else
        .low = UNDEF_PIN,
#endif
#if CONCAT(SWITCH_CH_6, 2)
        .high = CONCAT(SWITCH_CH_6, 2)
#else
        .high = UNDEF_PIN,
#endif
    },
#endif
};

constexpr uint8_t num_switches = ARRAY_SIZE(switch_gpios);

/* Paranoia... make sure the switch count is correct */
static_assert(MAX_NUM_SWITCHES == num_switches,
              "Invalid switch count! Check config...");

struct switch_pins {
    struct gpio_in low;
    struct gpio_in high;
};

static struct switch_pins switches[num_switches];

void switches_init(void)
{
    uint8_t iter;
    for (iter = 0; iter < num_switches; iter++) {
        if (switch_gpios[iter].low != UNDEF_PIN) {
            switches[iter].low =
                gpio_in_setup(switch_gpios[iter].low, 1);
        }
        if (switch_gpios[iter].high != UNDEF_PIN) {
            switches[iter].high =
                gpio_in_setup(switch_gpios[iter].high, 1);
        }
    }
}

void FAST_CODE_1 switches_collect(uint16_t * const out)
{
    uint16_t value;
    uint8_t iter;
    for (iter = 0; iter < num_switches; iter++) {
        bool const low_valid = gpio_in_valid(switches[iter].low);
        bool const high_valid = gpio_in_valid(switches[iter].high);

        if (!low_valid && !high_valid) {
            // no pins defined for this switch
            value = SWITCH_MIN;
            continue;
        }

        if (!low_valid) {
            // Use HIGH pin only
            value = !gpio_in_read(switches[iter].high) ? SWITCH_MAX : SWITCH_MIN;
        } else if (!high_valid) {
            // Use LOW pin only
            value = !gpio_in_read(switches[iter].low) ? SWITCH_MAX : SWITCH_MIN;
        } else {
            if (!gpio_in_read(switches[iter].low)) {
                value = SWITCH_MIN;
            } else if (!gpio_in_read(switches[iter].high)) {
                value = SWITCH_MAX;
            } else {
                value = SWITCH_MID;
            }
        }
        out[iter] = value;
    }
}

uint8_t FAST_CODE_1 switches_get_available(void)
{
    return num_switches;
}
