#ifndef _GPIO_H__
#define _GPIO_H__

#include "PinNames.h"
#include <stdint.h> // uint32_t

#define GPIO_PIN_IVALID NC

struct gpio_out
{
    PinName pin;
};
struct gpio_out gpio_out_setup(uint32_t pin, uint32_t val);
void gpio_out_reset(struct gpio_out g, uint32_t val);
void gpio_out_toggle_noirq(struct gpio_out g);
void gpio_out_toggle(struct gpio_out g);
void gpio_out_write(struct gpio_out g, uint32_t val);
static inline uint8_t gpio_out_valid(struct gpio_out g) {
    return (g.pin != GPIO_PIN_IVALID);
}

struct gpio_in
{
    PinName pin;
};
struct gpio_in gpio_in_setup(uint32_t pin, int32_t pull_up);
void gpio_in_reset(struct gpio_in g, int32_t pull_up);
uint8_t gpio_in_read(struct gpio_in g);
static inline uint8_t gpio_in_valid(struct gpio_in g) {
    return (g.pin != GPIO_PIN_IVALID);
}
typedef void (*isr_cb_t)(void);
void gpio_in_isr(struct gpio_in g, isr_cb_t func, uint8_t type);
void gpio_in_isr_remove(struct gpio_in g);

#endif /*_GPIO_H__*/