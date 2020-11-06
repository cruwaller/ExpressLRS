#include "targets.h"
#include "debug_elrs.h"
#include "common.h"
#include "ESP8266_WebUpdate.h"
#include "gpio.h"

#include <Arduino.h>

#define WEB_UPDATE_LED_FLASH_INTERVAL 25

uint32_t webUpdateLedFlashIntervalNext = 0;

#if (GPIO_PIN_LED != UNDEF_PIN)
struct gpio_out led_pin; // Invert led
#endif

void ICACHE_RAM_ATTR Printf::_putchar(char character)
{
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(character);
#else
    (void)character;
#endif
}

void beginWebsever(int state)
{
    if (state != STATE_disconnected)
        return;

    forced_stop();

    BeginWebUpdate();
    write_u32(&connectionState, (uint32_t)STATE_fw_upgrade);
    webUpdateLedFlashIntervalNext = 0;
}

#if (GPIO_PIN_BUTTON != UNDEF_PIN)
#include "ClickButton.h"

/* Button is inverted */
ClickButton clickButton(GPIO_PIN_BUTTON, true, 40,  0,  1000);

void button_handle(int state)
{
    uint32_t ms = millis();
    clickButton.update(ms);
    if (clickButton.clicks <= -(BUTTON_RESET_INTERVAL_RX / 1000)) {
        ESP.restart();
    } else if (clickButton.clicks <= -(WEB_UPDATE_PRESS_INTERVAL / 1000)) {
        beginWebsever(state);
    }
}

#endif // GPIO_PIN_BUTTON

void platform_setup(void)
{
    /* Force WIFI off until it is realy needed */
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();

#if (GPIO_PIN_LED != UNDEF_PIN)
    led_pin = gpio_out_setup(GPIO_PIN_LED, 1);
#endif
}

void platform_loop(int state)
{
    uint32_t now = millis();
    if (state == STATE_fw_upgrade)
    {
        HandleWebUpdate();
        if (WEB_UPDATE_LED_FLASH_INTERVAL < (now - webUpdateLedFlashIntervalNext))
        {
#if (GPIO_PIN_LED != UNDEF_PIN)
            // toggle led
            gpio_out_toggle_noirq(led_pin);
#endif
            webUpdateLedFlashIntervalNext = now;
        }
    }
    else
    {
#if (GPIO_PIN_BUTTON != UNDEF_PIN)
        button_handle(state);
#endif
    }
}

void platform_connection_state(int state)
{
#ifdef AUTO_WIFI_ON_BOOT
    if (state == STATE_search_iteration_done && millis() < 30000)
    {
        /* state is disconnect at this point and update can be started */
        beginWebsever(STATE_disconnected);
    }
#endif /* AUTO_WIFI_ON_BOOT */
}

void platform_set_led(bool state)
{
#if (GPIO_PIN_LED != UNDEF_PIN)
    gpio_out_write(led_pin, !state);
#endif
}

void platform_restart(void)
{
    ESP.restart();
}

void platform_wd_feed(void)
{
}
