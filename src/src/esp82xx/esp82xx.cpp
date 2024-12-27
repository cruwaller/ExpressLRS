#include "targets.h"
#include "debug_elrs.h"
#include "common.h"
#include "ESP8266_WebUpdate.h"
#include "gpio.h"

#include <Arduino.h>

#if TARGET_ESP8285_TX
#include <EEPROM.h>
#endif

#define WEB_UPDATE_LED_FLASH_INTERVAL 50

uint32_t webUpdateLedFlashIntervalNext = 0;

#if (GPIO_PIN_LED_RGB != UNDEF_PIN)
#include <Adafruit_NeoPixel.h>

static Adafruit_NeoPixel strip;
static constexpr uint16_t PixelCount = 2;
#ifdef WS2812_IS_GRB
#define PIXEL_FORMAT    (NEO_GRB + NEO_KHZ800)
#else
#define PIXEL_FORMAT    (NEO_RGB + NEO_KHZ800)
#endif

void IRAM_ATTR ws2812_set_color_u32(uint32_t const color)
{
    strip.fill(strip.Color(color));
    strip.Show();
}
#elif (GPIO_PIN_LED != UNDEF_PIN)
struct gpio_out led_pin; // Invert led
#endif

void IRAM_ATTR Printf::_putchar(char character)
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

#if !TARGET_ESP8285_TX
    forced_stop();
#endif

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

/******************* CONFIG *********************/
#if TARGET_ESP8285_TX
int8_t platform_config_load(struct platform_config &config)
{
#if STORE_TO_FLASH
    struct platform_config temp;

    EEPROM.get(0, temp);

    if (temp.key == ELRS_EEPROM_KEY) {
        /* load ok, copy values */
        memcpy(&config, &temp, sizeof(temp));
        return 0;
    } else if ((temp.key & ELRS_EEPROM_KEY_MASK) == ELRS_EEPROM_KEY_BASE) {
        platform_config_migrate(&temp, config);
    }
    return -1;
#else
    config.key = ELRS_EEPROM_KEY;
    return 0;
#endif
}

int8_t platform_config_save(struct platform_config &config)
{
    int8_t retval;
    if (config.key != ELRS_EEPROM_KEY)
        return -1;
#if STORE_TO_FLASH
    EEPROM.put(0, config);
    retval = EEPROM.commit() ? 0 : -1;
    return retval;
#else
    return 0;
#endif
}
#endif

/******************* SETUP *********************/
void platform_setup(void)
{
    /* Force WIFI off until it is realy needed */
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();

#if (GPIO_PIN_LED_RGB != UNDEF_PIN)
    strip.setPin(GPIO_PIN_LED_RGB);
    strip.updateType(PIXEL_FORMAT);
    strip.begin();
    strip.updateLength(PixelCount);
    strip.setBrightness(255);
    strip.fill();
    strip.show();
#elif (GPIO_PIN_LED != UNDEF_PIN)
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

#if TARGET_ESP8285_TX
void platform_mode_notify(uint8_t mode)
{
    (void)mode;
}

void platform_wifi_start(void)
{
    beginWebsever(STATE_disconnected);
}
#endif

static uint8_t loop_cnt;
void platform_connection_state(int state)
{
#ifdef AUTO_WIFI_ON_BOOT
    if (state == STATE_search_iteration_done && 2 <= ++loop_cnt) {
#if (GPIO_PIN_LED_RGB != UNDEF_PIN)
        ws2812_set_color_u32(0xe803fc /*purple*/);
#endif
        /* state is disconnect at this point and update can be started */
        beginWebsever(STATE_disconnected);
    } else
#endif /* AUTO_WIFI_ON_BOOT */
    {
#if (GPIO_PIN_LED_RGB != UNDEF_PIN)
        ws2812_set_color_u32(((state == STATE_connected) ? 0xFF00 : 0x00FF));
#endif
    }
}

void platform_set_led(uint8_t state)
{
#if (GPIO_PIN_LED_RGB != UNDEF_PIN)
    ws2812_set_color_u32((state ? 0xFF0000 /*blue*/ : 0xFF /*red*/));
#elif (GPIO_PIN_LED != UNDEF_PIN)
    gpio_out_write(led_pin, state);
#else
    (void)state;
#endif
}

void platform_restart(void)
{
    ESP.restart();
}

void platform_reboot_into_bootloader(const uint8_t * info)
{
    if (validate_bl_indentifier(info) < 0)
        return;
#if !TARGET_ESP8285_TX
    forced_stop();
    delay(100);
#endif
    ESP.rebootIntoUartDownloadMode();
}

void platform_wd_feed(void)
{
}

// Called from core's user_rf_pre_init() function (which is called by SDK) before setup()
RF_PRE_INIT()
{
    // Set whether the chip will do RF calibration or not when power up.
    // I believe the Arduino core fakes this (byte 114 of phy_init_data.bin)
    // to be 1, but the TX power calibration can pull over 300mA which can
    // lock up receivers built with a underspeced LDO (such as the EP2 "SDG")
    // Option 2 is just VDD33 measurement
    #if defined(RF_CAL_MODE)
    system_phy_set_powerup_option(RF_CAL_MODE);
    #else
    system_phy_set_powerup_option(2);
    #endif
}
