#include "targets.h"
#include "debug_elrs.h"
#include "common.h"
#include <Arduino.h>
//#include "LED.h"
#include <EEPROM.h>
#include "wifi_logger.h"
#include "printf.h"
#include "gpio.h"

//#include <soc/soc.h>
//#include <soc/rtc_cntl_reg.h>
#include <esp_task_wdt.h>

#include <soc/rtc_wdt.h>

#include <soc/timer_group_struct.h>
#include <soc/timer_group_reg.h>


SemaphoreHandle_t DRAM_ATTR irqMutex;

#if (GPIO_PIN_LED_RGB != UNDEF_PIN)
#include <NeoPixelBus.h>
static constexpr uint16_t PixelCount = 2;
#ifdef WS2812_IS_GRB
static NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, GPIO_PIN_LED_RGB);
#else
static NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> strip(PixelCount, GPIO_PIN_LED_RGB);
#endif

void IRAM_ATTR ws2812_set_color_u32(uint32_t const color)
{
    strip.ClearTo(RgbColor(HtmlColor(color)));
    strip.Show();
}
#elif (GPIO_PIN_LED != UNDEF_PIN)
struct gpio_out led_out;
#endif


void IRAM_ATTR Printf::_putchar(char character)
{
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(character);
#else
    (void)character;
#endif
}

void feedTheDog(void)
{
    // feed dog 0
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE; // write enable
    TIMERG0.wdt_feed = 1;                       // feed dog
    TIMERG0.wdt_wprotect = 0;                   // write protect
    // feed dog 1
    TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE; // write enable
    TIMERG1.wdt_feed = 1;                       // feed dog
    TIMERG1.wdt_wprotect = 0;                   // write protect
}

static void configure_wdt(void)
{
#if 0
    rtc_wdt_protect_off();
    rtc_wdt_disable();
#endif
#if 1
    // Set WDT timeout to 250ms
    rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
    rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
    rtc_wdt_set_time(RTC_WDT_STAGE0, 250);
#endif

    /* Disable watchdogs to precent unwanted resets */
    //disableCore0WDT();
    //disableCore1WDT();
#if 0
    TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE; // Unlock timer config.
    TIMERG1.wdt_feed = 1; // Reset feed count.
    TIMERG1.wdt_config0.en = 0; // Disable timer.
    TIMERG1.wdt_wprotect = 0; // Lock timer config.

    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_config0.en = 0;
    TIMERG0.wdt_wprotect = 0;
#endif
    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
}

void initVariant(void)
{
    //configure_wdt();
}

/******************* CONFIG *********************/
int8_t platform_config_load(struct platform_config &config)
{
#if STORE_TO_FLASH
    struct platform_config temp;

    //taskDISABLE_INTERRUPTS();
    EEPROM.get(0, temp);
    //taskENABLE_INTERRUPTS();

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
    //taskDISABLE_INTERRUPTS();
    retval = EEPROM.commit() ? 0 : -1;
    //taskENABLE_INTERRUPTS();
    return retval;
#else
    return 0;
#endif
}


/******************* SETUP *********************/
void platform_setup(void)
{
    configure_wdt();

    // Set higher task priority
    vTaskPrioritySet(NULL, 10);

    irqMutex = xSemaphoreCreateMutex();

#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.begin(115200);
#endif // DEBUG_SERIAL

    //Serial.print("EpressLRS on core ");
    //Serial.println(xPortGetCoreID());

    EEPROM.begin(sizeof(struct platform_config));

#if (GPIO_PIN_LED_RGB != UNDEF_PIN)
    strip.Begin();
#elif (GPIO_PIN_LED != UNDEF_PIN)
    led_out = gpio_out_setup(GPIO_PIN_LED, 0);
#endif

#if WIFI_LOGGER || WIFI_UPDATER || ESP_NOW
    wifi_init();
#endif

#if 0
    //strip.Begin();

    uint8_t baseMac[6];
    // Get base mac address
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    // Print base mac address
    // This should be copied to common.h and is used to generate a unique hop sequence, DeviceAddr, and CRC.
    // UID[0..2] are OUI (organisationally unique identifier) and are not ESP32 unique.  Do not use!
    DEBUG_PRINT("uint8_t UID[6] = {");
    DEBUG_PRINT(baseMac[0]);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(baseMac[1]);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(baseMac[2]);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(baseMac[3]);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(baseMac[4]);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(baseMac[5]);
    DEBUG_PRINTLN("};");
#endif
}

void platform_mode_notify(uint8_t mode)
{
    (void)mode;
}

void platform_loop(int state)
{
    (void)state;
}

void platform_connection_state(int state)
{
#if (GPIO_PIN_LED_RGB != UNDEF_PIN)
    ws2812_set_color_u32(((state == STATE_connected) ? 0xFF00 : 0x00FF));
#else
    (void)state;
#endif
}

void platform_set_led(uint8_t state)
{
#if (GPIO_PIN_LED_RGB != UNDEF_PIN)
    ws2812_set_color_u32((state ? 0xFF0000 /*blue*/ : 0xFF /*red*/));
#elif (GPIO_PIN_LED != UNDEF_PIN)
    gpio_out_write(led_out, state);
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
    DEBUG_PRINTF("Jumping to Bootloader... IGNORED\n");
    (void)info;
    //if (validate_bl_indentifier(info) < 0)
    //    return;
    //ESP.rebootIntoUartDownloadMode();
}

void platform_wifi_start(void)
{
#if WIFI_LOGGER || WIFI_UPDATER
    wifi_start();
#endif
}

void platform_wd_feed(void)
{
    //esp_task_wdt_reset(); // make sure the WD is feeded
    feedTheDog();
    yield();
}
