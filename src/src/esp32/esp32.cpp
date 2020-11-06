#include "targets.h"
#include "debug_elrs.h"
#include "common.h"
#include <Arduino.h>
//#include "LED.h"
#include <EEPROM.h>
#include "wifi_logger.h"
#include "printf.h"
#include "gpio.h"

#ifdef TARGET_EXPRESSLRS_PCB_TX_V3
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif
#include "esp_task_wdt.h"

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

SemaphoreHandle_t DRAM_ATTR irqMutex;

#if (GPIO_PIN_LED != UNDEF_PIN)
struct gpio_out led_out;
#endif


void ICACHE_RAM_ATTR Printf::_putchar(char character)
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

/******************* CONFIG *********************/
int8_t platform_config_load(struct platform_config &config)
{
#if STORE_TO_FLASH
    struct platform_config temp;

    //taskDISABLE_INTERRUPTS();
    temp.key = EEPROM.readUInt(offsetof(struct platform_config, key));
    temp.mode = EEPROM.readUInt(offsetof(struct platform_config, mode));
    temp.power = EEPROM.readUInt(offsetof(struct platform_config, power));
    temp.tlm = EEPROM.readUInt(offsetof(struct platform_config, tlm));
    //taskENABLE_INTERRUPTS();

    if (temp.key == ELRS_EEPROM_KEY)
    {
        /* load ok, copy values */
        config.key = temp.key;
        config.mode = temp.mode;
        config.power = temp.power;
        config.tlm = temp.tlm;
        return 0;
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
    EEPROM.writeUInt(offsetof(struct platform_config, key), config.key);
    EEPROM.writeUInt(offsetof(struct platform_config, mode), config.mode);
    EEPROM.writeUInt(offsetof(struct platform_config, power), config.power);
    EEPROM.writeUInt(offsetof(struct platform_config, tlm), config.tlm);
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
    disableCore0WDT();
    disableCore1WDT();

    // Set higher task priority
    vTaskPrioritySet(NULL, 10);

    irqMutex = xSemaphoreCreateMutex();

#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.begin(115200);
#endif // DEBUG_SERIAL

    //Serial.print("EpressLRS on core ");
    //Serial.println(xPortGetCoreID());

    EEPROM.begin(sizeof(struct platform_config));

#if (GPIO_PIN_LED != UNDEF_PIN)
    led_out = gpio_out_setup(GPIO_PIN_LED, 0);
#endif

#if WIFI_LOGGER || WIFI_UPDATER || ESP_NOW
    wifi_init();
#endif

#if 0
    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

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
    (void)state;
}

void platform_set_led(bool state)
{
#if (GPIO_PIN_LED != UNDEF_PIN)
    gpio_out_write(led_out, state);
#else
    (void)state;
#endif
}

void platform_restart(void)
{
    ESP.restart();
}

void platform_wifi_start(void)
{
#if WIFI_LOGGER || WIFI_UPDATER
    //platform_radio_force_stop();
    wifi_start();
#endif
}

void platform_wd_feed(void)
{
    //esp_task_wdt_reset(); // make sure the WD is feeded
    feedTheDog();
    yield();
}
