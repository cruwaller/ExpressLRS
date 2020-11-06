#ifndef __PLATFORM_H_
#define __PLATFORM_H_

#include "gpio.h"
#include <stdint.h>
#include <esp_attr.h>
#include <Stream.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define likely(x) (x)
#define unlikely(x) (x)

#if !defined(ICACHE_RAM_ATTR)
#define ICACHE_RAM_ATTR IRAM_ATTR
#endif // !defined(ICACHE_RAM_ATTR)

#define DRAM_FORCE_ATTR DRAM_ATTR

#define _DISABLE_IRQ()
#define _ENABLE_IRQ()
#define _SAVE_IRQ() 0; xSemaphoreTake(irqMutex, portMAX_DELAY);
#define _RESTORE_IRQ(_x) (void)_x; xSemaphoreGive(irqMutex);

extern SemaphoreHandle_t irqMutex;

class DebugSerial: public Stream
{
public:
    void begin(uint32_t) {}

    int available(void);
    int availableForWrite(void);
    int peek(void);
    int read(void);
    void flush(void);
    size_t write(uint8_t);
    size_t write(const uint8_t *buffer, size_t size);

    inline size_t write(const char * s)
    {
        return write((uint8_t*) s, strlen(s));
    }
    inline size_t write(unsigned long n)
    {
        return write((uint8_t) n);
    }
    inline size_t write(long n)
    {
        return write((uint8_t) n);
    }
    inline size_t write(unsigned int n)
    {
        return write((uint8_t) n);
    }
    inline size_t write(int n)
    {
        return write((uint8_t) n);
    }

private:
};

#if WIFI_LOGGER && !WIFI_UPDATER
extern DebugSerial wifi_logger_serial;
#endif

#ifdef ESP_NOW
#ifndef ESP_NOW_PEERS
#undef ESP_NOW
#endif // ESP_NOW_PEERS
#endif // ESP_NOW

#endif /* __PLATFORM_H_ */
