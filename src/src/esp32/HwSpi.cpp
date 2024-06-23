#include "platform.h"
#include "helpers.h"
#include <Arduino.h>
#include <SPI.h>

//#define USE_ARDUINO_SPI 1

SPIClass DRAM_ATTR SpiBus;

#if !USE_ARDUINO_SPI
#include "soc/rtc_wdt.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"

#if !CONFIG_DISABLE_HAL_LOCKS
#define SPI_MUTEX_LOCK()    do {} while (xSemaphoreTake(spi->lock, portMAX_DELAY) != pdPASS)
#define SPI_MUTEX_UNLOCK()  xSemaphoreGive(spi->lock)
#else
#define SPI_MUTEX_LOCK()
#define SPI_MUTEX_UNLOCK()
#endif

struct spi_struct_t {
    spi_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
};

constexpr uint32_t dev_buff_size = MEMBER_SIZEOF(spi_dev_t, data_buf);


static void FORCED_INLINE
memcpy_isr(uint8_t* __restrict__ dst, const uint8_t* __restrict__ src, size_t length)
{
    while (length--) {
        *dst++ = *src++;
    }
}


static void IRAM_ATTR
__spiTransferBytes(spi_dev_t * const dev, const uint8_t * const out, uint8_t * const in, uint32_t const bytes)
{
    int i;
    uint32_t const words = (bytes + 3) / 4; // round to words, max 16
    uint32_t wordsBuf[words];

    memcpy_isr((uint8_t*)wordsBuf, out, bytes); // copy data to buffer

    dev->mosi_dlen.usr_mosi_dbitlen = ((bytes * 8) - 1);
    dev->miso_dlen.usr_miso_dbitlen = ((bytes * 8) - 1);

    for (i = 0; i < words; i++) {
        dev->data_buf[i] = wordsBuf[i]; // copy buffer to spi fifo
    }

    dev->cmd.usr = 1;

    while (dev->cmd.usr); // wait transfer ready

    if (in) {
        for (i = 0; i < words; i++) {
            wordsBuf[i] = dev->data_buf[i]; // copy spi fifo to buffer
        }
        memcpy_isr(in, (uint8_t*)wordsBuf, bytes); // copy buffer to output
    }
}


static void IRAM_ATTR
__spiTransfer(struct spi_struct_t * const spi, const uint8_t * out, uint8_t * in, uint32_t size)
{
    if (!spi || !spi->dev) {
        return;
    }
    //rtc_wdt_feed();
    //SPI_MUTEX_LOCK();
    taskDISABLE_INTERRUPTS();
#if 1
    while (size) {
        if (dev_buff_size < size) {
            __spiTransferBytes(spi->dev, out, in, dev_buff_size);
            size -= dev_buff_size;
            out += dev_buff_size;
            if (in) {
                in += dev_buff_size;
            }
        } else {
            __spiTransferBytes(spi->dev, out, in, size);
            size = 0;
        }
    }
#else
    spi->dev->mosi_dlen.usr_mosi_dbitlen = 7;
    spi->dev->miso_dlen.usr_miso_dbitlen = 7;
    while (size--) {
        spi->dev->data_buf[0] = *out++;
        spi->dev->cmd.usr = 1;
        while (spi->dev->cmd.usr);
        if (in)
            *in++ = spi->dev->data_buf[0] & 0xFF;
    }
#endif
    taskENABLE_INTERRUPTS();
    //SPI_MUTEX_UNLOCK();
}
#endif // !USE_ARDUINO_SPI


struct spi_config IRAM_ATTR
spi_setup(uint32_t const speed, int const sck, int const miso, int const mosi, uint8_t const mode)
{
    SpiBus.begin(sck, miso, mosi, -1);
    SpiBus.setDataMode(mode);
    SpiBus.setFrequency(speed);
    SpiBus.setHwCs(false);
#if USE_ARDUINO_SPI
    return {.spi = &SpiBus};
#else
    return {.spi = SpiBus.bus()};
#endif
}

void IRAM_ATTR
spi_prepare(struct spi_config const config)
{
    (void)config;
}

void IRAM_ATTR
spi_transfer(struct spi_config const config, uint8_t const receive_data,
             uint8_t const len, uint8_t *data)
{
#if USE_ARDUINO_SPI
    SPIClass * spi = (SPIClass*)config.spi;
    taskDISABLE_INTERRUPTS();
    if (receive_data)
        spi->transfer(data, len);
    else
        spi->writeBytes(data, len);
    taskENABLE_INTERRUPTS();
#else
    __spiTransfer((struct spi_struct_t *)config.spi, data, (receive_data ? data : NULL), len);
#endif
}
