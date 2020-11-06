#ifndef RADIO_HAL_SPI_H_
#define RADIO_HAL_SPI_H_

#include "platform.h"
#include "HwSpi.h"
#include <stdint.h>

class RadioHalSpi {
public:

protected:
    RadioHalSpi(HwSpi &spi, uint32_t read = 0, uint32_t write = 0)
            : spi_bus(spi), p_write(write), p_read(read) {}
    void Begin(uint32_t speed, int sck, int miso, int mosi, int ss);

    uint8_t ICACHE_RAM_ATTR readRegister(uint8_t reg) const;
    void ICACHE_RAM_ATTR writeRegister(uint8_t reg, uint8_t data) const;

    void ICACHE_RAM_ATTR readRegisterBurst(uint8_t reg, uint8_t numBytes, uint8_t *inBytes) const;
    void ICACHE_RAM_ATTR writeRegisterBurst(uint8_t reg, uint8_t *data, uint8_t numBytes) const;

    void ICACHE_RAM_ATTR readRegisterAddr(uint8_t reg, uint16_t addr,
                                          uint8_t *data, uint8_t numBytes) const;
    void ICACHE_RAM_ATTR writeRegisterAddr(uint8_t reg, uint16_t addr,
                                           uint8_t *data, uint8_t numBytes) const;

    void ICACHE_RAM_ATTR readRegisterOffset(uint8_t reg, int offset,
                                            uint8_t *data, uint8_t numBytes) const;
    void ICACHE_RAM_ATTR writeRegisterOffset(uint8_t reg, uint8_t offset,
                                            uint8_t *data, uint8_t numBytes) const;

private:
    HwSpi &spi_bus;
    const uint32_t p_write = 0;
    const uint32_t p_read = 0;
};

#endif /* RADIO_HAL_SPI_H_ */
