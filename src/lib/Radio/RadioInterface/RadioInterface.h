#ifndef RADIO_INTERFACE_H_
#define RADIO_INTERFACE_H_

#include "platform.h"
#include "RadioHalSpi.h"
#include "gpio.h"

// default payload size is 8 bytes
#define RX_BUFFER_LEN (8)

enum isr_states
{
    NONE,
    RX_DONE,
    RX_TIMEOUT,
    TX_DONE,
    CRC_ERROR,
    CAD_DETECTED,
    CAD_DONE,
    ISR_RCVD,
};

enum module_types
{
    MODULE_DEFAULT = 0,
    MODULE_R9M_DAC,
    MODULE_LORA1280F27,
    MODULE_E28_2G4M12S,
    MODULE_LORA1276F30,
    MODULE_COUNT,
};

class RadioInterface : public RadioHalSpi
{
public:
    RadioInterface(uint8_t payload_len, uint8_t read = 0, uint8_t write = 0):
            RadioHalSpi(read, write), RX_buffer_size(payload_len) {
        RXdoneCallback1 = RadioInterface::rx_nullCallback;
        TXdoneCallback1 = RadioInterface::tx_nullCallback;
    }
    uint8_t GetModuleType(void) {
        return module_type;
    }

    void SetPins(int rst, int dio1, int dio2, int dio3,
                 int busy, int txpin, int rxpin, int cs);
    virtual int8_t Begin(int sck, int miso, int mosi) = 0;
    virtual void End(void) = 0;
    virtual void Config(uint32_t bw, uint32_t sf, uint32_t cr,
                        uint32_t freq, uint16_t PreambleLength,
                        uint8_t crc = 0) = 0;
    void SetSyncWord(uint8_t syncWord) {
        _syncWord = syncWord;
    };
    virtual void SetOutputPower(int8_t power, uint8_t init=0) = 0;
    virtual void setPPMoffsetReg(int32_t error_hz, uint32_t frf = 0) = 0;
    virtual int32_t GetFrequencyError() = 0;
    virtual int16_t MeasureNoiseFloor(uint32_t num_meas, uint32_t freq) = 0;
    virtual void StopContRX(void) = 0;
    virtual void RXnb(uint32_t freq = 0) = 0;
    virtual void TXnb(const uint8_t *data, uint8_t length, uint32_t freq = 0) = 0;

    inline enum isr_states isr_state_get(void) const {
        return p_state_isr;
    }
    inline void isr_state_set(enum isr_states isr) {
        p_state_isr = isr;
    }

    ////////// Callback Function Pointers //////////
    static void rx_nullCallback(uint8_t *, uint32_t){};
    static void tx_nullCallback(void){};
    void (*RXdoneCallback1)(uint8_t *buff, uint32_t rx_us);
    void (*TXdoneCallback1)(void);

    ////////// Packet Stats //////////
    volatile int16_t LastPacketRSSI;
    volatile int8_t LastPacketSNR;
    const uint8_t RX_buffer_size;

protected:
    void Reset(void);
    void WaitOnBusy() const;
    void TxEnable();
    void RxEnable();
    void TxRxDisable();

    gpio_out _RXen;
    gpio_out _TXen;
    gpio_out _RST;
    gpio_in _DIO1;
    gpio_in _DIO2;
    gpio_in _DIO3;
    gpio_in _BUSY;

    ////////// Config Variables //////////
    volatile uint32_t current_freq;
    volatile int8_t current_power;

    uint8_t module_type;
    uint8_t _syncWord;

private:
    volatile enum isr_states p_state_isr;
};

#endif /* RADIO_INTERFACE_H_ */
