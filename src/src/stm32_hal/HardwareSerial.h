#pragma once

#include <stdint.h>
#include <stddef.h>
#include "gpio.h"
#include "irq.h"
#include "platform.h"


#define SERIAL_8N1 0x06

#define UART_BUFF_SIZE 256

#if defined(STM32F7xx)
#ifndef PRINTF_NUM_BLOCKS
#define UART_DMA_TX_BUFF    64
#else
#define UART_DMA_TX_BUFF    PRINTF_NUM_BLOCKS
#endif
#else
#define UART_DMA_TX_BUFF    16
#endif

/* This is to avoid unnecessary data copy */
#define UART_USE_TX_POOL_ONLY 1

// Used by the TX DMA
// Note: make sure the TX is not called ofter for same buffer!
struct tx_pool_s {
    struct tx_pool_s * next;
    uint8_t const * data_ptr;
    uint32_t len;
};


/* Interface kept arduino compatible */
class HardwareSerial
{
public:
    HardwareSerial(uint32_t rx, uint32_t tx, uint8_t dma=0);
    void setTx(uint32_t tx_pin);
    void setRx(uint32_t rx_pin);
    void begin(unsigned long baud, uint8_t mode = SERIAL_8N1);
    void end(void);
    void Pause(void);
    void Continue(void);
    int available(void);
    int read(void);
    void flush(void);
    uint32_t write(const uint8_t *buff, uint32_t len);
    uint32_t write_direct(const uint8_t *buff, uint32_t len);
    void setTimeout(unsigned long) {}

    void hw_enable_receiver(void);
    void hw_enable_transmitter(void);

    uint8_t rx_head;
    uint8_t rx_tail;
    uint8_t rx_buffer[UART_BUFF_SIZE];

#if UART_USE_TX_POOL_ONLY
    uint8_t * tx_buffer_ptr;
    uint32_t tx_buffer_len;
#else
    uint8_t tx_head;
    uint8_t tx_tail;
    uint8_t tx_buffer[UART_BUFF_SIZE];
#endif

    // for DMA TX
    bool tx_pool_get(uint8_t** const p_data, uint32_t * const p_len)
    {
        irqstatus_t flag = irq_save();
        struct tx_pool_s * tail = tx_pool_pop();
        irq_restore(flag);
        if (!tail)
            return false;
        *p_data = (uint8_t*)tail->data_ptr;
        *p_len = tail->len;
        tail->data_ptr = NULL;
        return true;
    }

    void* dma_unit_tx;
    void* dma_unit_rx;
    void* p_usart_tx;
    void* p_usart_rx;
    uint8_t dma_ch_tx;
    uint8_t dma_ch_rx;
    uint8_t usart_tx_idx;
    uint8_t usart_rx_idx;

protected:
    uint32_t rx_pin;
    uint32_t tx_pin;
    struct gpio_out p_duplex_pin;
    uint8_t p_duplex_pin_inv;
    uint8_t inverted;

private:
    uint8_t usart_irq_rx;
    uint8_t usart_irq_tx;
    uint8_t p_use_dma;
    uint8_t dma_irq_tx;
    uint8_t dma_irq_rx;
    uint8_t half_duplex;

    // for DMA TX
    struct tx_pool_s tx_pool[UART_DMA_TX_BUFF];
    size_t tx_pool_in;
    size_t tx_pool_out;

    uint32_t DR_RX;
    uint32_t DR_TX;

    void tx_pool_init(void) {
        tx_pool_out = tx_pool_in = 0;
    }

    bool tx_pool_is_empty(void) {
        irqstatus_t flag = irq_save();
        bool empty = tx_pool_out == tx_pool_in;
        irq_restore(flag);
        return empty;
    }

    bool tx_pool_is_full(void) {
        irqstatus_t flag = irq_save();
        bool full = tx_pool_in == ((tx_pool_out - 1 + UART_DMA_TX_BUFF) % UART_DMA_TX_BUFF);
        irq_restore(flag);
        return full;
    }

    struct tx_pool_s * tx_pool_pop(void) {
        if (tx_pool_is_empty())
            return NULL;
        size_t out = tx_pool_out;
        struct tx_pool_s * p_out = &tx_pool[out];
        tx_pool_out = (out + 1) % UART_DMA_TX_BUFF;
        return p_out;
    }

    void tx_pool_push(const uint8_t * data, uint32_t len)
    {
        if (tx_pool_is_full())
            return;
        size_t in = tx_pool_in;
        tx_pool[in].data_ptr = data;
        tx_pool[in].len = len;
        tx_pool_in = (in + 1) % UART_DMA_TX_BUFF;
    }
};

extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
