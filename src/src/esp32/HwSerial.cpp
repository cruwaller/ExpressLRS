#include "HwSerial.h"
#include "targets.h"
#include "freertos/semphr.h"
#include "rom/uart.h"
#include "soc/uart_reg.h"
#include "soc/uart_struct.h"
#include "driver/uart.h"

#ifndef CRSF_SERIAL_NBR
#define CRSF_SERIAL_NBR 1
#elif (CRSF_SERIAL_NBR > 2)
#error "Not supported serial!"
#endif

#if CRSF_SERIAL_NBR == 0
#define UART_RXD_IDX (U0RXD_IN_IDX)
#define UART_TXD_IDX (U0TXD_OUT_IDX)
#elif CRSF_SERIAL_NBR == 1
#define UART_RXD_IDX (U1RXD_IN_IDX)
#define UART_TXD_IDX (U1TXD_OUT_IDX)
#elif CRSF_SERIAL_NBR == 2
#define UART_RXD_IDX (U2RXD_IN_IDX)
#define UART_TXD_IDX (U2TXD_OUT_IDX)
#endif

//#define DBG_PIN_UART_TX 15

/********************************************************************************
 *                                   PUBLIC
 ********************************************************************************/
HwSerial CrsfSerial(CRSF_SERIAL_NBR, -1);

HwSerial::HwSerial(int uart_nr, int32_t pin) : HardwareSerial(uart_nr)
{
    (void)pin;
}

void HwSerial::Begin(uint32_t baud, uint32_t config)
{
#ifdef DBG_PIN_UART_TX
    pinMode(DBG_PIN_UART_TX, OUTPUT);
    digitalWrite(DBG_PIN_UART_TX, 0);
#endif
    HardwareSerial::begin(baud, config, GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX, true);
    enable_receiver();
}

void ICACHE_RAM_ATTR HwSerial::enable_receiver(void)
{
#if 1
    // flush cleans RX buffer as well!
    HardwareSerial::flush(); // wait until write ends
#else
    // Wait until TX is ready
    uart_dev_t * dev = (uart_dev_t *)_uart;
    while(dev->status.txfifo_cnt || dev->status.st_utx_out);
#endif
    /* Detach TX pin */
    gpio_matrix_out((gpio_num_t)-1, UART_TXD_IDX, true, false);
    /* Attach RX pin */
    gpio_set_direction((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, GPIO_MODE_INPUT);
    gpio_matrix_in((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, UART_RXD_IDX, true);
    //gpio_pulldown_en((gpio_num_t)GPIO_PIN_RCSIGNAL_RX);
    //uart_enable_rx_intr((uart_port_t)CRSF_SERIAL_NBR);
    yield();

#ifdef DBG_PIN_UART_TX
    digitalWrite(DBG_PIN_UART_TX, 0);
#endif
}

void ICACHE_RAM_ATTR HwSerial::enable_transmitter(void)
{
#ifdef DBG_PIN_UART_TX
    digitalWrite(DBG_PIN_UART_TX, 1);
#endif
    delayMicroseconds(20);
    /* Detach RX pin */
    //uart_disable_rx_intr((uart_port_t)CRSF_SERIAL_NBR);
    //gpio_pulldown_dis((gpio_num_t)GPIO_PIN_RCSIGNAL_RX);
    gpio_matrix_in((gpio_num_t)-1, UART_RXD_IDX, false);
    /* Attach TX pin */
    gpio_set_level((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, 0);
    gpio_set_direction((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, GPIO_MODE_OUTPUT);
    gpio_matrix_out((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, UART_TXD_IDX, true, false);
}
