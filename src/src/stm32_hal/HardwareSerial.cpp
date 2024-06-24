//
// STM32 serial
//

#include "HardwareSerial.h"
#include "internal.h"
#include "irq.h"
#include "debug_elrs.h"
#include "targets.h"
#include "priorities.h"

#if defined(STM32F1xx)
#include <stm32f1xx_ll_usart.h>
#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_dma.h>
#define StatReg         SR
#define RxDataReg       DR
#define TxDataReg       DR
#elif defined(STM32L0xx)
#include <stm32l0xx_ll_usart.h>
#include <stm32l0xx_ll_bus.h>
#include <stm32l0xx_ll_dma.h>
#elif defined(STM32L4xx)
#include <stm32l4xx_ll_usart.h>
#include <stm32l4xx_ll_bus.h>
#include <stm32l4xx_ll_dma.h>
#elif defined(STM32F3xx)
#include <stm32f3xx_ll_usart.h>
#include <stm32f3xx_ll_bus.h>
#include <stm32f3xx_ll_dma.h>
#elif defined(STM32F7xx)
#include <stm32f7xx_ll_usart.h>
#include <stm32f7xx_ll_bus.h>
#include <stm32f7xx_ll_dma.h>
#define LL_DMA_EnableChannel LL_DMA_EnableStream
#define LL_DMA_DisableChannel LL_DMA_DisableStream
#define LL_DMA_IsEnabledChannel LL_DMA_IsEnabledStream
#define LL_DMA_SetChannelPriorityLevel LL_DMA_SetStreamPriorityLevel
#elif defined(STM32G0xx)
#include <stm32g0xx_ll_usart.h>
#include <stm32g0xx_ll_bus.h>
#include <stm32g0xx_ll_dma.h>
#define USART_SR_RXNE       USART_ISR_RXNE_RXFNE
#define USART_SR_TXE        USART_ISR_TXE_TXFNF
#define USART_CR1_RXNEIE    USART_CR1_RXNEIE_RXFNEIE
#define USART_CR1_TXEIE     USART_CR1_TXEIE_TXFNFIE
#define USART3_IRQn         USART3_4_LPUART1_IRQn
#endif
#include <string.h>

#ifndef StatReg
#define StatReg         ISR
#endif
#ifndef RxDataReg
#define RxDataReg       RDR
#endif
#ifndef TxDataReg
#define TxDataReg       TDR
#endif
#ifndef USART_SR_IDLE
#define USART_SR_IDLE   USART_ISR_IDLE
#endif
#ifndef USART_SR_RXNE
#define USART_SR_RXNE   USART_ISR_RXNE
#endif
#ifndef USART_SR_ORE
#define USART_SR_ORE    USART_ISR_ORE
#endif
#ifndef USART_SR_TXE
#define USART_SR_TXE    USART_ISR_TXE
#endif
#ifndef USART_SR_TC
#define USART_SR_TC     USART_ISR_TC
#endif

#define UART_ENABLE_DMA_RX 1
#define UART_ENABLE_DMA_TX 1


#ifdef DEBUG_SERIAL
// dummy putchar
#ifndef PRINTF_NUM_BLOCKS
#define PRINTF_NUM_BLOCKS   8
#endif
#ifndef PRINTF_BUFF_SIZE
#define PRINTF_BUFF_SIZE    64
#endif
char   printf_out[PRINTF_NUM_BLOCKS][PRINTF_BUFF_SIZE];
size_t printf_idx, printf_buff;

void FAST_CODE_2 Printf::_putchar(char const character)
{
    printf_out[printf_buff][printf_idx++] = character;

    /* Send buff out if line end or buffer is full */
    if ((character == '\n') || (PRINTF_BUFF_SIZE <= printf_idx)) {
        DEBUG_SERIAL.write((uint8_t*)printf_out[printf_buff], printf_idx);
        printf_buff = (printf_buff+1) % PRINTF_NUM_BLOCKS;
        printf_idx = 0;
    }
}
#endif //DEBUG_SERIAL

constexpr uint8_t serial_cnt = 0
#ifdef USART1
    + 1
#endif
#ifdef USART2
    + 1
#endif
#ifdef USART3
    + 1
#endif
#ifdef UART4
    + 1
#endif
#ifdef UART5
    + 1
#endif
#ifdef USART6
    + 1
#endif
#ifdef UART7
//    + 1
#endif
#ifdef UART8
//    + 1
#endif
    ;
HardwareSerial * _started_serials[serial_cnt];

enum {
    USE_DMA_NONE    = 0,
    USE_DMA_RX      = UART_ENABLE_DMA_RX << 1,
    USE_DMA_TX      = UART_ENABLE_DMA_TX << 2,
};

static FORCED_INLINE uint32_t
dma_ifcr_mask_get(uint32_t mask, uint8_t dma_ch)
{
#ifdef STM32F7xx
    if (dma_ch < 2) {
        mask <<= (DMA_LIFCR_CFEIF1_Pos * dma_ch);
    } else if (dma_ch < 4) {
        mask <<= (16 + (DMA_LIFCR_CFEIF1_Pos * (dma_ch - 2)));
    } else if (dma_ch < 6) {
        mask <<= (DMA_LIFCR_CFEIF1_Pos * (dma_ch - 4));
    } else {
        mask <<= (16 + (DMA_LIFCR_CFEIF1_Pos * (dma_ch - 6)));
    }
#else
    mask <<= (4 * (dma_ch - 1));
#endif
    return mask;
}

bool FAST_CODE_2 DMA_transmit(HardwareSerial * serial, uint8_t dma_ch)
{
    DMA_TypeDef * dma = (DMA_TypeDef *)serial->dma_unit_tx;
    /* Check if something to send */
    uint32_t len, data;
    if (!serial->tx_pool_get((uint8_t**)&data, &len))
        return false;
    /* Clear all irq flags */
    __IO uint32_t * IFCR;
    uint32_t mask;
#ifdef STM32F7xx
    mask = ((0x1 << DMA_LIFCR_CFEIF1_Pos) - 1);
    IFCR = (dma_ch < 4) ? &dma->LIFCR : &dma->HIFCR;
#else
    mask = 0xF;
    IFCR = &dma->IFCR;
#endif
    WRITE_REG(*IFCR, dma_ifcr_mask_get(mask, dma_ch));
    /* Set source address */
    LL_DMA_SetMemoryAddress(dma, dma_ch, data);
    LL_DMA_SetDataLength(dma, dma_ch, len);
    /* enable tx */
    serial->hw_enable_transmitter();
    /* Start transfer */
    LL_DMA_EnableChannel(dma, dma_ch);
    return true;
}

#if UART_USE_TX_POOL_ONLY
bool FAST_CODE_2 UART_transmit(HardwareSerial * serial)
{
    return serial->tx_pool_get(&serial->tx_buffer_ptr, &serial->tx_buffer_len);
}
#endif

void FAST_CODE_2 USART_IDLE_IRQ_handler(uint32_t index)
{
    if (serial_cnt <= index)
        return;
    HardwareSerial *serial = _started_serials[index];
    if (!serial)
        return;
    __IO USART_TypeDef * uart;

    if (serial->usart_rx_idx == index) {
        uart = (USART_TypeDef *)serial->p_usart_rx;
        uint32_t SR = uart->StatReg, CR1 = uart->CR1;

        /* Check for IDLE line interrupt */
        if ((SR & USART_SR_IDLE) && (CR1 & USART_CR1_IDLEIE)) {
            uint8_t head_pos = sizeof(serial->rx_buffer) -
                LL_DMA_GetDataLength((DMA_TypeDef *)serial->dma_unit_rx, serial->dma_ch_rx);
            serial->rx_head = head_pos;
            //LL_USART_ClearFlag_IDLE((USART_TypeDef *)uart);
            uart->ICR = USART_ICR_IDLECF;
        }
        /* Check for RX data */
        else if ((SR & (USART_SR_RXNE | USART_SR_ORE)) && (CR1 & USART_CR1_RXNEIE)) {
            uint8_t next = serial->rx_head;
            uint8_t data = uart->RxDataReg;
            if ((next + 1) != serial->rx_tail) {
                serial->rx_buffer[next] = data;
                serial->rx_head = next + 1;
            }
        }
    }

    if (serial->usart_tx_idx == index) {
        uart = (USART_TypeDef *)serial->p_usart_tx;
        // Check if TX is enabled and TX Empty IRQ triggered
        if ((uart->StatReg & USART_SR_TXE) && (uart->CR1 & USART_CR1_TXEIE)) {
            //  Check if data available
#if UART_USE_TX_POOL_ONLY
            if (serial->tx_buffer_len) {
                uart->TxDataReg = *serial->tx_buffer_ptr++;
                serial->tx_buffer_len--;
            } else if (!UART_transmit(serial)) {
                serial->hw_enable_receiver();
            }
#else
            if (serial->tx_head <= serial->tx_tail)
                serial->hw_enable_receiver();
            else
                uart->TxDataReg = serial->tx_buffer[serial->tx_tail++];
#endif
        }
    }
}

void FAST_CODE_2 USARTx_DMA_handler(uint32_t index)
{
    if (serial_cnt <= index)
        return;
    HardwareSerial *serial = _started_serials[index];
    if (!serial || !serial->dma_unit_tx)
        return;
    uint32_t channel = serial->dma_ch_tx;
    DMA_TypeDef * dma = (DMA_TypeDef *)serial->dma_unit_tx;
    uint32_t mask, sr;
#ifdef STM32F7xx
    sr = (channel < 4) ? dma->LISR : dma->HISR;
    mask = DMA_LIFCR_CTCIF0_Msk;
#else
    sr = dma->ISR;
    mask = DMA_ISR_TCIF1_Msk;
#endif
    mask = dma_ifcr_mask_get(mask, channel);
    if (sr & mask) {
        LL_DMA_DisableChannel(dma, channel);
        if (!DMA_transmit(serial, channel)) {
            // Nothing to send
            serial->hw_enable_receiver();
        }
#ifdef STM32F7xx
        if (channel < 4)
            WRITE_REG(dma->LIFCR, mask);
        else
            WRITE_REG(dma->HIFCR, mask);
#else
        WRITE_REG(dma->IFCR, mask);
#endif
    }
}

HardwareSerial::HardwareSerial(uint32_t rx, uint32_t tx, uint8_t dma)
{
    rx_pin = rx;
    tx_pin = tx;
    p_duplex_pin = (struct gpio_out){.regs = NULL, .bit = 0};
    p_usart_tx = p_usart_rx = NULL;
    usart_irq_rx = usart_irq_tx = 0xff;
    p_use_dma = dma ? (USE_DMA_RX | USE_DMA_TX) : USE_DMA_NONE;
    usart_tx_idx = usart_rx_idx = 0xff;
    inverted = 0;
}

void HardwareSerial::setTx(uint32_t pin)
{
    tx_pin = pin;
}

void HardwareSerial::setRx(uint32_t pin)
{
    rx_pin = pin;
}

static void configure_uart_peripheral(USART_TypeDef * uart, uint32_t baud, uint8_t half_duplex)
{
    enable_pclock((uint32_t)uart);
    uint32_t pclk = get_pclock_frequency((uint32_t)uart);
#if defined(STM32F1xx)
    LL_USART_SetBaudRate(uart, pclk, baud);
#elif defined(STM32G0xx)
    LL_USART_SetBaudRate(uart, pclk, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, baud);
#else
    LL_USART_SetBaudRate(uart, pclk, LL_USART_OVERSAMPLING_16, baud);
#endif
    /* Configure other UART registers */
    LL_USART_ConfigAsyncMode(uart);
    if (half_duplex)
        LL_USART_EnableHalfDuplex(uart);
}

void HardwareSerial::begin(unsigned long baud, uint8_t mode)
{
    (void)mode;
    USART_TypeDef * uart = NULL;
    DMA_TypeDef * dmaptr;

    half_duplex = ((rx_pin == tx_pin) || (rx_pin == (uint32_t)UNDEF_PIN));

    p_usart_rx = (USART_TypeDef*)uart_peripheral_get(rx_pin);
    p_usart_tx = (USART_TypeDef*)uart_peripheral_get(tx_pin);

    if (half_duplex)
        p_usart_rx = p_usart_tx;

    if (p_usart_tx == USART1) {
        usart_irq_tx = USART1_IRQn;
        write_u8(&usart_tx_idx, 0);
        write_u32(&_started_serials[0], (uint32_t)this);
#ifdef USART2
    } else if (p_usart_tx == USART2) {
        usart_irq_tx = USART2_IRQn;
        write_u8(&usart_tx_idx, 1);
        write_u32(&_started_serials[1], (uint32_t)this);
#endif // USART2
#ifdef USART3
    } else if (p_usart_tx == USART3) {
        usart_irq_tx = USART3_IRQn;
        write_u8(&usart_tx_idx, 2);
        write_u32(&_started_serials[2], (uint32_t)this);
#endif // USART3
    } else {
        // Invalid HW UART config!
        return;
    }

    if (p_usart_rx != p_usart_tx) {
        if (p_usart_rx == USART1) {
            usart_irq_rx = USART1_IRQn;
            write_u8(&usart_rx_idx, 0);
            write_u32(&_started_serials[0], (uint32_t)this);
#ifdef USART2
        } else if (p_usart_rx == USART2) {
            usart_irq_rx = USART2_IRQn;
            write_u8(&usart_rx_idx, 1);
            write_u32(&_started_serials[1], (uint32_t)this);
#endif // USART2
#ifdef USART3
        } else if (p_usart_rx == USART3) {
            usart_irq_rx = USART3_IRQn;
            write_u8(&usart_rx_idx, 2);
            write_u32(&_started_serials[2], (uint32_t)this);
#endif // USART3
        } else {
            // Invalid HW UART config!
            return;
        }
    } else {
        write_u8(&usart_rx_idx, usart_tx_idx);
    }

    dma_unit_rx = dma_unit_tx = NULL;

    /* Init RX buffer */
    rx_head = rx_tail = 0;
#if UART_USE_TX_POOL_ONLY
    tx_buffer_ptr = NULL;
    tx_buffer_len = 0;
#else
    tx_head = tx_tail = 0;
#endif

    tx_pool_init();

    if ((p_use_dma & USE_DMA_RX) && p_usart_rx) {
        uart = (USART_TypeDef*)p_usart_rx;
        dmaptr = (DMA_TypeDef *)dma_get((uint32_t)uart, DMA_USART_RX, 0);
        if (dmaptr) {
            /*********** USART RX DMA Init ***********/
            dma_ch_rx = dma_channel_get((uint32_t)uart, DMA_USART_RX, 0);
            dma_irq_rx = dma_irq_get((uint32_t)uart, DMA_USART_RX, 0);
            /* Validate DMA params */
            if (dma_ch_rx != 0xFF && dma_irq_rx != 0xFF) {
                enable_pclock((uint32_t)dmaptr);
                dma_unit_rx = dmaptr;

                /* RX DMA stream config */
                LL_DMA_DisableChannel(dmaptr, dma_ch_rx);
                LL_DMA_SetDataTransferDirection(dmaptr, dma_ch_rx, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
                LL_DMA_SetChannelPriorityLevel(dmaptr, dma_ch_rx, LL_DMA_PRIORITY_LOW);
                LL_DMA_SetMode(dmaptr, dma_ch_rx, LL_DMA_MODE_CIRCULAR);
                LL_DMA_SetPeriphIncMode(dmaptr, dma_ch_rx, LL_DMA_PERIPH_NOINCREMENT);
                LL_DMA_SetMemoryIncMode(dmaptr, dma_ch_rx, LL_DMA_MEMORY_INCREMENT);
                LL_DMA_SetPeriphSize(dmaptr, dma_ch_rx, LL_DMA_PDATAALIGN_BYTE);
                LL_DMA_SetMemorySize(dmaptr, dma_ch_rx, LL_DMA_MDATAALIGN_BYTE);
                /* Set source and target */
                LL_DMA_SetPeriphAddress(dmaptr, dma_ch_rx, (uint32_t)&uart->RxDataReg);
                LL_DMA_SetMemoryAddress(dmaptr, dma_ch_rx, (uint32_t)rx_buffer);
                LL_DMA_SetDataLength(dmaptr, dma_ch_rx, sizeof(rx_buffer));
                /* Set interrupts */
                //LL_DMA_EnableIT_HT(dmaptr, dma_ch_rx);
                //LL_DMA_EnableIT_TC(dmaptr, dma_ch_rx);
                LL_DMA_DisableIT_HT(dmaptr, dma_ch_rx);
                LL_DMA_DisableIT_TC(dmaptr, dma_ch_rx);
                /* Set interrupt configuration */
                NVIC_SetPriority((IRQn_Type)dma_irq_rx,
                    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_UART_DMA, 0));
                NVIC_EnableIRQ((IRQn_Type)dma_irq_rx);

                dma_request_config((uint32_t)uart, DMA_USART_RX, 0);

                /* Enable DMA */
                LL_DMA_EnableChannel(dmaptr, dma_ch_rx);
            }
        }
    }

    if ((p_use_dma & USE_DMA_TX) && p_usart_tx) {
        uart = (USART_TypeDef*)p_usart_tx;
        dmaptr = (DMA_TypeDef *)dma_get((uint32_t)uart, DMA_USART_TX, 0);
        if (dmaptr) {
            /*********** USART TX DMA Init ***********/
            dma_ch_tx = dma_channel_get((uint32_t)uart, DMA_USART_TX, 0);
            dma_irq_tx = dma_irq_get((uint32_t)uart, DMA_USART_TX, 0);
            /* Validate DMA params */
            if (dma_ch_tx != 0xFF && dma_irq_tx != 0xFF) {
                enable_pclock((uint32_t)dmaptr);
                dma_unit_tx = dmaptr;

                /* TX DMA stream config */
                LL_DMA_DisableChannel(dmaptr, dma_ch_tx);
                LL_DMA_SetDataTransferDirection(dmaptr, dma_ch_tx, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
                LL_DMA_SetChannelPriorityLevel(dmaptr, dma_ch_tx, LL_DMA_PRIORITY_LOW);
                LL_DMA_SetMode(dmaptr, dma_ch_tx, LL_DMA_MODE_NORMAL);
                LL_DMA_SetPeriphIncMode(dmaptr, dma_ch_tx, LL_DMA_PERIPH_NOINCREMENT);
                LL_DMA_SetMemoryIncMode(dmaptr, dma_ch_tx, LL_DMA_MEMORY_INCREMENT);
                LL_DMA_SetPeriphSize(dmaptr, dma_ch_tx, LL_DMA_PDATAALIGN_BYTE);
                LL_DMA_SetMemorySize(dmaptr, dma_ch_tx, LL_DMA_MDATAALIGN_BYTE);
                /* Set target address */
                LL_DMA_SetPeriphAddress(dmaptr, dma_ch_tx, (uint32_t)&uart->TxDataReg);
                /* Set interrupts */
                LL_DMA_DisableIT_HT(dmaptr, dma_ch_tx);
                LL_DMA_EnableIT_TC(dmaptr, dma_ch_tx);
                /* Set interrupt configuration */
                NVIC_SetPriority((IRQn_Type)dma_irq_tx,
                    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_UART_DMA, 0));
                NVIC_EnableIRQ((IRQn_Type)dma_irq_tx);

                dma_request_config((uint32_t)uart, DMA_USART_TX, 0);
            }
        }
    }

    /* Create UART CR1 configurations for TX and RX modes */
    DR_RX = USART_CR1_UE | USART_CR1_RE;
    DR_TX = USART_CR1_UE | USART_CR1_TE;
    DR_RX |= ((dma_unit_rx) ? USART_CR1_IDLEIE : USART_CR1_RXNEIE);
    DR_TX |= ((dma_unit_tx) ? 0U : USART_CR1_TXEIE);
    if (!half_duplex && (p_usart_tx == p_usart_rx)) {
        if (!gpio_out_valid(p_duplex_pin)) {
            // Full duplex
            DR_RX |= USART_CR1_TE;  // Keep also TX active
            DR_TX |= DR_RX;         // Keep also RX active
        }
    }

    /*********** USART Init ***********/
    configure_uart_peripheral((USART_TypeDef*)p_usart_tx, baud, half_duplex);
    if (p_usart_rx != p_usart_tx) {
        configure_uart_peripheral((USART_TypeDef*)p_usart_rx, baud, uart_pin_is_tx(rx_pin));
#if 0 //defined(STM32F3xx)
        if (uart_pin_is_tx(rx_pin))
            LL_USART_SetTXRXSwap((USART_TypeDef*)p_usart_rx, LL_USART_TXRX_SWAPPED);
#endif
    }

#if defined(STM32F3xx)
    /* F3 can swap Rx and Tx pins */
    if (uart_tx_pin_is_rx(tx_pin)) {
        LL_USART_SetTXRXSwap((USART_TypeDef*)p_usart_tx, LL_USART_TXRX_SWAPPED);
    }
    /* F3 can invert uart lines */
    if (inverted) {
        LL_USART_SetTXPinLevel((USART_TypeDef*)p_usart_tx, LL_USART_TXPIN_LEVEL_INVERTED);
        LL_USART_SetRXPinLevel((USART_TypeDef*)p_usart_rx, LL_USART_RXPIN_LEVEL_INVERTED);
    }
#endif

    if (dma_unit_rx)
        LL_USART_EnableDMAReq_RX((USART_TypeDef*)p_usart_rx);
    else
        LL_USART_DisableDMAReq_RX((USART_TypeDef*)p_usart_rx);
    if (dma_unit_tx)
        LL_USART_EnableDMAReq_TX((USART_TypeDef*)p_usart_tx);
    else
        LL_USART_DisableDMAReq_TX((USART_TypeDef*)p_usart_tx);

    /* Configure to receiver mode by default */
    hw_enable_receiver();
    /* Enable UART IRQ */
    if (usart_irq_tx < 0xff) {
        NVIC_SetPriority((IRQn_Type)usart_irq_tx,
            NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_UART, 0));
        NVIC_EnableIRQ((IRQn_Type)usart_irq_tx);
    }
    if (usart_irq_rx < 0xff) {
        NVIC_SetPriority((IRQn_Type)usart_irq_rx,
            NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_UART, 0));
        NVIC_EnableIRQ((IRQn_Type)usart_irq_rx);
    }
    /* Configure UART pins to alternative mode */
    uart_config_afio((uint32_t)p_usart_tx, rx_pin, tx_pin);
}

void HardwareSerial::Pause(void)
{
    /* Stop RX UART */
    ((USART_TypeDef *)p_usart_rx)->CR1 = 0;
}

void HardwareSerial::Continue(void)
{
    /* Enable RX UART */
    ((USART_TypeDef *)p_usart_rx)->CR1 = DR_RX;
}

void HardwareSerial::end(void)
{
    if (dma_unit_rx) {
        LL_DMA_DisableChannel((DMA_TypeDef *)dma_unit_rx, dma_ch_rx);
        NVIC_DisableIRQ((IRQn_Type)dma_irq_rx);
    }
    if (dma_unit_tx) {
        LL_DMA_DisableChannel((DMA_TypeDef *)dma_unit_tx, dma_ch_tx);
        NVIC_DisableIRQ((IRQn_Type)dma_irq_tx);
    }
    if (p_usart_rx != p_usart_tx) {
        ((USART_TypeDef *)p_usart_rx)->CR1 = 0;
        if (usart_irq_rx < 0xff)
            NVIC_DisableIRQ((IRQn_Type)usart_irq_rx);
    }
    if (p_usart_tx) {
        ((USART_TypeDef *)p_usart_tx)->CR1 = 0;
        if (usart_irq_tx < 0xff)
            NVIC_DisableIRQ((IRQn_Type)usart_irq_tx);
    }
}

int HardwareSerial::available(void)
{
    //irqstatus_t flag = irq_save();
    //uint8_t head = read_u8(&rx_head), tail = read_u8(&rx_tail);
    uint8_t head = rx_head, tail = rx_tail;
    //irq_restore(flag);
    return (int)(head - tail);
}

int HardwareSerial::read(void)
{
    if (!available())
        return -1;
    //irqstatus_t flag = irq_save();
    uint8_t tail = read_u8(&rx_tail);
    write_u8(&rx_tail, tail+1);
    //irq_restore(flag);
    return rx_buffer[tail++];
}

void HardwareSerial::flush(void)
{
    // Wait until data is sent
    //while(read_u8(&tx_head) != read_u8(&tx_tail))
    //    ;
}

uint32_t HardwareSerial::write(const uint8_t *buff, uint32_t len)
{
#if UART_USE_TX_POOL_ONLY
    tx_pool_push(buff, len);
#endif
    if (dma_unit_tx) {
#if !UART_USE_TX_POOL_ONLY
        tx_pool_push(buff, len);
#endif
        if (!LL_DMA_IsEnabledChannel((DMA_TypeDef *)dma_unit_tx, dma_ch_tx))
            DMA_transmit(this, dma_ch_tx);
    } else {
#if UART_USE_TX_POOL_ONLY
        if (!read_u32(&tx_buffer_len)) {
            UART_transmit(this);
            hw_enable_transmitter();
        }
#else
        //irqstatus_t flag = irq_save();
        // push data into tx_buffer...
        uint8_t tmax = read_u8(&tx_head), tpos = read_u8(&tx_tail);
        if (tpos >= tmax) {
            tpos = tmax = 0;
            write_u8(&tx_head, 0);
            write_u8(&tx_tail, 0);
        }
        if ((tmax + len) > sizeof(tx_buffer)) {
            if ((tmax + len - tpos) > sizeof(tx_buffer))
                // Not enough space for message
                return 0;
            // Disable TX irq and move buffer
            write_u8(&tx_head, 0); // this stops TX irqs

            tpos = read_u8(&tx_tail);
            tmax -= tpos;
            memmove(&tx_buffer[0], &tx_buffer[tpos], tmax);
            write_u8(&tx_tail, 0);
            write_u8(&tx_head, tmax);
        }

        memcpy(&tx_buffer[tmax], buff, len);
        write_u8(&tx_head, (tmax + len));
        //irq_restore(flag);
        hw_enable_transmitter();
#endif
    }
    return len;
}

uint32_t HardwareSerial::write_direct(const uint8_t *buff, uint32_t len)
{
    USART_TypeDef * const uart_tx = (USART_TypeDef *)p_usart_tx;
    hw_enable_transmitter();
    uart_tx->CR1 &= ~USART_CR1_TXEIE;  // Disable TX ISR
    while (len--) {
        while(!(uart_tx->StatReg & USART_ISR_TXE));
        uart_tx->TxDataReg = *buff++;
    }
    hw_enable_receiver();
    return 0;
}

void FAST_CODE_2 HardwareSerial::hw_enable_receiver(void)
{
    USART_TypeDef * const uart = (USART_TypeDef *)p_usart_rx;
    uint8_t duplex = gpio_out_valid(p_duplex_pin);
    if (half_duplex || duplex) {
        USART_TypeDef * const uart_tx = (USART_TypeDef *)p_usart_tx;
        // Wait until transfer is completed
        while (!(uart_tx->StatReg & USART_SR_TC));
    }
    if (duplex) {
        gpio_out_write(p_duplex_pin, 0 ^ p_duplex_pin_inv);
    }
    uart->CR1 = DR_RX;
}

void FAST_CODE_2 HardwareSerial::hw_enable_transmitter(void)
{
    USART_TypeDef * uart = (USART_TypeDef *)p_usart_tx;
    uart->CR1 = DR_TX;
    if (gpio_out_valid(p_duplex_pin)) {
        gpio_out_write(p_duplex_pin, 1 ^ p_duplex_pin_inv);
    }
}

#if defined(DEFINE_SERIAL1)
#ifndef SERIAL1_USE_DMA
#define SERIAL1_USE_DMA 0
#endif
HardwareSerial Serial1(GPIO('A', 10), GPIO('A', 9), SERIAL1_USE_DMA);
#endif // DEFINE_SERIAL1

#if defined(DEFINE_SERIAL2)
#ifndef SERIAL2_USE_DMA
#define SERIAL2_USE_DMA 0
#endif
HardwareSerial Serial2(GPIO('A', 3), GPIO('A', 2), SERIAL2_USE_DMA);
#endif // DEFINE_SERIAL2

#if defined(DEFINE_SERIAL3)
#ifndef SERIAL3_USE_DMA
#define SERIAL3_USE_DMA 0
#endif
HardwareSerial Serial3(GPIO('B', 11), GPIO('B', 10), SERIAL3_USE_DMA);
#endif // DEFINE_SERIAL2
