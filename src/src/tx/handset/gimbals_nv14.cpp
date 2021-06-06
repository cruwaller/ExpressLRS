#include "gimbals.h"
#include "gpio.h"
#include "internal.h"
#include "stm32_def.h"
#include "helpers.h"
#include "targets.h"
#include "debug_elrs.h"
#include "rc_channels.h"
#include "priorities.h"
#include "common_defs.h"
#include "HardwareSerial.h"
#include <stddef.h>

#if TX_NUM_ANALOGS != NUM_ANALOGS
#error "Invalid configuration!"
#endif


#if (HANDSET_GIMBALS == GIMBALS_NV14)
/** FlSky NV14 gimbal driver.
 *      Gimbals are using UART communication to exchange data
 */

#define NV14_MIN_VAL    0
#define NV14_MID_VAL    2047
#define NV14_MAX_VAL    4095
#define NV14_BAUDRATE   921600

#ifndef NV14_UART_TX
#define NV14_UART_TX    GPIO('C', 6)
#endif
#ifndef NV14_UART_RX
#define NV14_UART_RX    GPIO('C', 7)
#endif

#define TIMx_ISR_EN     0
#define TIMx            TIM6
#define TIMx_IRQn       TIM6_DAC_IRQn
#define TIMx_IRQx_FUNC  TIM6_DAC_IRQHandler
#define TIM_INVERVAL_US 1000U

#define TIM_MARGIN_US   110U

#if TIMx_ISR_EN
#define DBG_PIN SWITCH_6_1
#ifdef DBG_PIN
static struct gpio_out debug_pin;
#endif
#endif


static uint16_t calc_crc16(uint8_t byte, uint16_t crc16);
static uint16_t calc_crc16_len(const uint8_t *pBuffer, uint8_t BufferSize);


/* Definitions copied from EdgeTX implementation
 *  https://github.com/EdgeTX/edgetx/blob/2.4/radio/src/targets/nv14/hallStick_driver.h
 */

typedef struct
{
    int16_t min;
    int16_t mid;
    int16_t max;
} STICK_CALIBRATION_S;

typedef struct
{
    STICK_CALIBRATION_S channel[4];
} STICK_CALIBRATION_PACK_S;

typedef struct
{
    int16_t  channel[4];
} CHANNEL_PACK_S;

typedef union
{
    struct {
        uint8_t senderID:2;
        uint8_t receiverID:2;
        uint8_t packetID:4;
    };
    uint8_t id;
} HALLID_S;

enum {
    GET_START = 0,
    GET_ID,
    GET_LENGTH,
    GET_DATA,
    GET_STATE,
    GET_CHECKSUM,
    CHECKSUM,
};

enum HALLSTICK_SEND_STATE_E {
    HALLSTICK_SEND_STATE_IDLE,
    HALLSTICK_STATE_SEND_RESET,
    HALLSTICK_STATE_GET_CONFIG,
    HALLSTICK_STATE_GET_FIRMWARE,
    HALLSTICK_STATE_UPDATE_FW
};

enum TRANSFER_DIR_E {
    TRANSFER_DIR_HALLSTICK,
    TRANSFER_DIR_TXMCU,
    TRANSFER_DIR_HOSTPC,
    TRANSFER_DIR_RFMODULE,
};

#define NV14_PROTOLO_HEAD       0x55
#define NV14_REQ_FIRMWARE       0x0A // Name??
#define NV14_REQ_COMMAND        0x0D // Name??
#define NV14_RESP_TYPE_VALUES   0x0C
#define NV14_RESP_TYPE_CALIB    0x0E

#define NV14_CRC16_INIT         0xffff


typedef struct {
    union nv14_msg {
        struct {
            uint8_t  head;
            HALLID_S id;
            uint8_t  len;
            uint8_t  data[1];
        } header;
        uint8_t payload[128];
    } msg;
    uint16_t crc, rx_crc;
    uint8_t iterator, crc_iter;

    void reset(void) {
        memset(&msg, 0, sizeof(msg));
        iterator = 0;
        crc_iter = 0;
        rx_crc = 0;
        crc = NV14_CRC16_INIT;
    }

    uint8_t ready(void) const {
        return (iterator >= (msg.header.len + offsetof(union nv14_msg, header.data)));
    }

    uint8_t add(uint8_t const b) {
        msg.payload[iterator++] = b;
        crc = calc_crc16(b, crc);
        return ready();
    }

    uint8_t get_status_byte(void) const {
        return msg.header.data[msg.header.len - 1];
    }

} nv14_message_t;

static nv14_message_t rx_msg;

static HardwareSerial uart_gimbal(NV14_UART_RX, NV14_UART_TX, true);

static STICK_CALIBRATION_PACK_S DMA_ATTR calibration;
static CHANNEL_PACK_S DMA_ATTR raw_values;
static uint16_t DMA_ATTR adc_values[NUM_ANALOGS];


/* crc16 implementation according to CCITT standards (POLY 0x1021) */
static uint16_t CRC16Table[256] = {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

static uint16_t calc_crc16(uint8_t const data, uint16_t const crc16)
{
    return (crc16 << 8) ^ CRC16Table[((crc16 >> 8) ^ data) & 0x00ff];
}

static uint16_t calc_crc16_len(const uint8_t *pBuffer, uint8_t BufferSize)
{
    uint16_t crc16 = NV14_CRC16_INIT;
    while (BufferSize--) {
        //crc16 = (crc16 << 8) ^ CRC16Table[((crc16 >> 8) ^ (*pBuffer)) & 0x00ff];
        //pBuffer++;
        crc16 = calc_crc16(*pBuffer++, crc16);
    }
    return crc16;
}

static void HallSendBuffer(uint8_t * buffer, uint32_t const count)
{
    /* Calculate command CRC */
    uint16_t const crc16 = calc_crc16_len(buffer, (count - 2));
    buffer[count - 2] = crc16 & 0xff;
    buffer[count - 1] = (crc16 >> 8) & 0xff;
    /* Send it */
    uart_gimbal.write(buffer, count);
}

struct hall_stick_cmd_value {
    uint8_t  head;
    HALLID_S id;
    uint8_t  len;
    uint8_t  value;
    uint8_t  crc[2];
};

struct hall_stick_cmd_zero {
    uint8_t  head;
    HALLID_S id;
    uint8_t  len;
    uint8_t  crc[2];
};

static void reset_hall_stick(void)
{
    static struct hall_stick_cmd_value cmd_stick;
    cmd_stick.head = NV14_PROTOLO_HEAD;
    //cmd_stick.id.id = 0xD1;
    cmd_stick.id.receiverID = TRANSFER_DIR_HALLSTICK;
    cmd_stick.id.senderID = TRANSFER_DIR_TXMCU;
    cmd_stick.id.packetID = NV14_REQ_COMMAND;
    cmd_stick.len = 0x01;
    cmd_stick.value = 0x01;
    HallSendBuffer((uint8_t*)&cmd_stick, sizeof(cmd_stick));
}

static void get_hall_config(void)
{
    static struct hall_stick_cmd_value cmd_config;
    cmd_config.head = NV14_PROTOLO_HEAD;
    //cmd_config.id.id = 0xD1;
    cmd_config.id.receiverID = TRANSFER_DIR_HALLSTICK;
    cmd_config.id.senderID = TRANSFER_DIR_TXMCU;
    cmd_config.id.packetID = NV14_REQ_COMMAND;
    cmd_config.len = 0x01;
    cmd_config.value = 0x00;
    HallSendBuffer((uint8_t*)&cmd_config, sizeof(cmd_config));
    // CRC = 2B 2C
}

#if 0
static void get_hall_firmware_info(void)
{
    static struct hall_stick_cmd_zero cmd_fw_info;
    cmd_fw_info.head = NV14_PROTOLO_HEAD;
    //cmd_fw_info.id.id = 0xA2;
    cmd_fw_info.id.receiverID = TRANSFER_DIR_HALLSTICK;
    cmd_fw_info.id.senderID = TRANSFER_DIR_HOSTPC;
    cmd_fw_info.id.packetID = NV14_REQ_FIRMWARE;
    cmd_fw_info.len = 0x00;
    // CRC = BE 02

    HallSendBuffer((uint8_t*)&cmd_fw_info, sizeof(cmd_fw_info));
}

static void hallStickUpdatefwEnd(void)
{
    static struct hall_stick_cmd_value cmd_fw_end;
    cmd_fw_end.head = NV14_PROTOLO_HEAD;
    //cmd_fw_end.id.id = 0xA2;
    cmd_fw_end.id.receiverID = TRANSFER_DIR_HALLSTICK;
    cmd_fw_end.id.senderID = TRANSFER_DIR_HOSTPC;
    cmd_fw_end.id.packetID = NV14_REQ_FIRMWARE;
    cmd_fw_end.len = 0x01;
    cmd_fw_end.value = 0x07;
    // CRC = 94 DD

    HallSendBuffer((uint8_t*)&cmd_fw_end, sizeof(cmd_fw_end));
}
#endif


extern "C" {
void FAST_CODE_1 TIMx_IRQx_FUNC(void)
{
    uint16_t const SR = TIMx->SR;
    if (SR & TIM_SR_UIF) {
        TIMx->SR = SR & ~(TIM_SR_UIF);
#ifdef DBG_PIN
        gpio_out_write(debug_pin, 1);
#endif
    }
}
}

static FORCED_INLINE void timer_reset_period(void)
{
    TIMx->ARR = TIM_INVERVAL_US - 1;
}

static void configure_timer(void)
{
    enable_pclock((uint32_t)TIMx);

    /* Config timer to trigger conversions */
    TIMx->CR1 = 0;
    TIMx->DIER = 0;
    TIMx->SR &= ~(TIM_SR_UIF);
    // Set clock prescaler to 1us
    TIMx->PSC = (2 * get_pclock_frequency((uint32_t)TIMx) / 1000000) - 1;
    TIMx->ARR = TIM_INVERVAL_US - 1;
    TIMx->CNT = 0;
    //MODIFY_REG(TIMx->CR2, TIM_CR2_MMS, 0x2 << TIM_CR2_MMS_Pos); // TRGO update
#if TIMx_ISR_EN
    //TIMx->EGR = TIM_EGR_UG;
    TIMx->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIMx_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(TIMx_IRQn);
#endif
    TIMx->CR1 = TIM_CR1_CEN | TIM_CR1_URS | TIM_CR1_DIR;
}

void gimbals_init(void)
{
#ifdef DBG_PIN
    debug_pin = gpio_out_setup(DBG_PIN, 0);
#endif

    /* Init structs */

    /* Init UART */
    uart_gimbal.end();
    uart_gimbal.begin(NV14_BAUDRATE);

    /* Initial config */
    reset_hall_stick();

    /* Start timer to read data from gimbals */
    configure_timer();
}

void FAST_CODE_1 gimbals_timer_adjust(uint32_t us)
{
    // Counts up
    TIMx->CNT = TIM_MARGIN_US;
}

void FAST_CODE_1 gimbals_get(uint16_t * const out)
{
    struct gimbal_limit * limit;
    uint32_t curr;
    uint8_t iter;
    for (iter = 0; iter < ARRAY_SIZE(pl_config.gimbals); iter++) {
        limit = &pl_config.gimbals[iter];
        curr = adc_values[iter];
        if (curr <= limit->mid)
            out[iter] = MAP_U16((uint16_t)curr, limit->low, limit->mid,
                                ANALOG_MIN_VAL, ANALOG_MID_VAL);
        else
            out[iter] = MAP_U16((uint16_t)curr, limit->mid+1, limit->high,
                                ANALOG_MID_VAL, ANALOG_MAX_VAL);
    }
}


#define ERROR_OFFSET 10
static void FAST_CODE_1 convert_raw_to_adc_value(void)
{
    STICK_CALIBRATION_S const * calib;
    uint16_t value, input;
    uint8_t channel;
    for (channel = 0; channel < ARRAY_SIZE(raw_values.channel); channel++ ) {
        calib = &calibration.channel[channel];
        input = raw_values.channel[channel];
        if (input < calib->mid) {
            value = calib->mid - (calib->min + ERROR_OFFSET);
            value = (NV14_MID_VAL * (calib->mid - input)) / value;

            if (value >= NV14_MID_VAL)
                value = NV14_MID_VAL;

            adc_values[channel] = NV14_MID_VAL - value;
        } else {
            value = (calib->max - ERROR_OFFSET) - calib->mid;
            value = (NV14_MID_VAL * (input - calib->mid)) / value;

            if (value >= NV14_MID_VAL)
                value = NV14_MID_VAL;

            adc_values[channel] = NV14_MID_VAL + value + 1;
        }
    }
}

static void FAST_CODE_1 message_process(void)
{
    switch (rx_msg.msg.header.id.receiverID) {
        case TRANSFER_DIR_TXMCU:
            if (rx_msg.msg.header.id.packetID == NV14_RESP_TYPE_CALIB) {
                memcpy(&calibration, rx_msg.msg.header.data, sizeof(calibration));
            } else if (rx_msg.msg.header.id.packetID == NV14_RESP_TYPE_VALUES) {
                memcpy(&raw_values, rx_msg.msg.header.data, sizeof(raw_values));
                convert_raw_to_adc_value();
            }
            break;
        case TRANSFER_DIR_HOSTPC:
            if (rx_msg.msg.header.len == 0x01 &&
                    (rx_msg.msg.header.data[0] == 0x05 || rx_msg.msg.header.data[0] == 0x06)) {
                //hallStickSendState = HALLSTICK_SEND_STATE_IDLE;
            }
        case TRANSFER_DIR_HALLSTICK:
#if 0
            HallProtocolCount++;
            uint8_t *pt = (uint8_t*)&HallProtocol;
            //HallProtocol.head = NV14_PROTOLO_HEAD;
            //TRACE("HALL: %02X %02X %02X ...%04X", pt[0], pt[1], pt[2], HallProtocol.checkSum);
            pt[HallProtocol.length + 3] = HallProtocol.checkSum & 0xFF;
            pt[HallProtocol.length + 4] = HallProtocol.checkSum >> 8;
            usbDownloadTransmit( pt, HallProtocol.length + 5 );
#endif
            break;
    }
}


static uint8_t rx_state = GET_START;
static uint32_t last_calibrate_get;

void FAST_CODE_1 gimbals_process_loop(void)
{
    uint8_t data;
    while (uart_gimbal.available()) {
        data = uart_gimbal.read();
        switch (rx_state) {
            case GET_START:
                if (NV14_PROTOLO_HEAD == data) {
                    rx_msg.reset();
                    rx_msg.add(data);
                    rx_state = GET_ID;
                }
                break;
            case GET_ID:
                rx_msg.add(data);
                rx_state = GET_LENGTH;
                break;
            case GET_LENGTH:
                rx_msg.add(data);
                rx_state = GET_DATA;
                break;
            case GET_DATA:
                if (rx_msg.add(data))
                    rx_state = GET_CHECKSUM;
                break;
            case GET_CHECKSUM:
                rx_msg.rx_crc += (uint16_t)data << (8 * rx_msg.crc_iter++);
                if (2 <= rx_msg.crc_iter) {
                    if (rx_msg.rx_crc == rx_msg.crc) {
                        /* Process msg */
                        message_process();
                    }
                    rx_state = GET_START;
                }
                break;
            default:
                rx_state = GET_START;
                break;
        }
    }

    uint32_t const now = millis();
    // Validate calibration periodically
    if (200U <= (millis() - last_calibrate_get)) {
        // invalid calibration
        if ((calibration.channel[0].max - calibration.channel[0].min) < 1024) {
            get_hall_config();
            last_calibrate_get = now;
        }
    }
}

uint8_t gimbals_calibrate(uint8_t * data)
{
    //uint8_t type = data[0], res = 0;
    if (data[1] == 0) {
        /* Stop if needed... */
        return 0;
    }
    data[0] = 0; //res;
    return 1;
}

#endif /* HANDSET_GIMBALS == GIMBALS_NV14 */
