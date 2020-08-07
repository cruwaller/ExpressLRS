#ifndef H_CRSF
#define H_CRSF

#include "HwSerial.h"
#include "helpers.h"
#include "crc.h"
#include "platform.h"
#include "utils.h"
#include "msp.h"

#define CRSF_RX_BAUDRATE 420000
#define CRSF_TX_BAUDRATE_FAST 400000
#define CRSF_TX_BAUDRATE_SLOW 115200 // Used for QX7 not supporting 400kbps
#define CRSF_NUM_CHANNELS 16         // Number of input channels
#define CRSF_CHANNEL_VALUE_MIN 172
#define CRSF_CHANNEL_VALUE_MID 992
#define CRSF_CHANNEL_VALUE_MAX 1811

#define CRSF_SYNC_BYTE 0xC8

#define RCframeLength sizeof(crsf_channels_t)                         // 22, length of the RC data packed bytes frame. 16 channels in 11 bits each.
#define LinkStatisticsFrameLength sizeof(crsfPayloadLinkstatistics_s) // 10
#define OpenTXsyncFrameLength 11                                      //
#define BattSensorFrameLength sizeof(crsf_sensor_battery_t)           // 8
#define VTXcontrolFrameLength 12                                      //
#define LUArespLength 6

#define CRSF_PAYLOAD_SIZE_MAX 62
#define CRSF_FRAME_NOT_COUNTED_BYTES 2
#define CRSF_MSP_FRAME_NOT_COUNTED_BYTES 6                                            // dest, orig, hdr, size, cmd, crc
#define CRSF_FRAME_SIZE(payload_size) ((payload_size) + CRSF_FRAME_NOT_COUNTED_BYTES) // See crsf_header_t.frame_size
#define CRSF_EXT_FRAME_SIZE(payload_size) (CRSF_FRAME_SIZE(payload_size) + CRSF_FRAME_NOT_COUNTED_BYTES)
#define CRSF_MSP_FRAME_SIZE(payload_size) ((payload_size) + CRSF_MSP_FRAME_NOT_COUNTED_BYTES)
#define CRSF_FRAME_SIZE_MAX (CRSF_PAYLOAD_SIZE_MAX + CRSF_FRAME_NOT_COUNTED_BYTES)

//////////////////////////////////////////////////////////////

enum crsf_frame_type_e
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
};

enum crsf_addr_e
{
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
};

typedef enum
{
    CRSF_UINT8 = 0,
    CRSF_INT8 = 1,
    CRSF_UINT16 = 2,
    CRSF_INT16 = 3,
    CRSF_UINT32 = 4,
    CRSF_INT32 = 5,
    CRSF_UINT64 = 6,
    CRSF_INT64 = 7,
    CRSF_FLOAT = 8,
    CRSF_TEXT_SELECTION = 9,
    CRSF_STRING = 10,
    CRSF_FOLDER = 11,
    CRSF_INFO = 12,
    CRSF_COMMAND = 13,
    CRSF_VTX = 15,
    CRSF_OUT_OF_RANGE = 127,
} crsf_value_type_e;

typedef struct crsf_header_s
{
    uint8_t device_addr; // from crsf_addr_e
    uint8_t frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t type;        // from crsf_frame_type_e
} PACKED crsf_header_t;

// Used by extended header frames (type in range 0x28 to 0x96)
typedef struct crsf_ext_header_s
{
    // Common header fields, see crsf_header_t
    uint8_t device_addr;
    uint8_t frame_size;
    uint8_t type;
    // Extended fields
    uint8_t dest_addr;
    uint8_t orig_addr;
} PACKED crsf_ext_header_t;

typedef struct crsf_channels_s
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED crsf_channels_t;

// Used by extended header frames (type in range 0x28 to 0x96)
typedef struct crsf_sensor_battery_s
{
    uint16_t voltage;  // mv * 100
    uint16_t current;  // ma * 100
    uint32_t capacity : 24; // mah
    uint32_t remaining : 8; // %
} PACKED crsf_sensor_battery_t;

/*
 * 0x14 Link statistics
 * Payload:
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Downlink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 250mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 *
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 * Uplink:   PILOT => UAV
 * Downlink: UAV   => PILOT
 */
typedef struct crsfPayloadLinkstatistics_s
{
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality; // this goes to opentx rssi
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
} crsfLinkStatistics_t;

typedef struct crsf_sensor_gps_s
{
    int32_t latitude;
    int32_t longitude;
    uint16_t speed;
    uint16_t heading;
    uint16_t altitude;
    uint8_t satellites;
    uint8_t valid; // available to send (1)
} PACKED crsf_sensor_gps_t;

/////inline and utility functions//////

#define CRSF_to_US(val) MAP_U16((val), CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 988, 2012)

#define UINT10_to_CRSF(val) MAP_U16((val), 0, 1024, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX)
#define CRSF_to_UINT10(val) MAP_U16((val), CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 0, 1023)

// 234 = (1811-172) / 7
#define CRSF_to_SWITCH3b(val) ((val) / 234)
#define SWITCH3b_to_CRSF(val) ((val)*234 + CRSF_CHANNEL_VALUE_MIN)

// 3 state aka 2b switches use 0, 1 and 2 as values to represent low, middle and high
// 819 = (1811-172) / 2
#define CRSF_to_SWITCH2b(val) ((val) / 819)
#define SWITCH2b_to_CRSF(val) ((val)*819 + CRSF_CHANNEL_VALUE_MIN)

#define CRSF_to_BIT(val) (((val) > 1000) ? 1 : 0)
#define BIT_to_CRSF(val) ((val) ? CRSF_CHANNEL_VALUE_MAX : CRSF_CHANNEL_VALUE_MIN)

class CRSF
{
public:
    CRSF(HwSerial *dev) : _dev(dev) {}
    CRSF(HwSerial &dev) : _dev(&dev) {}

    void Begin();

    ///// Callbacks /////
    static void (*RCdataCallback1)(crsf_channels_t const *const channels); //function pointer for new RC data callback
    static void (*MspCallback)(uint8_t const *const input);

    static void (*disconnected)();
    static void (*connected)();

    // Protocol funcs
    void LinkStatisticsExtract(volatile uint8_t const *const data,
                               int8_t snr,
                               uint8_t rssi);
    void ICACHE_RAM_ATTR LinkStatisticsPack(uint8_t *const output);

    void ICACHE_RAM_ATTR GpsStatsExtract(volatile uint8_t const *const data);
    uint8_t ICACHE_RAM_ATTR GpsStatsPack(uint8_t *const output);

    volatile crsfPayloadLinkstatistics_s LinkStatistics = {0}; // Link Statisitics Stored as Struct
    volatile crsf_sensor_battery_s TLMbattSensor = {0};
    volatile crsf_sensor_gps_s TLMGPSsensor = {0};

protected:
    uint8_t *ParseInByte(uint8_t inChar);
    virtual void LinkStatisticsSend(void) = 0;
    virtual void BatterySensorSend(void);

    HwSerial *_dev = NULL;

    /// UART validity check ///
    uint32_t GoodPktsCount = 0;
    uint32_t BadPktsCount = 0;

private:
    bool CRSFframeActive = false;
    uint8_t SerialInCrc = 0;
    uint8_t SerialInPacketStart = 0;
    uint8_t SerialInPacketLen = 0;               // length of the CRSF packet as measured
    uint8_t SerialInPacketPtr = 0;               // index where we are reading/writing
};

// CRSF frame TX buffer
extern uint8_t DMA_ATTR outBuffer[];

#endif
