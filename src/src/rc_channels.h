#ifndef __RC_CHANNELS_H
#define __RC_CHANNELS_H

#include "platform.h"
#include "msp.h"
#include <stdint.h>

#if defined(TX_SKIP_SYNC_WHEN_ARMED)
#define TX_SKIP_SYNC 1
#else
#define TX_SKIP_SYNC 0
#endif
#define AUX_CHANNEL_ARM     0


#define OTA_PACKET_DATA     6
#define OTA_PACKET_CRC      2
#define OTA_PACKET_PAYLOAD  (OTA_PACKET_DATA)
#define OTA_PACKET_SIZE     (OTA_PACKET_PAYLOAD + OTA_PACKET_CRC)

// current and sent switch values
#define N_CONTROLS 4
#ifndef N_SWITCHES
#define N_SWITCHES 5
#endif
#define N_CHANNELS (N_CONTROLS + N_SWITCHES)


/*************************************************
 *    Data conversion macros [HANDSET]
 *************************************************/
/* Note: this must match to OTA bits */
#define ANALOG_BITS     10U
#define ANALOG_MIN_VAL  0U
#define ANALOG_MID_VAL  (1U << (ANALOG_BITS - 1))
#define ANALOG_MAX_VAL  ((1U << ANALOG_BITS) - 1)

#define SWITCH_MIN      0U
#define SWITCH_MID      1U
#define SWITCH_MAX      2U

/*************************************************
 *    Data conversion macros [CRSF]
 *************************************************/
/** NOTE
 * CRSF input range is [0...992...1984]
 * CRSF output range is [172...992...1811]
 **/
// OUT to flight controller
#define CRSF_CHANNEL_OUT_VALUE_MIN 172
#define CRSF_CHANNEL_OUT_VALUE_MID 992
#define CRSF_CHANNEL_OUT_VALUE_MAX 1811

// IN comming from handset
#define CRSF_CHANNEL_IN_VALUE_MIN 0
#define CRSF_CHANNEL_IN_VALUE_MID 992
#define CRSF_CHANNEL_IN_VALUE_MAX 1984

#define CRSFv3_BITS 11 //ANALOG_BITS

#define UINT10_to_CRSF(val) MAP_U16((val), 0, 1024, CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX)
#define CRSF_to_UINT10(val) MAP_U16((val), CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX, 0, 1023)

// 7 state aka 3b switches use 0...6 as values to represent 7 different values
// 1984 / 6 = 330 => taken down a bit to align result more evenly
// (1811-172) / 6 = 273
#define CRSF_to_SWITCH3b(val) ((val) / 300)
#if PROTOCOL_CRSF_V3_TO_FC
#if CRSFv3_BITS == 10
#define SWITCH3b_to_CRSF(val) ((val) * 170) // round down 170.5 to 170
#elif CRSFv3_BITS == 11
#define SWITCH3b_to_CRSF(val) ((val) * 341) // round down 341.17 to 341
#endif
#else
#define SWITCH3b_to_CRSF(val) ((val) * 273 + CRSF_CHANNEL_OUT_VALUE_MIN)
#endif

// 3 state aka 2b switches use 0, 1 and 2 as values to represent low, middle and high
// 819 = (1811-172) / 2
#define CRSF_to_SWITCH2b(val) ((val) / 819)
#if PROTOCOL_CRSF_V3_TO_FC
#if CRSFv3_BITS == 10
#define SWITCH2b_to_CRSF(val) ((val)*511) // round down 511.5 to 511
#elif CRSFv3_BITS == 11
#define SWITCH2b_to_CRSF(val) ((val)*1023) // round down 1023.5 to 1023
#endif
#else
#define SWITCH2b_to_CRSF(val) ((val)*819 + CRSF_CHANNEL_OUT_VALUE_MIN)
#endif

#define CRSF_to_BIT(val) (((val) > 1000) ? 1 : 0)
#define BIT_to_CRSF(val) ((val) ? CRSF_CHANNEL_OUT_VALUE_MAX : CRSF_CHANNEL_OUT_VALUE_MIN)

#define SWITCH2b_to_3b(_D) ((_D) ? ((2 == (_D)) ? 6 : 3) : 0)


/* This can be used to check if switch is set or not */
#if PROTOCOL_ELRS_TO_FC
#define SWITCH_IS_SET(val) (0 < (val))
#elif PROTOCOL_CRSF_V3_TO_FC
#define SWITCH_IS_SET(val) (100 < (val))
#else
#define SWITCH_IS_SET(val) ((CRSF_CHANNEL_OUT_VALUE_MIN+100) < (val))
#endif


// ExpressLRS packet header types for uplink OTA packets
enum {
    UL_PACKET_UNKNOWN = 0b00,
    UL_PACKET_RC_DATA = 0b01,
    UL_PACKET_SYNC = 0b10,
    UL_PACKET_MSP = 0b11,
};

// ExpressLRS packet header types for downlink OTA packets
enum {
    DL_PACKET_FREE1 = 0b00,
    DL_PACKET_TLM_MSP = 0b01,
    DL_PACKET_GPS = 0b10,
    DL_PACKET_TLM_LINK = 0b11,
};

#define SYNC_KEY    0x2A

typedef struct ElrsSyncPacket_s {
    uint16_t CRCCaesarCipher;
    uint8_t fhssIndex;
    uint8_t rxtx_counter;
    uint8_t tlm_interval    : 3,
            no_sync_armed   : 1,
            arm_aux         : 4;
    uint8_t pkt_type        : 2,
            sync_key        : 6;
} ElrsSyncPacket_s;


static FORCED_INLINE uint8_t
RcChannels_packetTypeGet(uint8_t const *const input)
{
    return input[OTA_PACKET_DATA-1] & 0b11;
}

static FORCED_INLINE void
RcChannels_packetTypeSet(uint8_t *const output, uint8_t type)
{
    uint8_t val = output[OTA_PACKET_DATA-1];
    val = (val & 0xFC) + (type & 0b11);
    output[OTA_PACKET_DATA-1] = val;
}


/*************************************************************************************
 * RC OTA PACKET
 *************************************************************************************/
typedef struct rc_channels_s
{
    /* CRSF V1 and V2 is 16 channels */
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
} PACKED rc_channels_t;


#if PROTOCOL_ELRS_TO_FC
typedef struct rc_channels_rx_s {
    // 64 bits of data (4 x 10 bits + 8 x 3 bits channels) = 8 bytes.
    unsigned int ch0 : ANALOG_BITS;
    unsigned int ch1 : ANALOG_BITS;
    unsigned int ch2 : ANALOG_BITS;
    unsigned int ch3 : ANALOG_BITS;
    unsigned int ch4 : 3;
    unsigned int ch5 : 3;
    unsigned int ch6 : 3;
    unsigned int ch7 : 3;
    unsigned int ch8 : 3;
    unsigned int ch9 : 3;
    unsigned int ch10 : 3;
    unsigned int ch11 : 3;
} PACKED rc_channels_rx_t;
#elif PROTOCOL_CRSF_V3_TO_FC

enum {
    CRSFv3_RES_10B = 0,
    CRSFv3_RES_11B,
    CRSFv3_RES_12B,
    CRSFv3_RES_13B,
};

typedef struct rc_channels_rx_s {
    /* CRSF V3 supports up to 24 channels */
    uint8_t ch_idx : 5, ch_res : 2, ch_reserved : 1;
    uint32_t ch0 : CRSFv3_BITS;
    uint32_t ch1 : CRSFv3_BITS;
    uint32_t ch2 : CRSFv3_BITS;
    uint32_t ch3 : CRSFv3_BITS;
    // switches:
    uint32_t ch4 : CRSFv3_BITS;
    uint32_t ch5 : CRSFv3_BITS;
#if 2 < N_SWITCHES
    uint32_t ch6 : CRSFv3_BITS;
#endif
#if 3 < N_SWITCHES
    uint32_t ch7 : CRSFv3_BITS;
#endif
#if 4 < N_SWITCHES
    uint32_t ch8 : CRSFv3_BITS;
#endif
#if 5 < N_SWITCHES
    uint32_t ch9 : CRSFv3_BITS;
#endif
#if 6 < N_SWITCHES
    uint32_t ch10 : CRSFv3_BITS;
#endif
#if 7 < N_SWITCHES
    uint32_t ch11 : CRSFv3_BITS;
#endif
#if 8 < N_SWITCHES
    uint32_t ch12 : CRSFv3_BITS;
#endif
#if 9 < N_SWITCHES
    uint32_t ch13 : CRSFv3_BITS;
#endif
#if 10 < N_SWITCHES
    uint32_t ch14 : CRSFv3_BITS;
#endif
#if 11 < N_SWITCHES
    uint32_t ch15 : CRSFv3_BITS;
#endif
} PACKED rc_channels_rx_t;
#else // !PROTOCOL_ELRS_TO_FC
typedef rc_channels_t rc_channels_rx_t;
#endif // PROTOCOL_ELRS_TO_FC


void
RcChannels_processChannels(rc_channels_t const *const channels);
void
RcChannels_processChannelsCrsf(rc_channels_t const *const channels);
void FAST_CODE_1
RcChannels_get_packed_data(uint8_t *const output);
uint8_t FAST_CODE_1
RcChannels_get_arm_channel_state(void);

// RX related
void FAST_CODE_1
RcChannels_channels_extract(uint8_t const *const input,
                            rc_channels_rx_t &output);

/*************************************************************************************
 * TELEMETRY OTA PACKET
 *************************************************************************************/

typedef void (*MspCallback_t)(uint8_t const *const input);

uint8_t FAST_CODE_1
RcChannels_tlm_ota_send(uint8_t *const output,
                        mspPacket_t &packet,
                        uint8_t tx=1);
uint8_t FAST_CODE_1
RcChannels_tlm_ota_receive(uint8_t const *const input,
                           mspPacket_t &packet);

/*************************************************************************************
 * LINK STATISTICS OTA PACKET
 *************************************************************************************/
typedef struct LinkStatsLink_s {
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
} PACKED LinkStatsLink_t;

typedef struct LinkStatsBatt_s {
    uint16_t voltage;  // mv * 100
    uint16_t current;  // ma * 100
    uint32_t capacity : 24; // mah
    uint32_t remaining : 8; // %
} PACKED LinkStatsBatt_t;

typedef struct LinkStats_s {
    LinkStatsLink_t link;
    LinkStatsBatt_t batt;
} PACKED LinkStats_t;

typedef void (*BattInfoCallback_t)(LinkStatsBatt_t *);

void FAST_CODE_1
RcChannels_link_stas_pack(uint8_t *const output,
                          LinkStats_t &input, uint_fast8_t ul_lq);
void FAST_CODE_1
RcChannels_link_stas_extract(uint8_t const *const input,
                             LinkStats_t &output,
                             int8_t snr, int16_t rssi);


/*************************************************************************************
 * GPS OTA PACKET
 *************************************************************************************/
typedef struct GpsOta_s {
    int32_t latitude;
    int32_t longitude;
    uint16_t speed;
    uint16_t heading;
    uint16_t altitude;
    uint8_t satellites;
    uint8_t pkt_cnt;
} PACKED GpsOta_t;

/* Packet received callback */
typedef void (*GpsCallback_t)(GpsOta_t * gps);

void FAST_CODE_1
RcChannels_gps_extract(uint8_t const *const input, GpsOta_t & output);
uint8_t FAST_CODE_1
RcChannels_gps_pack(uint8_t *const output, GpsOta_t & input);

#endif /* __RC_CHANNELS_H */
