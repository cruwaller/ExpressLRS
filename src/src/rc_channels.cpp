#include "rc_channels.h"
#include "common.h"
#include "helpers.h"
#include "FHSS.h"
#include "debug_elrs.h"

#include <assert.h>

// ------------------------------------------------------------------

#if N_SWITCHES < 2
#error "At least two (2) AUX channels are needed!"
#endif
#if 12 < N_SWITCHES
#error "Up to 12 AUX channels supported!"
#endif
#if (16 < N_CHANNELS)
#error "CRSF Channels Config is not OK"
#endif

#if OTA_PAYLOAD_SX127x < OTA_PAYLOAD_MIN
#error "SX127x payload size is invalid!"
#endif
#if OTA_PAYLOAD_SX128x < OTA_PAYLOAD_MIN
#error "SX128x payload size is invalid!"
#endif

// ------------------------------------------------------------------

// 7 state aka 3b switches use 0...6 as values to represent 7 different values
// 1984 / 6 = 330 => taken down a bit to align result more evenly
// (1811-172) / 6 = 273
#define CRSF_to_SWITCH3b(val) ((val) / 300)
#if PROTOCOL_CRSF_V3_TO_FC
#if CRSFv3_BITS == 10
#define SWITCH3b_to_CRSF(val) ((val) * 170) // round down 170.5 to 170
#elif CRSFv3_BITS == 11
#define SWITCH3b_to_CRSF(val) ((val) * 341) // round down 341.17 to 341
#elif CRSFv3_BITS == 12
#define SWITCH3b_to_CRSF(val) ((val) * 682) // round down 682.5 to 682
#endif
#else
#define SWITCH3b_to_CRSF(val) ((val) * 273 + CRSF_CHANNEL_OUT_VALUE_MIN)
#endif

// ------------------------------------------------------------------

// 3 state aka 2b switches use 0, 1 and 2 as values to represent low, middle and high
// 819 = (1811-172) / 2
#define CRSF_to_SWITCH2b(val) ((val) / 819)
#if PROTOCOL_CRSF_V3_TO_FC
#if CRSFv3_BITS == 10
#define SWITCH2b_to_CRSF(val) ((val) * 511) // round down 511.5 to 511
#elif CRSFv3_BITS == 11
#define SWITCH2b_to_CRSF(val) ((val) * 1023) // round down 1023.5 to 1023
#elif CRSFv3_BITS == 12
#define SWITCH2b_to_CRSF(val) ((val) * 2047) // round down 2047.5 to 2047
#endif
#else
#define SWITCH2b_to_CRSF(val) (((val) * 819) + CRSF_CHANNEL_OUT_VALUE_MIN)
#endif

#define CRSF_to_BIT(val) (((val) > 1000) ? 1 : 0)
#define BIT_to_CRSF(val) ((val) ? CRSF_CHANNEL_OUT_VALUE_MAX : CRSF_CHANNEL_OUT_VALUE_MIN)

#define SWITCH2b_to_3b(_D) ((_D) ? ((2 == (_D)) ? 6 : 3) : 0)

// ------------------------------------------------------------------

typedef void
(*channels_pack_t)(uint16_t const ch1, uint16_t const ch2,
                   uint16_t const ch3, uint16_t const ch4);
static channels_pack_t DRAM_ATTR channels_pack_ptr;


static_assert(sizeof(ElrsSyncPacket_s) == OTA_PAYLOAD_MIN,
              "Incorrect sync packet size!");


/*************************************************************************************
 * RC OTA PACKET
 *************************************************************************************/

typedef struct
{
    // Packet type
    unsigned pkt_type : 2;
    // The round-robin switch
#if N_SWITCHES <= 5
    unsigned aux_n_idx : 2; // 4 other channels
    unsigned aux_n : 2;
    unsigned aux0 : 2; // aux0 is sent always
#elif N_SWITCHES <= 8
    unsigned aux_n_idx : 3;
    unsigned aux_n : 3;
#else
    unsigned aux_n_idx : 4;
    unsigned aux_n : 2;
#endif
} PACKED RcDataSwitches_t;

static_assert(1 == sizeof(RcDataSwitches_t), "Switch pkt size is not correct");


typedef struct
{
    // The analog channels
    unsigned rc1 : RC_BITS_LEGACY;
    unsigned rc2 : RC_BITS_LEGACY;
    unsigned rc3 : RC_BITS_LEGACY;
    unsigned rc4 : RC_BITS_LEGACY;
    RcDataSwitches_t switches;
} PACKED RcDataPacket_s;

static_assert(sizeof(RcDataPacket_s) <= OTA_PAYLOAD_SX127x,
              "OTA pkt size is not correct");


typedef struct
{
    // The analog channels
    unsigned rc1 : RC_BITS_FULL;
    unsigned rc2 : RC_BITS_FULL;
    unsigned rc3 : RC_BITS_FULL;
    unsigned rc4 : RC_BITS_FULL;
    // Make sure the channels are byte aligned!
#if 0 < ((RC_BITS_FULL * 4) % 8)
    unsigned free : ((RC_BITS_FULL * 4) % 8);
#endif
    RcDataSwitches_t switches;
} PACKED RcDataPacketFull_s;

static_assert(sizeof(RcDataPacketFull_s) <= OTA_PAYLOAD_SX128x,
              "OTA pkt size is not correct");


// Holds last state of the switches
uint8_t DRAM_ATTR currentSwitches[N_SWITCHES];

// Packed OTA packet buffer
uint8_t DMA_ATTR packed_buffer[sizeof(RcDataPacketFull_s)];
uint8_t DRAM_ATTR packed_buffer_size;

// bitmap of changed switches
uint16_t DRAM_ATTR p_auxChannelsChanged;
// which switch should be sent in the next rc packet
uint8_t DRAM_ATTR p_nextSwitchIndex;


/**
 * Determine which switch to send next.
 *
 * If any switch has changed since last sent, it sends the lowest index changed switch.
 * If no switches have changed then this sends p_nextSwitchIndex and increment the value.
 */
FORCED_INLINE uint8_t
getNextSwitchIndex(void)
{
    uint16_t changed = p_auxChannelsChanged;
#if (N_SWITCHES <= 5)
    changed &= ~(0x1);
#endif

    /* Check if channel is changed and send it immediately,
     * otherwise send next sequential switch */
    int8_t index = __builtin_ffs(changed) - 1;
    if ((index < 0) || (N_SWITCHES < index)) {
        uint8_t s_index = p_nextSwitchIndex;
        index = s_index++;
        s_index %= N_SWITCHES;
#if N_SWITCHES <= 5
        if (!s_index) s_index++;
#endif
        p_nextSwitchIndex = s_index;
        changed = 0;
    } else {
        changed &= ~(0x1 << index);
    }
    p_auxChannelsChanged = changed;
    return index; // % N_SWITCHES;
}

/**
 * Sequential switches packet
 *
 * Cycle through N_SWITCHES switches on successive packets. If any switches have
 * changed takes the lowest indexed one and send that, hence lower indexed switches have
 * higher priority in the event that several are changed at once.
 */
FORCED_INLINE void
channel_pack_switches(RcDataSwitches_t * const switches)
{
    // find the next switch to send
    uint8_t const ch_idx = getNextSwitchIndex();
    // The round-robin switch
#if N_SWITCHES <= 5
    switches->aux0 = currentSwitches[0];
    switches->aux_n_idx = ch_idx - 1; // 1...4 => 0...3
#else
    switches->aux_n_idx = ch_idx;
#endif
    switches->aux_n = currentSwitches[ch_idx];
    // Set type
    switches->pkt_type = UL_PACKET_RC_DATA;
}


void
channels_pack_legacy(uint16_t const ch1, uint16_t const ch2,
                     uint16_t const ch3, uint16_t const ch4)
{
    RcDataPacket_s * const rcdata = (RcDataPacket_s *)&packed_buffer[0];
    // The analog channels, scale down to 10bits
    rcdata->rc1 = ch1;
    rcdata->rc2 = ch2;
    rcdata->rc3 = ch3;
    rcdata->rc4 = ch4;
    channel_pack_switches(&rcdata->switches);
}

void
channels_pack_full(uint16_t const ch1, uint16_t const ch2,
                   uint16_t const ch3, uint16_t const ch4)
{
    // find the next switch to send
    RcDataPacketFull_s * const rcdata = (RcDataPacketFull_s *)&packed_buffer[0];
    // The analog channels, scale down to 10bits
    rcdata->rc1 = ch1;
    rcdata->rc2 = ch2;
    rcdata->rc3 = ch3;
    rcdata->rc4 = ch4;
    channel_pack_switches(&rcdata->switches);
}

FORCED_INLINE void
channels_pack(uint16_t const ch1, uint16_t const ch2,
              uint16_t const ch3, uint16_t const ch4)
{
    uint32_t irq = _SAVE_IRQ();
    channels_pack_ptr(ch1, ch2, ch3, ch4);
    _RESTORE_IRQ(irq);
}


/**
 * Convert received OTA packet data to CRSF packet for FC
 */
FORCED_INLINE void
RcChannels_extract_switches(rc_channels_rx_t &PackedRCdataOut,
                            RcDataSwitches_t const * const switches)
{
    uint16_t switchValue;
    uint8_t switchIndex;

    // The round-robin switch
    switchIndex = switches->aux_n_idx;
#if PROTOCOL_ELRS_TO_FC
#if N_SWITCHES <= 5
    PackedRCdataOut.ch4 = SWITCH2b_to_3b(switches->aux0);
    switchValue = SWITCH2b_to_3b(switches->aux_n);
    switchIndex++; // values 1...4
#elif N_SWITCHES <= 8
    switchValue = switches->aux_n;
#else
    switchValue = SWITCH2b_to_3b(switches->aux_n);
#endif
#else // PROTOCOL_ELRS_TO_FC
#if N_SWITCHES <= 5
    PackedRCdataOut.ch4 = SWITCH2b_to_CRSF(switches->aux0);
    switchValue = SWITCH2b_to_CRSF(switches->aux_n);
    switchIndex++; // values 1...4
#elif N_SWITCHES <= 8
    switchValue = SWITCH3b_to_CRSF(switches->aux_n);
#else
    switchValue = SWITCH2b_to_CRSF(switches->aux_n);
#endif
#endif // PROTOCOL_ELRS_TO_FC

    switch (switchIndex) {
#if 5 < N_SWITCHES
        case 0:
            PackedRCdataOut.ch4 = switchValue;
            break;
#endif
        case 1:
            PackedRCdataOut.ch5 = switchValue;
            break;
#if 2 < N_SWITCHES
        case 2:
            PackedRCdataOut.ch6 = switchValue;
            break;
#endif
#if 3 < N_SWITCHES
        case 3:
            PackedRCdataOut.ch7 = switchValue;
            break;
#endif
#if 4 < N_SWITCHES
        case 4:
            PackedRCdataOut.ch8 = switchValue;
            break;
#endif
#if 5 < N_SWITCHES
        case 5:
            PackedRCdataOut.ch9 = switchValue;
            break;
#endif
#if 6 < N_SWITCHES
        case 6:
            PackedRCdataOut.ch10 = switchValue;
            break;
#endif
#if 7 < N_SWITCHES
        case 7:
            PackedRCdataOut.ch11 = switchValue;
            break;
#endif
#if 8 < N_SWITCHES
        case 8:
            PackedRCdataOut.ch12 = switchValue;
            break;
#endif
#if 9 < N_SWITCHES
        case 9:
            PackedRCdataOut.ch13 = switchValue;
            break;
#endif
#if 10 < N_SWITCHES
        case 10:
            PackedRCdataOut.ch14 = switchValue;
            break;
#endif
#if 11 < N_SWITCHES
        case 11:
            PackedRCdataOut.ch15 = switchValue;
            break;
#endif
        default:
            break;
    }
}

#if defined(RX_MODULE)
#if RADIO_SX127x
constexpr uint16_t channelMax = ANALOG_MAX_VAL(RC_BITS_LEGACY);

void FAST_CODE_1 RcChannels_channels_extract(uint8_t const *const input,
                                             rc_channels_rx_t &PackedRCdataOut)
{
    RcDataPacket_s const * const rcdata = (RcDataPacket_s *)&input[0];

#if !PROTOCOL_ELRS_TO_FC && PROTOCOL_CRSF_V3_TO_FC
    PackedRCdataOut.ch_idx = 0;
#if (CRSFv3_BITS == 10)
    PackedRCdataOut.ch_res = CRSFv3_RES_10B;
#elif (CRSFv3_BITS == 11)
    PackedRCdataOut.ch_res = CRSFv3_RES_11B;
#elif (CRSFv3_BITS == 12)
    PackedRCdataOut.ch_res = CRSFv3_RES_12B;
#else
#error "CRSFv3: only 10b, 11b or 12b"
#endif
#endif

#if PROTOCOL_ELRS_TO_FC || PROTOCOL_CRSF_V3_TO_FC
#if PROTOCOL_ELRS_TO_FC || (CRSFv3_BITS == 12)
    #define OUT_SCALE 2
#elif (CRSFv3_BITS == 11)
    #define OUT_SCALE 1
#elif (CRSFv3_BITS == 10)
    #define OUT_SCALE 0
#endif
    // The analog channels are sent as is
    PackedRCdataOut.ch0 = (uint16_t)rcdata->rc1 << OUT_SCALE;
    PackedRCdataOut.ch1 = (uint16_t)rcdata->rc2 << OUT_SCALE;
    PackedRCdataOut.ch2 = (uint16_t)rcdata->rc3 << OUT_SCALE;
    PackedRCdataOut.ch3 = (uint16_t)rcdata->rc4 << OUT_SCALE;
#else
    // Map input values to the CRSF analog output values
    PackedRCdataOut.ch0 = MAP_U16(
        (uint16_t)rcdata->rc1, ANALOG_MIN_VAL, channelMax,
        CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX);
    PackedRCdataOut.ch1 = MAP_U16(
        (uint16_t)rcdata->rc2, ANALOG_MIN_VAL, channelMax,
        CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX);
    PackedRCdataOut.ch2 = MAP_U16(
        (uint16_t)rcdata->rc3, ANALOG_MIN_VAL, channelMax,
        CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX);
    PackedRCdataOut.ch3 = MAP_U16(
        (uint16_t)rcdata->rc4, ANALOG_MIN_VAL, channelMax,
        CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX);
#endif

    RcChannels_extract_switches(PackedRCdataOut, &rcdata->switches);
}
#endif // RADIO_SX127x

#if RADIO_SX128x

constexpr uint16_t channelMax = ANALOG_MAX_VAL(RC_BITS_FULL);

void FAST_CODE_1 RcChannels_channels_extract(uint8_t const *const input,
                                             rc_channels_rx_t &PackedRCdataOut)
{
    RcDataPacketFull_s const * const rcdata = (RcDataPacketFull_s *)&input[0];

#if !PROTOCOL_ELRS_TO_FC && PROTOCOL_CRSF_V3_TO_FC
    PackedRCdataOut.ch_idx = 0;
#if (CRSFv3_BITS == 10)
    PackedRCdataOut.ch_res = CRSFv3_RES_10B;
#elif (CRSFv3_BITS == 11)
    PackedRCdataOut.ch_res = CRSFv3_RES_11B;
#elif (CRSFv3_BITS == 12)
    PackedRCdataOut.ch_res = CRSFv3_RES_12B;
#else
#error "CRSFv3: only 10b, 11b or 12b"
#endif
#endif

#if PROTOCOL_ELRS_TO_FC || PROTOCOL_CRSF_V3_TO_FC
#if PROTOCOL_ELRS_TO_FC || (CRSFv3_BITS == 12)
    #define OUT_SCALE 0
#elif (CRSFv3_BITS == 11)
    #define OUT_SCALE 1
#elif (CRSFv3_BITS == 10)
    #define OUT_SCALE 2
#endif
    // The analog channels are sent as is
    PackedRCdataOut.ch0 = (uint16_t)rcdata->rc1 >> OUT_SCALE;
    PackedRCdataOut.ch1 = (uint16_t)rcdata->rc2 >> OUT_SCALE;
    PackedRCdataOut.ch2 = (uint16_t)rcdata->rc3 >> OUT_SCALE;
    PackedRCdataOut.ch3 = (uint16_t)rcdata->rc4 >> OUT_SCALE;
#else
    // Map input values to the CRSF analog output values
    PackedRCdataOut.ch0 = MAP_U16(
        (uint16_t)rcdata->rc1, ANALOG_MIN_VAL, channelMax,
        CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX);
    PackedRCdataOut.ch1 = MAP_U16(
        (uint16_t)rcdata->rc2, ANALOG_MIN_VAL, channelMax,
        CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX);
    PackedRCdataOut.ch2 = MAP_U16(
        (uint16_t)rcdata->rc3, ANALOG_MIN_VAL, channelMax,
        CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX);
    PackedRCdataOut.ch3 = MAP_U16(
        (uint16_t)rcdata->rc4, ANALOG_MIN_VAL, channelMax,
        CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX);
#endif

    RcChannels_extract_switches(PackedRCdataOut, &rcdata->switches);
}
#endif // RADIO_SX128x
#endif // defined(RX_MODULE)


void RcChannels_initRcPacket(uint_fast8_t const payloadSize)
{
#if SERIAL_PASSTHROUGH
    packed_buffer_size = payloadSize - 1;
#else
    if (payloadSize <= sizeof(RcDataPacket_s)) {
        packed_buffer_size = sizeof(RcDataPacket_s);
        channels_pack_ptr = channels_pack_legacy;
        return;
    }
    packed_buffer_size = sizeof(RcDataPacketFull_s);
    channels_pack_ptr = channels_pack_full;
#endif
}

uint8_t FAST_CODE_1 RcChannels_payloadSizeGet(void)
{
    return packed_buffer_size;
}

uint16_t FAST_CODE_1 RcChannels_channelMaxValueGet(void)
{
    uint16_t const maxVal = (packed_buffer_size == sizeof(RcDataPacketFull_s)) ?
            ANALOG_MAX_VAL(RC_BITS_FULL) : ANALOG_MAX_VAL(RC_BITS_LEGACY);
    return maxVal;
}


#if TARGET_HANDSET
/**
 * Store channels input to local buffers for OTA packet.
 * Note: this does not do any scaling!!
 */
void FAST_CODE_1
RcChannels_processChannels(rc_channels_handset_t const *const rcChannels)
{
    uint16_t const * const ChannelDataIn = &rcChannels->aux[0];
    uint_fast8_t switch_state, idx;

    /**
     * Convert the rc data corresponding to switches to 2 bit values.
     *
     * I'm defining channels 4 through 11 inclusive as representing switches
     * Take the input values and convert them to the range 0 - 2.
     * (not 0-3 because most people use 3 way switches and expect the middle
     *  position to be represented by a middle numeric value)
     */
    for (idx = 0; idx < N_SWITCHES; idx++) {
#if N_SWITCHES <= 5 || 8 < N_SWITCHES
        switch_state = ChannelDataIn[idx];
        if (0b11 < switch_state) switch_state = 0b11;
#else
        // input is 0 - 2048, output is 0 - 7
        switch_state = SWITCH2b_to_3b(ChannelDataIn[idx]);
        if (0b111 < switch_state) switch_state = 0b111;
#endif
        // Check if state is changed
        if (switch_state != currentSwitches[idx]) {
            p_auxChannelsChanged |= (0x1 << idx);
            currentSwitches[idx] = switch_state;
        }
    }

    channels_pack(rcChannels->gimbal[0], rcChannels->gimbal[1],
                  rcChannels->gimbal[2], rcChannels->gimbal[3]);
}
#endif

/**
 * Convert received CRSF serial packet from handset and
 * store it to local buffers for OTA packet
 */
void FAST_CODE_1
RcChannels_processChannelsCrsf(rc_channels_module_t const *const rcChannels)
{
    uint16_t const maxVal = RcChannels_channelMaxValueGet();
    // channels input range: 0...2048
    uint16_t ChannelDataIn[N_SWITCHES] = {
        (uint16_t)rcChannels->ch4,
        (uint16_t)rcChannels->ch5,
#if 2 < N_SWITCHES
        (uint16_t)rcChannels->ch6,
#endif
#if 3 < N_SWITCHES
        (uint16_t)rcChannels->ch7,
#endif
#if 4 < N_SWITCHES
        (uint16_t)rcChannels->ch8,
#endif
#if 5 < N_SWITCHES
        (uint16_t)rcChannels->ch9,
#endif
#if 6 < N_SWITCHES
        (uint16_t)rcChannels->ch10,
#endif
#if 7 < N_SWITCHES
        (uint16_t)rcChannels->ch11
#endif
#if 8 < N_SWITCHES
    , (uint16_t)rcChannels->ch12
#endif
#if 9 < N_SWITCHES
    , (uint16_t)rcChannels->ch13
#endif
#if 10 < N_SWITCHES
    , (uint16_t)rcChannels->ch14
#endif
#if 11 < N_SWITCHES
    , (uint16_t)rcChannels->ch15
#endif
    };
    uint8_t switch_state;

    /**
     * Convert the rc data corresponding to switches to 2 bit values.
     *
     * I'm defining channels 4 through 11 inclusive as representing switches
     * Take the input values and convert them to the range 0 - 2.
     * (not 0-3 because most people use 3 way switches and expect the middle
     *  position to be represented by a middle numeric value)
     */
    for (uint8_t idx = 0; idx < N_SWITCHES; idx++) {
#if N_SWITCHES <= 5 || 8 < N_SWITCHES
        switch_state = CRSF_to_SWITCH2b(ChannelDataIn[idx]);
        if (0b11 < switch_state) switch_state = 0b11;
#else
        // input is 0 - 2048, output is 0 - 7
        switch_state = CRSF_to_SWITCH3b(ChannelDataIn[idx]);
        if (0b111 < switch_state) switch_state = 0b111;
#endif
        // Check if state is changed
        if (switch_state != currentSwitches[idx]) {
            p_auxChannelsChanged |= (0x1 << idx);
            currentSwitches[idx] = switch_state;
        }
    }
    // Map CRSF channel inputs to OTA bits
    ChannelDataIn[0] = MAP_U16((uint16_t)rcChannels->ch0,
        CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX,
        ANALOG_MIN_VAL, maxVal);
    ChannelDataIn[1] = MAP_U16((uint16_t)rcChannels->ch1,
        CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX,
        ANALOG_MIN_VAL, maxVal);
    ChannelDataIn[2] = MAP_U16((uint16_t)rcChannels->ch2,
        CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX,
        ANALOG_MIN_VAL, maxVal);
    ChannelDataIn[3] = MAP_U16((uint16_t)rcChannels->ch3,
        CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX,
        ANALOG_MIN_VAL, maxVal);
    channels_pack(ChannelDataIn[0], ChannelDataIn[1], ChannelDataIn[2], ChannelDataIn[3]);
}

void FAST_CODE_1 RcChannels_get_packed_data(uint8_t *const output)
{
    uint8_t iter = packed_buffer_size;
    uint32_t irq = _SAVE_IRQ();
    while (iter--)
        output[iter] = packed_buffer[iter];
    _RESTORE_IRQ(irq);
}

uint8_t FAST_CODE_1
RcChannels_get_arm_channel_state(void)
{
#if TX_SKIP_SYNC
    #if (N_SWITCHES <= AUX_CHANNEL_ARM)
        #error "Invalid AUX channel for arming!"
    #endif
#endif
    return !!currentSwitches[AUX_CHANNEL_ARM];
}

/*************************************************************************************
 * TELEMETRY OTA PACKET
 *************************************************************************************/
#define MSP_OTA_PKT_TYPE_BITS   (2)          // bits 7..6
#define MSP_OTA_LAST            (0x1 << 5)   // bit 5
#define MSP_OTA_FIRST           (0x1 << 4)   // bit 4
#define MSP_OTA_SEQ(x)          ((x) & 0xF) // bits 3..0

typedef struct {
    union {
        struct {
            uint8_t func;
            uint8_t payloadSize;
            uint8_t data[3];
        } hdr;
        struct {
            uint8_t data[5];
        } payload;
    };
    uint8_t flags;
} TlmDataPacket_s;

static_assert(sizeof(TlmDataPacket_s) <= OTA_PAYLOAD_MIN,
              "OTA pkt size is not correct");

bool FAST_CODE_1
RcChannels_tlm_ota_send(uint8_t *const output,
                        mspPacket_t &packet)
{
    TlmDataPacket_s * const tlm_ptr = (TlmDataPacket_s *)output;
    uint8_t flags = MSP_OTA_SEQ(packet.sequence_nbr);
    uint8_t iter;

    if (flags == 0) {
        /* First send header */
        flags |= MSP_OTA_FIRST;
        tlm_ptr->hdr.func = packet.function;
        tlm_ptr->hdr.payloadSize = packet.payloadSize;
        for (iter = 0; iter < sizeof(tlm_ptr->hdr.data); iter++)
            tlm_ptr->hdr.data[iter] = packet.readByte();
    } else {
        for (iter = 0; iter < sizeof(tlm_ptr->payload.data); iter++)
            tlm_ptr->payload.data[iter] = packet.readByte();
    }

    if (packet.iterated())
        flags |= MSP_OTA_LAST; // this is last junk

    tlm_ptr->flags = (flags << MSP_OTA_PKT_TYPE_BITS);

    //DEBUG_PRINTF("%x,%x,%x,%x,%x,%x\n", output[0],output[1],output[2],output[3],output[4],output[5]);

    packet.sequence_nbr++;
    return !!(flags & MSP_OTA_LAST);
}

bool FAST_CODE_1
RcChannels_tlm_ota_receive(uint8_t const *const input,
                           mspPacket_t &packet)
{
    TlmDataPacket_s const * const tlm_ptr = (TlmDataPacket_s *)input;
    uint8_t const flags = (tlm_ptr->flags >> MSP_OTA_PKT_TYPE_BITS); // remove pkt_type
    uint8_t const sequence_nbr = MSP_OTA_SEQ(flags);
    uint8_t iter;

    //DEBUG_PRINTF("%x,%x,%x,%x,%x,%x\n", input[0],input[1],input[2],input[3],input[4],input[5]);

    if ((flags & MSP_OTA_FIRST) && sequence_nbr == 0) {
        /* first junk, reset packet and start reception */
        packet.reset();
        packet.type = MSP_PACKET_TLM_OTA;
        packet.function = tlm_ptr->hdr.func;
        packet.payloadSize = tlm_ptr->hdr.payloadSize;
        for (iter = 0; iter < sizeof(tlm_ptr->hdr.data) && !packet.iterated(); iter++) {
            packet.addByte(tlm_ptr->hdr.data[iter]);
        }
        //DEBUG_PRINTF("flags: %u, func %u, size %u\n", packet.flags, packet.function, packet.payloadSize);
    } else {
        //DEBUG_PRINTF("DONE seq %u == %u data: ", sequence_nbr, packet.sequence_nbr);
        if (sequence_nbr == packet.sequence_nbr) {
            for (iter = 0; iter < sizeof(tlm_ptr->payload.data) && !packet.iterated(); iter++) {
                packet.addByte(tlm_ptr->payload.data[iter]);
                //DEBUG_PRINTF("0x%X,", tlm_ptr->payload.data[iter]);
            }
        } else {
            /* Mismatch - drop a package to avoid possible issues */
            packet.error = true;
            return true;
        }
        //DEBUG_PRINTF("\n");
        //DEBUG_PRINTF("DONE size %u, iter %u\n", packet.payloadSize, packet.payloadIterator);
    }

    if ((flags & MSP_OTA_LAST) || packet.iterated()) {
        /* Last junk, clear counter for TX to FC */
        packet.error = false;
        packet.payloadIterator = 0;
        packet.sequence_nbr = 0;
        return true;
    }
    packet.sequence_nbr++;
    return false;
}


/*************************************************************************************
 * LINK STATISTICS OTA PACKET
 *************************************************************************************/
#define FRAMETYPE_LINK_STATISTICS 0x14
void FAST_CODE_1
RcChannels_link_stas_pack(uint8_t *const output,
                          LinkStats_t &input, uint_fast8_t const ul_lq)
{
    // NOTE: output is only 5 bytes + 6bits (MSB)!!

    // OpenTX hard codes "rssi" warnings to the LQ sensor for crossfire, so the
    // rssi we send is for display only.
    // OpenTX treats the rssi values as signed.
    uint8_t openTxRSSI = input.link.uplink_RSSI_1;
    // truncate the range to fit into OpenTX's 8 bit signed value
    if (openTxRSSI > 127)
        openTxRSSI = 127;
    // convert to 8 bit signed value in the negative range (-128 to 0)
    openTxRSSI = 255 - openTxRSSI;
    output[0] = openTxRSSI;
    output[1] = (input.batt.voltage & 0xFF00) >> 8;
    output[2] = input.link.uplink_SNR;
    output[3] = ul_lq;
    output[4] = (input.batt.voltage & 0x00FF);
    output[5] = FRAMETYPE_LINK_STATISTICS << 2;
}

void FAST_CODE_1
RcChannels_link_stas_extract(uint8_t const *const input,
                             LinkStats_t &output,
                             int8_t snr, int16_t rssi)
{
    // NOTE: input is only 5 bytes + 6bits (MSB)!!
    if (rssi < INT8_MIN) rssi = INT8_MIN;
    if (12 < snr) snr = 12;
    else if (snr < -12) snr = 12;

    output.link.downlink_SNR = snr * 10;
    output.link.downlink_RSSI = 120 + rssi;

    if ((input[5] >> 2) == FRAMETYPE_LINK_STATISTICS) {
        output.link.uplink_RSSI_1 = input[0];
        output.link.uplink_RSSI_2 = 0;
        output.link.uplink_SNR = input[2];
        output.link.uplink_Link_quality = input[3];

        output.batt.voltage = ((uint16_t)input[1] << 8) + input[4];
    }
}

/*************************************************************************************
 * GPS OTA PACKET
 *************************************************************************************/
#define FRAMETYPE_GPS 0x02

void FAST_CODE_1
RcChannels_gps_extract(uint8_t const *const input, GpsOta_t & output)
{
    uint8_t type = input[5] >> 2;
    if ((type & 0xf) != FRAMETYPE_GPS)
        return;
    switch (type >> 4) {
        case 3:
            output.latitude = input[0];
            output.latitude <<= 8;
            output.latitude += input[1];
            output.latitude <<= 8;
            output.latitude += input[2];
            output.latitude <<= 8;
            output.latitude += input[3];
            output.speed = input[4];
            output.speed <<= 8;
            break;
        case 2:
            output.longitude = input[0];
            output.longitude <<= 8;
            output.longitude += input[1];
            output.longitude <<= 8;
            output.longitude += input[2];
            output.longitude <<= 8;
            output.longitude += input[3];
            output.speed += input[4];
            break;
        case 1:
            output.heading = input[0];
            output.heading <<= 8;
            output.heading += input[1];
            output.altitude = input[2];
            output.altitude <<= 8;
            output.altitude += input[3];
            output.satellites = input[4];
            output.pkt_cnt = 1;
            break;
    }
}

uint8_t FAST_CODE_1
RcChannels_gps_pack(uint8_t *const output, GpsOta_t & input)
{
    uint8_t type = (input.pkt_cnt << 4) + FRAMETYPE_GPS;
    if (!input.pkt_cnt)
        return 0;
    // GPS block is split into pieces
    switch (input.pkt_cnt--) {
        case 3:
            output[0] = (uint8_t)(input.latitude >> 24);
            output[1] = (uint8_t)(input.latitude >> 16);
            output[2] = (uint8_t)(input.latitude >> 8);
            output[3] = (uint8_t)input.latitude;
            output[4] = (uint8_t)(input.speed >> 8);
            break;
        case 2:
            output[0] = (uint8_t)(input.longitude >> 24);
            output[1] = (uint8_t)(input.longitude >> 16);
            output[2] = (uint8_t)(input.longitude >> 8);
            output[3] = (uint8_t)input.longitude;
            output[4] = (uint8_t)(input.speed);
            break;
        case 1:
            output[0] = (uint8_t)(input.heading >> 8);
            output[1] = (uint8_t)(input.heading);
            output[2] = (uint8_t)(input.altitude >> 8);
            output[3] = (uint8_t)(input.altitude);
            output[4] = input.satellites;
            break;
    }
    output[5] = type << 2;
    return 1;
}


/*************************************************************************************
 * DEVICE INFO OTA PACKET
 *************************************************************************************/
uint8_t FAST_CODE_1
RcChannels_dev_info_extract(uint8_t const *const input, DeviceInfo_t & output)
{
    if (input[0] == 0xCA && input[1] == 0xFE) {
        output.state = input[2];
        output.count = input[3];
        return 1;
    }
    return 0;
}

uint8_t FAST_CODE_1
RcChannels_dev_info_pack(uint8_t *const output, DeviceInfo_t & input)
{
    if (!input.transmit)
        return 0;
    output[0] = 0xCA;
    output[1] = 0xFE;
    output[2] = input.state;
    output[3] = input.count++;
    input.transmit = false;
    return 1;
}
