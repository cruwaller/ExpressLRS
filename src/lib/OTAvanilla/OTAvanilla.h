/**
 * This file is part of ExpressLRS
 * See https://github.com/AlessandroAU/ExpressLRS
 *
 * This file provides utilities for packing and unpacking the data to
 * be sent over the radio link.
 */

#ifndef H_VANILLA_OTA
#define H_VANILLA_OTA

#include "msp.h"            // just to make pio happy
#include "RadioInterface.h" // just to make pio happy
#include "rc_channels.h"
#include <stdint.h>


#if TARGET_HANDSET && OTA_VANILLA_ENABLED

#define OTA4_PACKET_SIZE 8U
#define OTA_VANILLA_SIZE OTA4_PACKET_SIZE

#define CRSF_CHANNEL_VALUE_MIN  172
#define CRSF_CHANNEL_VALUE_1000 0
#define CRSF_CHANNEL_VALUE_MID  ((CRSF_CHANNEL_VALUE_2000 + CRSF_CHANNEL_VALUE_1000) / 2 - 1)
#define CRSF_CHANNEL_VALUE_2000 1023 // 10bits
#define CRSF_CHANNEL_VALUE_MAX  1811

// Used to XOR with OtaCrcInitializer and macSeed to reduce compatibility with previous versions.
// It should be incremented when the OTA packet structure is modified.
#define OTA_VERSION_ID      3


static inline uint32_t __attribute__((always_inline))
OTA_vanilla_get_channelMinValue(void)
{
    return CRSF_CHANNEL_VALUE_1000;
}

static inline uint32_t __attribute__((always_inline))
OTA_vanilla_get_channelMaxValue(void)
{
    return CRSF_CHANNEL_VALUE_2000;
}

static inline uint8_t __attribute__((always_inline))
OTA_vanilla_getArmChannelState(void)
{
    extern uint8_t arm_channel_state;
    return arm_channel_state;
}

void OTA_vanilla_processChannels(rc_channels_handset_t const *const channels);

void OTA_vanilla_SyncPacketData(uint8_t * const TXdataBuffer, uint32_t NonceTX,
                                uint8_t tlm_interval);
void OTA_vanilla_PackChannelData(uint8_t * const TXdataBuffer, uint32_t NonceTX,
                                 uint8_t NonceFHSSresult);

void OTA_vanilla_calculateCrc(uint8_t * const TXdataBuffer, uint16_t cipher);

void OTA_vanilla_set_version(uint8_t version);

#endif // TARGET_HANDSET

#endif // H_VANILLA_OTA
