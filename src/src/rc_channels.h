#ifndef __RC_CHANNELS_H
#define __RC_CHANNELS_H

#include "platform.h"
#include "CRSF.h" // N_SWITCHES
#include "msp.h"
#include <stdint.h>

#define OTA_PACKET_DATA     6
#define OTA_PACKET_CRC      2
#define OTA_PACKET_SIZE     (OTA_PACKET_DATA+OTA_PACKET_CRC)

// current and sent switch values
#define N_CONTROLS 4
#define N_SWITCHES 8
#define N_CHANNELS 16 // (N_CONTROLS + N_SWITCHES)

// expresslrs packet header types
// 00 -> standard 4 channel data packet
// 01 -> switch data packet
// 10 -> sync packet with hop data
// 11 -> tlm packet (MSP)
enum
{
    UL_PACKET_RC_DATA = 0b00,
    UL_PACKET_SWITCH_DATA = 0b01,
    UL_PACKET_SYNC = 0b10,
    UL_PACKET_MSP = 0b11,
};

enum
{
    DL_PACKET_FREE1 = 0b00,
    DL_PACKET_TLM_MSP = 0b01,
    DL_PACKET_FREE2 = 0b10,
    DL_PACKET_TLM_LINK = 0b11,
};

#define USE_CRC_CAESAR_CIPHER_IN_SYNC 1
typedef struct ElrsSyncPacket_s {
#if USE_CRC_CAESAR_CIPHER_IN_SYNC
    uint16_t CRCCaesarCipher;
    uint8_t padding;
#endif
    uint8_t fhssIndex;
    uint8_t rxtx_counter;
#if RX_UPDATE_AIR_RATE
    uint8_t air_rate: 4;
    uint8_t tlm_interval : 4;
#else
    uint8_t tlm_interval;
#endif
#if !USE_CRC_CAESAR_CIPHER_IN_SYNC
    uint8_t uid3;
    uint8_t uid4;
    uint8_t uid5;
#endif
} ElrsSyncPacket_s;

#define TYPE_PACK(T) (((T) & 0b11) << 6)
#define TYPE_EXTRACT(B) (((B) >> 6) & 0b11)

#if (OTA_PACKET_DATA < 6)
#error "Min OTA size is 6 bytes!"
#endif

class RcChannels
{
public:
    RcChannels() {}

    // TX related
    void processChannels(crsf_channels_t const *const channels);
    void ICACHE_RAM_ATTR get_packed_data(uint8_t *const output)
    {
        for (uint8_t i = 0; i < sizeof(packed_buffer); i++)
            output[i] = packed_buffer[i];
    }

    // RX related
    void ICACHE_RAM_ATTR channels_extract(volatile uint8_t const *const input,
                                          crsf_channels_t &output);

    // TLM pkt
    uint8_t ICACHE_RAM_ATTR tlm_send(uint8_t *const output,
                                     mspPacket_t &packet);
    uint8_t ICACHE_RAM_ATTR tlm_receive(volatile uint8_t const *const input,
                                        mspPacket_t &packet);

private:
    void channels_pack(void);
    // Switches / AUX channel handling
    uint8_t getNextSwitchIndex(void);
    void setPacketType(uint8_t type) {
        packed_buffer[OTA_PACKET_DATA] = TYPE_PACK(type);
    }

    // Channel processing data
    volatile uint16_t ChannelDataIn[N_CHANNELS] = {0};  // range: 0...2048
    volatile uint8_t currentSwitches[N_SWITCHES] = {0}; // range: 0,1,2

    // esp requires aligned buffer
    volatile uint8_t WORD_ALIGNED_ATTR packed_buffer[OTA_PACKET_SIZE];

    volatile uint16_t p_auxChannelsChanged = 0; // bitmap of changed switches
    // which switch should be sent in the next rc packet
    volatile uint8_t p_nextSwitchIndex = 0;

#if !defined(HYBRID_SWITCHES_8) && !defined(SEQ_SWITCHES)
    uint32_t SwitchPacketNextSend = 0; //time in ms when the next switch data packet will be send
#define SWITCH_PACKET_SEND_INTERVAL 200u
#endif
};

#endif /* __RC_CHANNELS_H */
