#include "CRSF.h"
#include "debug_elrs.h"
#include "crc.h"
#include "platform.h"
#if !BACKPACK_LOGGER_BUILD
#include "HwSerial.h"
#endif

GenericLutCRC<uint8_t, CRSF_GEN_POLY, 8> DMA_ATTR crc_crsf;

uint8_t DMA_ATTR SerialInBuffer[2][CRSF_FRAME_SIZE_MAX + 16];

//#define DBF_PIN_CRSF_PACKET 2
#ifdef DBF_PIN_CRSF_PACKET
struct gpio_out crsf_packet;
#endif

void CRSF::Begin()
{
#ifdef DBF_PIN_CRSF_PACKET
    crsf_packet = gpio_out_setup(DBF_PIN_CRSF_PACKET, 0);
#endif

    GoodPktsCount = 0;
    BadPktsCount = 0;
    CRSFframeActive = false;
    pingpong = false;
    p_buff = SerialInBuffer[pingpong];

#if !BACKPACK_LOGGER_BUILD
    _dev->flush_read();
#endif
}

uint8_t CRSF::CalcCRC(uint8_t const * data, uint8_t size) const
{
    return crc_crsf.calc(data, size);
}

uint8_t *CRSF::ParseInByte(uint8_t const inChar)
{
    uint8_t *packet_ptr = NULL;

    // CRSF Frame:
    // | address | payload_len | payload* | crc |

    if (CRSFframeActive == false)
    {
        if ((inChar == CRSF_ADDRESS_CRSF_RECEIVER) || (inChar == CRSF_ADDRESS_CRSF_TRANSMITTER) ||
            (inChar == CRSF_SYNC_BYTE) || (inChar == CRSF_ADDRESS_BROADCAST))
        {
            CRSFframeActive = true;
            SerialInPacketLen = 0;
#ifdef DBF_PIN_CRSF_PACKET
            gpio_out_write(crsf_packet, 1);
#endif
        }
        return NULL;
    }

    if (SerialInPacketLen == 0) // we read the packet length and save it
    {
        SerialInCrc = 0;
        SerialInPacketPtr = 0;
        SerialInPacketLen = inChar;
        if ((inChar < CRSF_FRAME_HEADER_BYTES) || (CRSF_PAYLOAD_SIZE_MAX < inChar))
        {
            // packet too long -> discard
            CRSFframeActive = false;
            BadPktsCount++;
        }
        return NULL;
    }

    if (CRSF_PAYLOAD_SIZE_MAX <= SerialInPacketPtr)
    {
        // we reached the maximum allowable packet length,
        // so start again because shit fucked up hey.
        CRSFframeActive = false;
        BadPktsCount++;
        return NULL;
    }

    // store byte
    p_buff[SerialInPacketPtr++] = inChar;

    if (SerialInPacketPtr < SerialInPacketLen)
    {
        // Calc crc on the fly
        SerialInCrc = crc_crsf.calc(inChar, SerialInCrc);
        return NULL;
    }

    /* Check packet CRC */
    if (SerialInCrc == inChar)
    {
        packet_ptr = p_buff;
        GoodPktsCount++;
        /* Swap the input buffer */
        pingpong ^= true;
        p_buff = SerialInBuffer[pingpong];
    }
    else
    {
#if 0
        // https://crccalc.com/
        // CRC algorithm: CRC-8/DVB-S2
        DEBUG_PRINTF("UART CRC %X != %X, len %u, data:\n", SerialInCrc, inChar, SerialInPacketLen);
        for (int i = 0; i < SerialInPacketLen; i++)
        {
            DEBUG_PRINTF(" 0x%X", p_buff[i]);
        }
        DEBUG_PRINTF("\n");
#elif defined(DEBUG_SERIAL)
        DEBUG_PRINTF("!C");
#endif
        BadPktsCount++;
    }

#ifdef DBF_PIN_CRSF_PACKET
    gpio_out_write(crsf_packet, 0);
#endif

    // packet handled, start next
    CRSFframeActive = false;
    SerialInPacketPtr = 0;
    SerialInPacketLen = 0;

    return packet_ptr;
}
