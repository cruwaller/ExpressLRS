#include "CRSF.h"
#include "debug_elrs.h"

uint8_t DMA_ATTR SerialInBuffer[/*CRSF_EXT_FRAME_SIZE(CRSF_PAYLOAD_SIZE_MAX)*/256];

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
    SerialInPacketStart = 0;
    SerialInPacketLen = 0;
    SerialInPacketPtr = 0;
    CRSFframeActive = false;

    MspCallback = NULL;
    BattInfoCallback = NULL;
    GpsCallback = NULL;

    _dev->flush_read();
}

uint8_t *CRSF::ParseInByte(uint8_t inChar)
{
    uint8_t *packet_ptr = NULL;

    if (SerialInPacketPtr >= sizeof(SerialInBuffer))
    {
        // we reached the maximum allowable packet length,
        // so start again because shit fucked up hey.
        SerialInPacketPtr = 0;
        CRSFframeActive = false;
        BadPktsCount++;
    }

    // store byte
    SerialInBuffer[SerialInPacketPtr++] = inChar;

    // CRSF Frame:
    // | address | payload_len | payload* | crc |

    if (CRSFframeActive == false)
    {
        if ((inChar == CRSF_ADDRESS_CRSF_RECEIVER) ||
            (inChar == CRSF_ADDRESS_CRSF_TRANSMITTER) ||
            (inChar == CRSF_SYNC_BYTE))
        {
            crsf_cmd_ongoing = false;
            CRSFframeActive = true;
            SerialInPacketLen = 0;
#ifdef DBF_PIN_CRSF_PACKET
            gpio_out_write(crsf_packet, 1);
#endif
        }
        else
        {
            SerialInPacketPtr = 0;
        }
    }
    else
    {
        if (SerialInPacketLen == 0) // we read the packet length and save it
        {
            SerialInCrc = 0;
            SerialInPacketLen = inChar;
            SerialInPacketStart = SerialInPacketPtr;
            if ((SerialInPacketLen < 2) || (CRSF_FRAME_SIZE_MAX < SerialInPacketLen))
            {
                // failure -> discard
                CRSFframeActive = false;
                SerialInPacketPtr = 0;
                BadPktsCount++;
            }
        }
        else
        {
            uint32_t const current_len = SerialInPacketPtr - SerialInPacketStart;
            if (SerialInPacketLen <= current_len)
            {
                /* Check packet CRC */
                if (SerialInCrc == inChar)
                {
                    packet_ptr = &SerialInBuffer[SerialInPacketStart];
                    GoodPktsCount++;
                }
                else
                {
#if 0
                    // https://crccalc.com/
                    // CRC algorithm: CRC-8/DVB-S2
                    DEBUG_PRINTF("UART CRC %X != %X\n", SerialInCrc, inChar);
                    for (int i = (SerialInPacketStart-2); i < (SerialInPacketLen + 2); i++)
                    {
                        DEBUG_PRINTF(" 0x%X", SerialInBuffer[i]);
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
            }
            else
            {
                // Calc crc on the fly
#if PROTOCOL_CRSF_V3_TO_FC
                if ((inChar == CRSF_FRAMETYPE_COMMAND) &&
                    (current_len == 1)) {
                    crsf_cmd_ongoing = true;
                }
                uint8_t const poly = (crsf_cmd_ongoing) ? CRSF_CMD_POLY : CRSF_GEN_POLY;
                SerialInCrc = CalcCRC8(inChar, SerialInCrc, poly);
#else
                SerialInCrc = CalcCRC8(inChar, SerialInCrc, CRSF_GEN_POLY);
#endif
            }
        }
    }

    return packet_ptr;
}
