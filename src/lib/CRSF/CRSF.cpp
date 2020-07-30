#include "CRSF.h"
#include "debug_elrs.h"

void rcNullCb(crsf_channels_t const *const) {}
void (*CRSF::RCdataCallback1)(crsf_channels_t const *const) = &rcNullCb; // function is called whenever there is new RC data.

void MspNullCallback(uint8_t const *const){};
void (*CRSF::MspCallback)(uint8_t const *const input) = MspNullCallback;

void nullCallback(void){};
void (*CRSF::disconnected)() = &nullCallback; // called when CRSF stream is lost
void (*CRSF::connected)() = &nullCallback;    // called when CRSF stream is regained

uint8_t DMA_ATTR outBuffer[CRSF_EXT_FRAME_SIZE(CRSF_PAYLOAD_SIZE_MAX)];

//#define DBF_PIN_CRSF_PACKET 2

void CRSF::Begin()
{
#ifdef DBF_PIN_CRSF_PACKET
    pinMode(DBF_PIN_CRSF_PACKET, OUTPUT);
    digitalWrite(DBF_PIN_CRSF_PACKET, 0);
#endif

    GoodPktsCount = 0;
    BadPktsCount = 0;
    SerialInPacketStart = 0;
    SerialInPacketLen = 0;
    SerialInPacketPtr = 0;
    CRSFframeActive = false;

    TLMbattSensor.capacity = 0;
    TLMbattSensor.current = 0;
    TLMbattSensor.voltage = 0;
    //TLMbattSensor.remaining = 100;

    _dev->flush_read();
}

void CRSF::LinkStatisticsExtract(volatile uint8_t const *const input,
                                 int8_t snr,
                                 uint8_t rssi)
{
    // NOTE: input is only 6 bytes!!

    LinkStatistics.downlink_SNR = snr * 10;
    LinkStatistics.downlink_RSSI = 120 + rssi;

    if (input[0] == CRSF_FRAMETYPE_LINK_STATISTICS)
    {
        LinkStatistics.uplink_RSSI_1 = input[1];
        LinkStatistics.uplink_RSSI_2 = 0;
        LinkStatistics.uplink_SNR = input[3];
        LinkStatistics.uplink_Link_quality = input[4];

        TLMbattSensor.voltage = ((uint16_t)input[2] << 8) + input[5];
    }
}

void ICACHE_RAM_ATTR CRSF::LinkStatisticsPack(uint8_t *const output)
{
    // NOTE: output is only 6 bytes!!

    output[0] = CRSF_FRAMETYPE_LINK_STATISTICS;

    // OpenTX hard codes "rssi" warnings to the LQ sensor for crossfire, so the
    // rssi we send is for display only.
    // OpenTX treats the rssi values as signed.
    uint8_t openTxRSSI = LinkStatistics.uplink_RSSI_1;
    // truncate the range to fit into OpenTX's 8 bit signed value
    if (openTxRSSI > 127)
        openTxRSSI = 127;
    // convert to 8 bit signed value in the negative range (-128 to 0)
    openTxRSSI = 255 - openTxRSSI;
    output[1] = openTxRSSI;
    output[2] = (TLMbattSensor.voltage & 0xFF00) >> 8;
    output[3] = LinkStatistics.uplink_SNR;
    output[4] = LinkStatistics.uplink_Link_quality;
    output[5] = (TLMbattSensor.voltage & 0x00FF);
}

void CRSF::BatterySensorSend(void)
{
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
        if (inChar == CRSF_ADDRESS_CRSF_RECEIVER ||
            inChar == CRSF_ADDRESS_CRSF_TRANSMITTER ||
            inChar == CRSF_SYNC_BYTE)
        {
            CRSFframeActive = true;
            SerialInPacketLen = 0;
#ifdef DBF_PIN_CRSF_PACKET
            digitalWrite(DBF_PIN_CRSF_PACKET, 1);
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
            if ((SerialInPacketPtr - SerialInPacketStart) >= (SerialInPacketLen))
            {
                //uint8_t *payload = &SerialInBuffer[SerialInPacketStart];
                //uint8_t CalculatedCRC = CalcCRC(payload, (SerialInPacketLen - 1));

                if (/*CalculatedCRC*/ SerialInCrc == inChar)
                {
                    //packet_ptr = payload;
                    packet_ptr = &SerialInBuffer[SerialInPacketStart];
                    GoodPktsCount++;
                }
                else
                {
#if 0
                    // https://crccalc.com/
                    // CRC algorithm: CRC-8/DVB-S2
                    DEBUG_PRINT("UART CRC failure ");
                    DEBUG_PRINT(SerialInCrc, HEX);
                    DEBUG_PRINT("!=");
                    DEBUG_PRINT(inChar, HEX);
                    for (int i = (SerialInPacketStart-2); i < (SerialInPacketLen + 2); i++)
                    {
                        DEBUG_PRINT(" 0x");
                        DEBUG_PRINT(SerialInBuffer[i], HEX);
                    }
                    DEBUG_PRINTLN();
#elif defined(DEBUG_SERIAL)
                    DEBUG_SERIAL.print("!C");
                    //DEBUG_SERIAL.write((uint8_t *)&SerialInBuffer[SerialInPacketStart - 2], SerialInPacketLen + 2);
#endif
                    BadPktsCount++;
                }

#ifdef DBF_PIN_CRSF_PACKET
                digitalWrite(DBF_PIN_CRSF_PACKET, 0);
#endif

                // packet handled, start next
                CRSFframeActive = false;
                SerialInPacketPtr = 0;
                SerialInPacketLen = 0;
            }
            else
            {
                // Calc crc on the fly
                SerialInCrc = CalcCRC(inChar, SerialInCrc);
            }
        }
    }

    return packet_ptr;
}
