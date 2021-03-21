#include "CRSF_RX.h"
#include "utils.h"
#include "debug_elrs.h"
#include <string.h>

crsf_channels_msg_t DMA_ATTR p_crsf_channels;
crsf_msp_packet_fc_t DMA_ATTR msp_packet;

#if PROTOCOL_ELRS_TO_FC
crsfLinkStatisticsMsg_elrs_t DMA_ATTR link_stat_packet;
#else // !PROTOCOL_ELRS_TO_FC
crsfLinkStatisticsMsg_t DMA_ATTR link_stat_packet;
#endif // PROTOCOL_ELRS_TO_FC

void CRSF_RX::Begin(void)
{
    link_stat_packet.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    link_stat_packet.header.frame_size = sizeof(link_stat_packet) - CRSF_FRAME_START_BYTES;
#if PROTOCOL_ELRS_TO_FC
    link_stat_packet.header.type = CRSF_FRAMETYPE_LINK_STATISTICS_ELRS;
#else // !PROTOCOL_ELRS_TO_FC
    link_stat_packet.header.type = CRSF_FRAMETYPE_LINK_STATISTICS;
#endif // PROTOCOL_ELRS_TO_FC

    msp_packet.header.device_addr = CRSF_ADDRESS_BROADCAST;
    msp_packet.header.frame_size = sizeof(msp_packet) - CRSF_FRAME_START_BYTES;
    msp_packet.header.type = CRSF_FRAMETYPE_MSP_WRITE;
    msp_packet.header.dest_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    msp_packet.header.orig_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;

    p_crsf_channels.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    p_crsf_channels.header.frame_size = sizeof(p_crsf_channels) - CRSF_FRAME_START_BYTES;
#if PROTOCOL_ELRS_TO_FC
    p_crsf_channels.header.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED_ELRS;
#else // !PROTOCOL_ELRS_TO_FC
    p_crsf_channels.header.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
#endif // PROTOCOL_ELRS_TO_FC

    CRSF::Begin();
}

void FAST_CODE_1 CRSF_RX::sendFrameToFC(uint8_t *buff, uint8_t size) const
{
    buff[size - 1] = CalcCRC(&buff[2], (buff[1] - 1));
#if !NO_DATA_TO_FC
    uint32_t irq = _SAVE_IRQ();
    _dev->write(buff, size);
    _RESTORE_IRQ(irq);
#endif
}

void CRSF_RX::LinkStatisticsSend(LinkStatsLink_t & stats) const
{
#if PROTOCOL_ELRS_TO_FC
    link_stat_packet.stats.uplink_RSSI = stats.uplink_RSSI_1;
    link_stat_packet.stats.uplink_Link_quality = stats.uplink_Link_quality;
    link_stat_packet.stats.uplink_SNR = stats.uplink_SNR;
    link_stat_packet.stats.rf_Mode = stats.rf_Mode;
#else // PROTOCOL_ELRS_TO_FC
    link_stat_packet.stats.uplink_RSSI_1 = stats.uplink_RSSI_1;
    link_stat_packet.stats.uplink_RSSI_2 = stats.uplink_RSSI_2;
    link_stat_packet.stats.uplink_Link_quality = stats.uplink_Link_quality;
    link_stat_packet.stats.uplink_SNR = stats.uplink_SNR;
    link_stat_packet.stats.active_antenna = stats.active_antenna;
    link_stat_packet.stats.rf_Mode = stats.rf_Mode;
    link_stat_packet.stats.uplink_TX_Power = stats.uplink_TX_Power;
    link_stat_packet.stats.downlink_RSSI = stats.downlink_RSSI;
    link_stat_packet.stats.downlink_Link_quality = stats.downlink_Link_quality;
    link_stat_packet.stats.downlink_SNR = stats.downlink_SNR;
#endif // PROTOCOL_ELRS_TO_FC
    sendFrameToFC((uint8_t*)&link_stat_packet, sizeof(link_stat_packet));
}

void FAST_CODE_1 CRSF_RX::sendRCFrameToFC(rc_channels_rx_t * channels) const
{
    memcpy(&p_crsf_channels.data, (void*)channels, sizeof(p_crsf_channels.data));
    sendFrameToFC((uint8_t*)&p_crsf_channels, sizeof(p_crsf_channels));
}

void FAST_CODE_1 CRSF_RX::sendMSPFrameToFC(mspPacket_t & msp) const
{
    uint8_t i;
    msp_packet.msp.flags = MSP_VERSION + msp.sequence_nbr;
    if (msp.payloadIterator == 0 && msp.sequence_nbr == 0) {
        msp_packet.msp.flags |= MSP_STARTFLAG;
    }
    msp.sequence_nbr++;
    for (i = 0; i < sizeof(msp_packet.msp.payload); i++) {
        msp_packet.msp.payload[i] = msp.readByte();
    }
    sendFrameToFC((uint8_t*)&msp_packet, sizeof(msp_packet));
}

void CRSF_RX::processPacket(uint8_t const *data)
{
    switch (data[0])
    {
        case CRSF_FRAMETYPE_COMMAND:
        {
            if (data[1] == 0x62 && data[2] == 0x6c)
            {
                DEBUG_PRINTF("Jumping to Bootloader...\n");
                delay(200);
                platform_restart();
            }
            break;
        }

        case CRSF_FRAMETYPE_BATTERY_SENSOR:
        {
            if (BattInfoCallback) {
                LinkStatsBatt_t batt;
                batt.voltage = data[1];
                batt.voltage <<= 8;
                batt.voltage += data[2];

                batt.current = data[3];
                batt.current <<= 8;
                batt.current += data[4];

                batt.capacity = data[5];
                batt.capacity <<= 8;
                batt.capacity += data[6];
                batt.capacity <<= 8;
                batt.capacity += data[7];

                batt.remaining = data[8];

                BattInfoCallback(&batt);
            }
            break;
        }

        case CRSF_FRAMETYPE_GPS:
        {
            if (GpsCallback) {
                GpsOta_t gps;
                gps.latitude = data[1];
                gps.latitude <<= 8;
                gps.latitude += data[2];
                gps.latitude <<= 8;
                gps.latitude += data[3];
                gps.latitude <<= 8;
                gps.latitude += data[4];

                gps.longitude = data[5];
                gps.longitude <<= 8;
                gps.longitude += data[6];
                gps.longitude <<= 8;
                gps.longitude += data[7];
                gps.longitude <<= 8;
                gps.longitude += data[8];

                gps.speed = data[9];
                gps.speed <<= 8;
                gps.speed += data[10];

                gps.heading = data[11];
                gps.heading <<= 8;
                gps.heading += data[12];

                gps.altitude = data[13];
                gps.altitude <<= 8;
                gps.altitude += data[14];

                gps.satellites = data[15];

                GpsCallback(&gps);
            }
            break;
        }

        case CRSF_FRAMETYPE_MSP_RESP:
        {
            if (MspCallback &&
                data[1] == CRSF_ADDRESS_RADIO_TRANSMITTER &&
                data[2] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
            {
                MspCallback(&data[3]); // pointer to MSP packet
            }
            break;
        }

        default:
            break;
    }
}

void CRSF_RX::handleUartIn(void)
{
    int available = _dev->available();
    uint8_t *ptr;

    if (16 < available) available = 16;
    else if (available < 0) available = 0;

    while (available--) {
        ptr = ParseInByte(_dev->read());
        if (ptr)
            processPacket(ptr);
    }
}
