#include "CRSF_TX.h"

#if !BACKPACK_LOGGER_BUILD

#include "platform.h"
#include "debug_elrs.h"
#include <string.h>

//#define DBF_PIN_CRSF_BYTES_IN 4
#ifdef DBF_PIN_CRSF_BYTES_IN
struct gpio_out crsf_byte_in;
#endif

#define OPENTX_SYNC_SCALE 10

static void rcNullCb(uint8_t const *const) {}
static void nullCallback(void){};

enum {
    SEND_NA = 0,
    SEND_LNK_STAT = 1 << 0,
    SEND_BATT = 1 << 1,
    SEND_LUA = 1 << 2,
    SEND_MSP = 1 << 3,
    SEND_SYNC = 1 << 4,
    SEND_GPS = 1 << 5,
};
volatile uint_fast8_t DMA_ATTR send_buffers;

elrs_lua_packet_t DMA_ATTR p_lua_packet;
OpenTxSyncPacket_s DMA_ATTR p_otx_sync_packet;
crsf_msp_packet_radio_s DMA_ATTR p_msp_packet;
crsfLinkStatisticsMsg_t DMA_ATTR link_stat_packet;
crsf_sensor_battery_s DMA_ATTR TLMbattSensor;
crsf_sensor_gps_s DMA_ATTR TLMGPSsensor;

uint32_t DMA_ATTR availableHandsetBauds[] = {
    CRSF_TX_BAUDRATE_FAST, CRSF_TX_BAUDRATE_SLOW,
    // Enable higher baudrates
    5250000, 3750000, 1870000, 921600};


CRSF_TX::CRSF_TX(HwSerial &dev) : CRSF(&dev)
{
    setRcPacketRate(5000); // default to 200hz as per 'normal'
    /* Init callback funcs */
    connected = nullCallback;
    disconnected = nullCallback;
    RCdataCallback1 = rcNullCb;
    ParamWriteCallback = NULL;
}

void CRSF_TX::Begin(void)
{
#ifdef DBF_PIN_CRSF_BYTES_IN
    crsf_byte_in = gpio_out_setup(DBF_PIN_CRSF_BYTES_IN, 0);
#endif

    /* Initialize used messages */

    link_stat_packet.header.device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    link_stat_packet.header.frame_size = sizeof(link_stat_packet) - CRSF_FRAME_START_BYTES;
    link_stat_packet.header.type = CRSF_FRAMETYPE_LINK_STATISTICS;

    TLMbattSensor.header.device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    TLMbattSensor.header.frame_size = sizeof(TLMbattSensor) - CRSF_FRAME_START_BYTES;
    TLMbattSensor.header.type = CRSF_FRAMETYPE_BATTERY_SENSOR;

    TLMGPSsensor.header.device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    TLMGPSsensor.header.frame_size = sizeof(TLMGPSsensor) - CRSF_FRAME_START_BYTES;
    TLMGPSsensor.header.type = CRSF_FRAMETYPE_GPS;

    p_lua_packet.header.device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    p_lua_packet.header.frame_size = 0; //sizeof(p_lua_packet) - CRSF_FRAME_START_BYTES;
    p_lua_packet.header.type = CRSF_FRAMETYPE_PARAMETER_WRITE;
    p_lua_packet.header.dest_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    p_lua_packet.header.orig_addr = CRSF_ADDRESS_CRSF_TRANSMITTER;

    p_otx_sync_packet.header.device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    p_otx_sync_packet.header.frame_size = sizeof(p_otx_sync_packet) - CRSF_FRAME_START_BYTES;
    p_otx_sync_packet.header.type = CRSF_FRAMETYPE_RADIO_ID;
    p_otx_sync_packet.header.dest_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    p_otx_sync_packet.header.orig_addr = CRSF_ADDRESS_CRSF_TRANSMITTER;
    p_otx_sync_packet.otx_id = CRSF_FRAMETYPE_OPENTX_SYNC;

    p_msp_packet.header.device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    p_msp_packet.header.frame_size = sizeof(p_msp_packet) - CRSF_FRAME_START_BYTES;
    p_msp_packet.header.type = CRSF_FRAMETYPE_MSP_RESP;
    p_msp_packet.header.dest_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    p_msp_packet.header.orig_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;

    CRSF::Begin();
    p_UartNextCheck = millis();
}

void FAST_CODE_2 CRSF_TX::CrsfFramePushToFifo(uint8_t *buff, uint8_t size) const
{
    buff[size - 1] = CalcCRC(&buff[2], (buff[1] - 1));
    _dev->write(buff, size);
    platform_wd_feed();
#if defined(BT_SERIAL)
    // Send to BT serial
    BT_SERIAL.write(buff, size);
    platform_wd_feed();
#endif // BT_SERIAL
}

void CRSF_TX::LinkStatisticsSend(LinkStatsLink_t & stats) const
{
    send_buffers |= SEND_LNK_STAT;
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
}
void CRSF_TX::LinkStatisticsProcess(void) const
{
    send_buffers &= ~SEND_LNK_STAT;
    CrsfFramePushToFifo((uint8_t*)&link_stat_packet, sizeof(link_stat_packet));
}

void CRSF_TX::BatterySensorSend(LinkStatsBatt_t & stats) const
{
    send_buffers |= SEND_BATT;
    TLMbattSensor.voltage = stats.voltage;
    TLMbattSensor.current = stats.current;
    TLMbattSensor.capacity = stats.capacity;
    TLMbattSensor.remaining = stats.remaining;
}
void CRSF_TX::BatteryStatisticsProcess(void) const
{
    send_buffers &= ~SEND_BATT;
    CrsfFramePushToFifo((uint8_t*)&TLMbattSensor, sizeof(TLMbattSensor));
}

void CRSF_TX::GpsSensorSend(GpsOta_t & gps) const
{
    if (!gps.pkt_cnt)
        return;
    send_buffers |= SEND_GPS;
    TLMGPSsensor.latitude = gps.latitude;
    TLMGPSsensor.longitude = gps.longitude;
    TLMGPSsensor.speed = gps.speed;
    TLMGPSsensor.heading = gps.heading;
    TLMGPSsensor.altitude = gps.altitude;
    TLMGPSsensor.satellites = gps.satellites;
    gps.pkt_cnt = 0;
}
void CRSF_TX::GpsSensorProcess(void)
{
    send_buffers &= ~SEND_GPS;
    CrsfFramePushToFifo((uint8_t *)&TLMGPSsensor, sizeof(TLMGPSsensor));
}

void CRSF_TX::sendLUAresponseToRadio(uint8_t * const data, uint8_t const size) const
{
    if (sizeof(p_lua_packet.buffer) < size)
        return;
    memcpy(p_lua_packet.buffer, data, size);
    p_lua_packet.header.frame_size = size;
    send_buffers |= SEND_LUA;
}
void CRSF_TX::LuaResponseProcess(void) const
{
    send_buffers &= ~SEND_LUA;
    // size = hdr + crc + payload - start_bytes
    uint8_t size = sizeof(crsf_ext_header_t) + 1 + p_lua_packet.header.frame_size;
    p_lua_packet.header.frame_size = size - CRSF_FRAME_START_BYTES;
    CrsfFramePushToFifo((uint8_t*)&p_lua_packet, size);
}

void CRSF_TX::sendMspPacketToRadio(mspPacket_t &msp) const
{
    if (!p_RadioConnected)
        return;
    uint8_t * p_dst = p_msp_packet.msp.payload;
    size_t iter;
    p_msp_packet.msp.flags = MSP_VERSION + (msp.sequence_nbr & MSP_SEQUENCE_MASK);
    if (msp.payloadIterator == 0 && msp.sequence_nbr == 0) {
        p_msp_packet.msp.flags |= MSP_STARTFLAG;
        p_msp_packet.msp.hdr.payloadSize = msp.payloadSize;
        p_msp_packet.msp.hdr.function = msp.function;
        p_dst = p_msp_packet.msp.hdr.payload;
    }
    msp.sequence_nbr++;
    for (iter = 0; iter < sizeof(p_msp_packet.msp.payload) && !msp.iterated(); iter++) {
        p_dst[iter] = msp.readByte();
    }
    CrsfFramePushToFifo((uint8_t*)&p_msp_packet, sizeof(p_msp_packet));
}

void CRSF_TX::sendSyncPacketToRadio(void)
{
#if (FEATURE_OPENTX_SYNC)
    if (!RCdataLastRecv || !p_RadioConnected)
    {
        return;
    }

    uint32_t const current = millis();
    // Adjust radio timing if not in requested window or not sent within 200ms
    if ((int32_t)(current - OpenTXsynNextSend) < OTX_SYNC_INTERVAL)
    {
        return;
    }

    int32_t const offset = (int32_t)(OpenTXsyncOffset - OTX_SYNC_ADVANCE);
    p_otx_sync_packet.offset = __builtin_bswap32(offset * OPENTX_SYNC_SCALE);
    CrsfFramePushToFifo((uint8_t*)&p_otx_sync_packet, sizeof(p_otx_sync_packet));

    //DEBUG_PRINTF("Sync: int=%u,off=%d\n", p_otx_sync_packet.packetRate, offset);

    OpenTXsynNextSend = current;
#endif /* FEATURE_OPENTX_SYNC */
    return;
}

void CRSF_TX::setRcPacketRate(uint32_t const interval) const
{
#if (FEATURE_OPENTX_SYNC)
    // Update packet rate and scale value to correct format
    p_otx_sync_packet.packetRate = __builtin_bswap32(interval * OPENTX_SYNC_SCALE);
#endif
}

void CRSF_TX::processPacket(uint8_t const *input)
{
    if (p_RadioConnected == false)
    {
        p_RadioConnected = true;
#if (FEATURE_OPENTX_SYNC)
        write_u32(&RCdataLastRecv, 0);
        OpenTXsynNextSend = millis();
#endif
        DEBUG_PRINTF("CRSF Connected.\n");
        connected();
    }

    switch (input[0]) // check CRSF command
    {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
        {
#if (FEATURE_OPENTX_SYNC)
            write_u32(&RCdataLastRecv, micros());
#endif
            (RCdataCallback1)(&input[1]); // run new RC data callback
            break;
        }
        case CRSF_FRAMETYPE_PARAMETER_WRITE:
        {
            if (input[1] == CRSF_ADDRESS_CRSF_TRANSMITTER &&
                input[2] == CRSF_ADDRESS_RADIO_TRANSMITTER)
            {
                if (ParamWriteCallback)
                    ParamWriteCallback(&input[3], 2);
            }
            break;
        }
        case CRSF_FRAMETYPE_MSP_REQ:
        case CRSF_FRAMETYPE_MSP_WRITE:
        {
            if (input[1] == CRSF_ADDRESS_FLIGHT_CONTROLLER &&
                input[2] == CRSF_ADDRESS_RADIO_TRANSMITTER)
            {
                if (MspCallback)
                    MspCallback(&input[3]); // pointer to MSP packet
            }
            break;
        }
        default:
            break;
    };
}

uint8_t CRSF_TX::handleUartIn(void)
{
    int available = _dev->available();
    uint8_t *ptr;
    uint8_t can_send = 0;

    if (16 < available) available = 16;
    else if (available < 0) available = 0;

    while (available--) {
#ifdef DBF_PIN_CRSF_BYTES_IN
        gpio_out_write(crsf_byte_in, 1);
#endif
        ptr = ParseInByte(_dev->read());
        if (ptr) {
            processPacket(ptr);
            platform_wd_feed();

            /* Can write right after successful package reception */
            sendSyncPacketToRadio();

            if (send_buffers & SEND_LNK_STAT)
                LinkStatisticsProcess();
            else if (send_buffers & SEND_BATT)
                BatteryStatisticsProcess();
            else if (send_buffers & SEND_LUA)
                LuaResponseProcess();
            else if (send_buffers & SEND_GPS)
                GpsSensorProcess();
            else
                can_send = 1;
        }

#ifdef DBF_PIN_CRSF_BYTES_IN
        gpio_out_write(crsf_byte_in, 0);
#endif
    }

    if (!can_send)
        uart_wdt();

    return can_send;
}

void CRSF_TX::uart_wdt(void)
{
    uint32_t now = millis();
    if ((int32_t)(now - p_UartNextCheck) < UARTwdtInterval)
    {
        // No need to check result yet
        return;
    }

    DEBUG_PRINTF("CRSF Bad:Good %u:%u\n", BadPktsCount, GoodPktsCount);

    if (BadPktsCount >= GoodPktsCount)
    {
        if (p_RadioConnected == true)
        {
            DEBUG_PRINTF("CRSF UART Disconnect. ");
            disconnected();
            p_RadioConnected = false;
        }

        _dev->end();
        platform_wd_feed();

        p_baudrateIdx = (p_baudrateIdx + 1) % ARRAY_SIZE(availableHandsetBauds);
        _dev->Begin(availableHandsetBauds[p_baudrateIdx]);
        DEBUG_PRINTF("Switched to baudrate %u\n", availableHandsetBauds[p_baudrateIdx]);
    }

    p_UartNextCheck = now;
    BadPktsCount = 0;
    GoodPktsCount = 0;
}

#endif // BACKPACK_LOGGER_BUILD
