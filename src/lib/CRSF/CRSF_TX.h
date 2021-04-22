#ifndef CRSF_TX_H_
#define CRSF_TX_H_

#include "CRSF.h"

/* OpenTX timer control sync packet */
typedef struct OpenTxSyncPacket_s {
    crsf_ext_header_s   header;
    uint8_t             otx_id;
    uint32_t            packetRate;
    uint32_t            offset;
    uint8_t             crc;
} PACKED OpenTxSyncPacket_t;

/* LUA packet is used only on TX */
#define ELRS_LUA_BUFF_SIZE 20U
typedef struct elrs_lua_packet_s
{
    crsf_ext_header_t header;
    uint8_t buffer[ELRS_LUA_BUFF_SIZE];
    uint8_t crc;
} PACKED elrs_lua_packet_t;


class CRSF_TX : public CRSF
{
public:
    CRSF_TX(HwSerial &dev);

    void Begin(void);

    // Handle incoming data
    uint8_t handleUartIn(void);

    // Send to RADIO
    void LinkStatisticsSend(LinkStatsLink_t & stats) const;
    void BatterySensorSend(LinkStatsBatt_t & stats) const;
    void GpsSensorSend(GpsOta_t & gps) const;
    void sendLUAresponseToRadio(uint8_t * const data, uint8_t len) const;
    void sendMspPacketToRadio(mspPacket_t &msp) const;

    // OpenTX Syncing
    void FAST_CODE_1 setRcPacketRate(uint32_t const interval)
    {
#if (FEATURE_OPENTX_SYNC)
        // Scale value to correct format
        RequestedRCpacketInterval = interval * 10;
#endif
    }

    void FAST_CODE_1 UpdateOpenTxSyncOffset(uint32_t const current_us)
    {
#if (FEATURE_OPENTX_SYNC)
        OpenTXsyncOffset = (int32_t)(current_us - RCdataLastRecv);
#endif
    }

    ///// Callbacks /////
    void (*connected)();
    void (*disconnected)();
    void (*RCdataCallback1)(uint8_t const *const channels);
    void (*ParamWriteCallback)(uint8_t const *msg, uint16_t len);

    ///// Variables /////

private:
    void uart_wdt(void);
    void processPacket(uint8_t const *input);
    void CrsfFramePushToFifo(uint8_t *buff, uint8_t size) const;
    void LinkStatisticsProcess(void) const;
    void BatteryStatisticsProcess(void) const;
    void GpsSensorProcess(void);
    void LuaResponseProcess(void) const;

    uint8_t TypeIsValid(uint8_t const type) const {
        switch (type) {
            /* TX */
            case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            case CRSF_FRAMETYPE_PARAMETER_WRITE:
            case CRSF_FRAMETYPE_MSP_REQ:
            case CRSF_FRAMETYPE_MSP_WRITE:
                return 1;
        }
        return 0;
    }

#if (FEATURE_OPENTX_SYNC)

#define OTX_SYNC_INTERVAL   200 // in ms
#define OTX_SYNC_ADVANCE    500 // 800 timing adcance in us

    uint32_t RCdataLastRecv;
    int32_t OpenTXsyncOffset;
    uint32_t RequestedRCpacketInterval;
    uint32_t OpenTXsynNextSend;
#endif /* FEATURE_OPENTX_SYNC */
    int sendSyncPacketToRadio();

    // for the UART wdt, every 1000ms we change bauds when connect is lost
#define UARTwdtInterval 1000
    uint32_t p_UartNextCheck;
    bool p_slowBaudrate;
    bool p_RadioConnected; // connected state
};

#endif /* CRSF_TX_H_ */
