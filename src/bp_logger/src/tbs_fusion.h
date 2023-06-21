#pragma once

#include "msp_handler_base.h"

class TbsFusion : public MspHandlerBase
{
public:
    TbsFusion(CtrlSerial * serial) : MspHandlerBase(serial)
    {
        fetch_dev_info = false;
    }
    ~TbsFusion()
    {
    }

    void init(void);

    void syncSettings(void);
    void syncSettings(AsyncWebSocketClient * const client);
    void syncSettings(AsyncEventSourceClient * const client);

    int parseSerialData(uint8_t chr);

    void loop(void);

private:
    enum {
        STATE_GET_CH_INDEX,
        STATE_GET_FREQ,
        STATE_GET_RECORDING,
        STATE_READY,
    };

    bool fetch_dev_info;

    // From WEB UI
    int parseCommandPriv(char const * cmd, size_t len, AsyncWebSocketClient * const client);
    int parseCommandPriv(websoc_bin_hdr_t const * const cmd, size_t len, AsyncWebSocketClient * const client);
    int parseCommandPriv(mspPacket_t & msp_in);

    void sendMspToUart(uint8_t const * const buff, uint16_t const len, uint16_t const function);
    void CrsfWrite(uint8_t * const buff, size_t size) const;
    void CrsfWriteCommand(uint8_t * const buff, size_t size) const;

    void handleUserTextCommand(const char * input, size_t len, AsyncWebSocketClient * const client);
    void handleVtxFrequencyCommand(vtx_params_t params, AsyncWebSocketClient * const client);
    void handleLaptimerLap(laptimer_lap_t const * lap, AsyncWebSocketClient * const client = NULL);
    void handleTelemetryGps(uint8_t const * const payload);
    void handleTelemetryLinkStats(uint8_t const * const payload);
    void handleTelemetryBattery(uint8_t const * const payload);
    void pingSend(void) const;

    void param_read_send(uint8_t field_id, uint8_t junk) const;
    bool param_entry_send(uint8_t param_index, uint8_t dest_addr, const char * info_msg = NULL);
};
