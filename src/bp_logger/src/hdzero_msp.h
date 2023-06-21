#pragma once

#include "msp_handler_base.h"

class HDZeroMsp : public MspHandlerBase
{
public:
    HDZeroMsp(CtrlSerial * serial) : MspHandlerBase(serial)
    {
    }
    ~HDZeroMsp()
    {
    }

    void init(void);

    void syncSettings(void);
    void syncSettings(AsyncWebSocketClient * const client);
    void syncSettings(AsyncEventSourceClient * const client);

    int parseSerialData(uint8_t chr);

    void vtxFrequencySet(uint16_t const freq, int8_t const power = -1, bool const disable_change = false);

    void OsdShowText(const char * text, uint32_t const timeout_ms = 0);

    void loop(void);

private:
    enum {
        /* Init states, keep first */
        STATE_GET_FW_VER,
        STATE_GET_CH_INDEX,
        STATE_GET_FREQ,
        STATE_GET_RECORDING,
        /* Running states */
        STATE_READY,
        STATE_CLEAR_OSD,
        STATE_DRAW_OSD,
    };

    uint32_t osd_timeout;
    uint8_t current_state;

    // From WEB UI
    int parseCommandPriv(char const * cmd, size_t len, AsyncWebSocketClient * const client);
    int parseCommandPriv(websoc_bin_hdr_t const * const cmd, size_t len, AsyncWebSocketClient * const client);
    int parseCommandPriv(mspPacket_t & msp_in);

    void sendMspToHdzero(uint8_t const * const buff,
                         uint16_t const len,
                         uint16_t const function,
                         bool const resp = false) const;

    void handleUserTextCommand(const char * input, size_t len, AsyncWebSocketClient * const client);
    void handleVtxFrequencyCommand(vtx_params_t params, AsyncWebSocketClient * const client);

    void handleRecordingStateCommand(uint8_t start) const;
    void handleBuzzerCommand(uint16_t time_ms) const;

    //
    void getFwVersion(void) const;
    void getChannelIndex(void) const;
    void getFrequency(void) const;
    void getRecordingState(void) const;

    void osdClear(void) const;
    void osdDraw(void);
    void osdText(char const * const p_text, size_t len, uint8_t row, uint8_t column);

    void handleLaptimerState(uint16_t const race_id,
                             uint16_t const round_num,
                             bool const state,
                             AsyncWebSocketClient * const client = NULL);
    void handleLaptimerLap(laptimer_lap_t const * lap, AsyncWebSocketClient * const client = NULL);

    virtual uint16_t osdRowMax(void) const
    {
        return 18; // VMAX
    }
    virtual uint16_t osdColumnMax(void) const
    {
        return 50; // HMAX
    }

    void checkFreqFromModule(uint16_t const freq) const;

    uint16_t getFreqByIndex(uint8_t index) const
    {
        if (index == ((3 * 8) + 1))
            return 5760; // F2
        if (index == ((3 * 8) + 3))
            return 5580; // F4
        index -= 4 * 8;
        if (index <= 7)
            return 5658U + (37U * index);
        return 0;
    }

    int8_t getIndexByFreq(uint16_t const freq) const
    {
        // MAP freq to HDZ supported channel indeces
        switch (freq) {
            case 5760: // F2
                return 3 * 8 + 1;
            case 5800: // F4
                return 3 * 8 + 3;
            case 5658: // R1
                return 4 * 8 + 0;
            case 5695: // R2
                return 4 * 8 + 1;
            case 5732: // R3
                return 4 * 8 + 2;
            case 5769: // R4
                return 4 * 8 + 3;
            case 5806: // R5
                return 4 * 8 + 4;
            case 5843: // R6
                return 4 * 8 + 5;
            case 5880: // R7
                return 4 * 8 + 6;
            case 5917: // R8
                return 4 * 8 + 7;
            default:
                break;
        }
        return -1;
    }
};
