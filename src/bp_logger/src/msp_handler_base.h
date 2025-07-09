#pragma once

#include "platform.h"
#include "msp.h"
#include "main.h"
#include "comm_espnow.h"
#include "storage.h"
#include <stdint.h>

#define VTX_CHANNEL_MAX 8

enum vtx_band_e {
    VTX_BAND_A = 0,
    VTX_BAND_B,
    VTX_BAND_E,
    VTX_BAND_F,
    VTX_BAND_R,
    VTX_BAND_L,
    VTX_BAND_MAX,
};

class MspHandlerBase
{
public:
    MspHandlerBase(CtrlSerial * serial) : _serial(serial)
    {
        _handler.markPacketFree();
        m_version_info = "";
        m_laptimer_state = false;
        m_recording_state = false;
    }
    ~MspHandlerBase()
    {
    }

    virtual void init(void)
    {
    }
    virtual void loop(void)
    {
    }
    virtual void printConnectionInfo(AsyncWebSocketClient * const client)
    {
    }

    //
    // Send settings to client (websocket or event)
    //
    virtual void syncSettings(void)
    {
    }
    virtual void syncSettings(AsyncWebSocketClient * const client)
    {
    }
    virtual void syncSettings(AsyncEventSourceClient * const client)
    {
    }

    //
    // Byte stream input to be checked for incoming MSP
    //
    virtual int parseSerialData(uint8_t chr)
    {
        return -1;
    }

    //
    // Message inputs from Web UI
    //
    int parseCommand(char const * cmd, size_t const len, AsyncWebSocketClient * const client)
    {
        return parseCommandPriv(cmd, len, client);
    }
    int parseCommand(websoc_bin_hdr_t const * const cmd, size_t const len, AsyncWebSocketClient * const client);
    int parseCommand(mspPacket_t & msp_in); // Handle received MSP packet

    //
    // VTX freq control
    //
    virtual void vtxFrequencySet(uint16_t const freq, bool const disable_change = false)
    {
        if (storeVtxFreq(NULL, freq, disable_change)) {
            handleVtxFrequencyCommand(freq, NULL);
            webUiSendVtxFrequency(freq);    // Update web UI
            espnow_vtxset_send(freq);       // inform other peers
        }
        m_vtx_set_disabled = disable_change;
    }
    bool vtxFreqChangeAllowed(void) const
    {
        return !m_vtx_set_disabled;
    }
    String vtxFreqGetBandChannel(uint16_t freq);

    //
    // OSD test
    //
    void OsdShowText(String & text, uint32_t const timeout_ms = 0)
    {
        OsdShowText(text.c_str(), timeout_ms);
    }
    virtual void OsdShowText(const char * text, uint32_t const timeout_ms = 0)
    {
        (void)text;
        (void)timeout_ms;
    };

    //
    // Laptimer control
    //
    void
    LaptimerStateStart(uint16_t const race_id, uint16_t const round_num, AsyncWebSocketClient * const client = NULL)
    {
        m_laptimer_state = true;
        webUiSendLaptimerState(race_id, round_num, true, client);
        handleLaptimerState(race_id, round_num, true, client);
    }
    void LaptimerStateStop(uint16_t const race_id, uint16_t const round_num, AsyncWebSocketClient * const client = NULL)
    {
        m_laptimer_state = false;
        webUiSendLaptimerState(race_id, round_num, false, client);
        handleLaptimerState(race_id, round_num, false, client);
    }
    void LaptimerLap(laptimer_lap_t const * lap, AsyncWebSocketClient * const client = NULL)
    {
        webUiSendLaptimerLap(lap, client);
        handleLaptimerLap(lap, client);
    }
    bool LaptimerStateGet(void) const
    {
        return m_laptimer_state;
    }
    bool RecordingStateGet(void) const
    {
        return m_recording_state;
    }

protected:
    CtrlSerial * _serial;

    MSP _handler;
    mspPacket_t msp_out;

    String m_version_info;

    bool m_vtx_set_disabled;
    bool m_laptimer_state;
    bool m_recording_state;

    const uint16_t frequency_table[VTX_BAND_MAX][VTX_CHANNEL_MAX] = {
        {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, // A
        {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, // B
        {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, // E
        {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, // F / Airwave
        {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, // R / Immersion Raceband
        {5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621}, // L
    };

    virtual int parseCommandPriv(char const * cmd, size_t len, AsyncWebSocketClient * const client) = 0;
    virtual int
    parseCommandPriv(websoc_bin_hdr_t const * const cmd, size_t len, AsyncWebSocketClient * const client) = 0;
    virtual int parseCommandPriv(mspPacket_t & msp_in) = 0;

    virtual void handleVtxFrequencyCommand(uint16_t const freq, AsyncWebSocketClient * const client) = 0;

    virtual void handleLaptimerState(uint16_t const race_id,
                                     uint16_t const round_num,
                                     bool const state,
                                     AsyncWebSocketClient * const client = NULL){};
    virtual void handleLaptimerLap(laptimer_lap_t const * lap, AsyncWebSocketClient * const client = NULL){};

    virtual uint16_t osdRowMax(void) const
    {
        return 0;
    }
    virtual uint16_t osdColumnMax(void) const
    {
        return 0;
    }

    void webUiSendVtxFrequency(uint16_t const freq, AsyncWebSocketClient * const client = NULL) const;
    void webUiSendVRecordingState(uint8_t const state, AsyncWebSocketClient * const client = NULL) const;
    void webUiSendLaptimerState(uint16_t const race_id,
                                uint16_t const round_num,
                                bool const state,
                                AsyncWebSocketClient * const client = NULL) const;
    void webUiSendLaptimerLap(laptimer_lap_t const * lap, AsyncWebSocketClient * const client = NULL) const;

    uint16_t parseFreq(uint8_t const * const payload) const
    {
        uint16_t const freq = ((uint16_t)payload[1] << 8) + payload[0];
        if (freq < (VTX_BAND_MAX * VTX_CHANNEL_MAX)) {
            uint8_t const _band = freq / VTX_CHANNEL_MAX;
            uint8_t const _index = freq % VTX_CHANNEL_MAX;
            return frequency_table[_band][_index];
        }
        return freq;
    }

    uint16_t getFreqByIndex(uint8_t const index) const
    {
        if (index >= (VTX_BAND_MAX * VTX_CHANNEL_MAX)) {
            return 0; // Invalid index
        }
        uint16_t const * const p_freq = &frequency_table[0][0];
        return p_freq[index];
    }

    int8_t getIndexByFreq(uint16_t const freq) const
    {
        if (freq < (VTX_BAND_MAX * VTX_CHANNEL_MAX)) {
            // freq is already an index
            return freq;
        }
        if (5000 <= freq && freq <= 6000) {
            // Lookup a correct index
            uint16_t const * const p_freq = &frequency_table[0][0];
            for (uint8_t iter = 0; iter < (VTX_BAND_MAX * VTX_CHANNEL_MAX); iter++) {
                if (freq == p_freq[iter]) {
                    return iter;
                }
            }
        }
        return -1;
    }

    bool storeVtxFreq(AsyncWebSocketClient * const client, uint16_t const freq, bool const force = false) const
    {
        if (!force && !vtxFreqChangeAllowed()) {
            websocket_send_txt("VTX change is not allowed!", client);
            return false;
        }

        String dbg_info = "Invalid VTX freq received!";
        if (freq == 0) {
            websocket_send_txt(dbg_info, client);
            return false;
        }

        bool const change_freq = eeprom_storage.vtx_freq != freq;
        if (change_freq) {
            dbg_info = "Setting vtx freq to: ";
            dbg_info += freq;
            dbg_info += "MHz";
            websocket_send_txt(dbg_info, client);

            eeprom_storage.vtx_freq = freq;
            eeprom_storage.markDirty();
        }
        return change_freq;
    }

    typedef struct laptime {
        uint16_t ms;
        uint8_t s;
        uint8_t m;
    } lap_time_t;

    lap_time_t convert_ms_to_time(uint32_t const lap_time) const
    {
        uint32_t secs = lap_time / 1000;
        uint16_t const ms = lap_time % 1000;
        uint8_t const hours = secs / 3600;
        uint8_t const mins = (secs - (hours * 3600)) / 60;
        secs = secs - (hours * 3600) - (mins * 60);
        return (lap_time_t){.ms = ms, .s = (uint8_t)secs, .m = mins};
    }

    uint32_t convert_time_to_ms(lap_time_t const time) const
    {
        uint32_t ms = time.ms;
        ms += 1000LU * time.s;
        ms += 60000LU * time.m;
        return ms;
    }

private:
};
