#pragma once

#include "msp_handler_base.h"

class ExpresslrsMsp : public MspHandlerBase
{
public:
    ExpresslrsMsp(CtrlSerial * serial) : MspHandlerBase(serial)
    {
        vtx_power = -1;
    }
    ~ExpresslrsMsp()
    {
    }

    void init(void);
    void printConnectionInfo(AsyncWebSocketClient * const client);

    void syncSettings(void);
    void syncSettings(AsyncWebSocketClient * const client);
    void syncSettings(AsyncEventSourceClient * const client);

    int parseSerialData(uint8_t const chr);

    void loop(void);

private:
    volatile uint32_t msp_send_save_requested_ms;

    uint8_t settings_rate;
    uint8_t settings_power, settings_power_max;
    uint8_t settings_tlm;
    uint8_t settings_region;
    uint8_t settings_valid;

    int8_t vtx_power;

#if CONFIG_HANDSET
    /* Handset specific data */
    struct gimbal_limit gimbals[TX_NUM_ANALOGS];
    struct mixer mixer[TX_NUM_MIXER];
    uint8_t handset_num_switches;
    uint8_t handset_num_aux;
    uint8_t handset_mixer_ok;
    uint8_t handset_adjust_ok;
#endif

    // From WEB UI
    int parseCommandPriv(char const * cmd, size_t len, AsyncWebSocketClient * const client);
    int parseCommandPriv(websoc_bin_hdr_t const * const cmd, size_t len, AsyncWebSocketClient * const client);
    int parseCommandPriv(mspPacket_t & msp_in);

    void handleVtxFrequencyCommand(vtx_params_t params, AsyncWebSocketClient * const client);

    void elrsSettingsSend(uint8_t const * buff, uint8_t len);
    void elrsSendMsp(uint8_t const * buff, uint8_t len, uint8_t function);

    void elrsSettingsLoad(void);
    void elrsSettingsSendToWebsocket(AsyncWebSocketClient * const client = NULL);
    void elrsSettingsSendEvent(AsyncEventSourceClient * const client = NULL);

#if CONFIG_HANDSET
    void handleHandsetCalibrateCommand(uint8_t const * const input);
    void handleHandsetCalibrateResp(uint8_t const * data, AsyncWebSocketClient * const = NULL);
    void handleHandsetMixerCommand(uint8_t const * const input, size_t length);
    void handleHandsetMixerResp(uint8_t const * data, AsyncWebSocketClient * const client = NULL);
    void handleHandsetAdjustCommand(uint8_t const * const input);
    void handleHandsetAdjustResp(uint8_t const * data, AsyncWebSocketClient * const client = NULL);
    void handsetConfigLoad(void);
    void handsetConfigGet(AsyncWebSocketClient * const client, uint8_t force = 0);
    void handsetConfigSave(void);
    void handleHandsetTlmLnkStatsAndBatt(uint8_t const * const data);
    void handleHandsetTlmGps(uint8_t const * const data);

    void handleBatteryVoltageReport(AsyncWebSocketClient * const client = NULL);
    void handleBatteryVoltageCommand(uint8_t scale, uint8_t warning);
    void batteryVoltageInit(void);
    void batteryVoltageMeasure(void);
#endif
};
