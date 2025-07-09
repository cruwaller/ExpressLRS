#include "common.h"
#include "platform.h"
#include "utils.h"
#include "FHSS.h"
#include "crc.h"
#include "rc_channels.h"
#include "targets.h"
#include "helpers.h"
#include "debug_elrs.h"
#if TARGET_HANDSET && OTA_VANILLA_ENABLED
#include "OTAvanilla.h"
#endif
#if RADIO_SX127x
#include "LoRa_SX127x.h"
#endif
#if RADIO_SX128x
#include "SX1280.h"
#endif

#if RADIO_RX_BUFFER_SIZE < OTA_PAYLOAD_MAX
#error "Radio RX data buffer is too small!"
#endif


uint8_t current_rate_config;
const expresslrs_mod_settings_t *ExpressLRS_currAirRate;

static expresslrs_mod_settings_t * current_settings;

/* clang-format off */
//
// https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/2R000000HUhK/6T9Vdb3_ldnElA8drIbPYjs1wBbhlWUXej8ZMXtZXOM
//
#if RADIO_SX128x
#if RADIO_SX128x_FLRC
static expresslrs_mod_settings_t DRAM_FORCE_ATTR ExpressLRS_AirRateConfig_128x_FLRC[] = {
#if TARGET_HANDSET || RX_MODULE
    /* 1000Hz = single tx */
    {RADIO_FLRC, SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2, 1000, 150000u, 1000, 32, 1000, 1200, TLM_RATIO_NO_TLM, FHSS_2, OSD_MODE_1kHz_FLRC,  OTA_PAYLOAD_SX128x, 1, HWCRC_EN}, // 0.3642ms
#endif
    /*  500Hz = double tx @1kHz */
    {RADIO_FLRC, SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2, 1000, 150000u,  500, 32, 1000, 1200, TLM_RATIO_NO_TLM, FHSS_1, OSD_MODE_500Hz_FLRC, OTA_PAYLOAD_SX128x, 2, HWCRC_EN}, // 0.3642ms
    /*  250Hz = quad tx @1kHz */
    {RADIO_FLRC, SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2, 1000, 150000u,  250, 32, 1000, 1200, TLM_RATIO_NO_TLM, FHSS_2, OSD_MODE_250Hz_FLRC, OTA_PAYLOAD_SX128x, 4, HWCRC_EN}, // 0.3642ms
};
#endif

static expresslrs_mod_settings_t DRAM_FORCE_ATTR ExpressLRS_AirRateConfig_128x[] = {
    // NOTE! Preamble len is calculate MANT*2^EXP when MANT is bits [3:0] and EXP is bits [7:4]
#if RADIO_SX128x_BW800
    /* 500Hz */
    {RADIO_LORA, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6,  2000, 250000u, 500, 0b01100 /*12*/, 1000, 1200, TLM_RATIO_NO_TLM, FHSS_2, OSD_MODE_500Hz, OTA_PAYLOAD_SX128x, 1, HWCRC_DIS}, // 1.51ms
    /* 250Hz */
    {RADIO_LORA, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7,  4000, 250000u, 250, 0b01110 /*14*/, 1000, 1500, TLM_RATIO_1_64,   FHSS_2, OSD_MODE_250Hz, OTA_PAYLOAD_SX128x, 1, HWCRC_DIS}, // 3.33ms
    /* 125Hz */
    {RADIO_LORA, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7,  8000, 500000u, 125, 0b11001 /*18*/, 1000, 1500, TLM_RATIO_1_32,   FHSS_1, OSD_MODE_125Hz, OTA_PAYLOAD_SX128x, 1, HWCRC_DIS}, // 6.82ms
    /* 50Hz */
    {RADIO_LORA, SX1280_LORA_BW_0800, SX1280_LORA_SF8, SX1280_LORA_CR_LI_4_7, 20000, 750000u,  50, 0b11010 /*20*/, 1000, 2000, TLM_RATIO_1_16,   FHSS_1, OSD_MODE_50Hz,  OTA_PAYLOAD_SX128x, 1, HWCRC_DIS}, // 13.32ms
#else // 1600MHz BW
    /* 500Hz */
    {RADIO_LORA, SX1280_LORA_BW_1600, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_6,  2000, 250000u, 500, 0b01100 /*12*/, 1000, 1500, TLM_RATIO_1_128, FHSS_2, OSD_MODE_500Hz, OTA_PAYLOAD_SX128x, 1, HWCRC_DIS}, // 1.35ms
    /* 250Hz */
    {RADIO_LORA, SX1280_LORA_BW_1600, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7,  4000, 250000u, 250, 0b01110 /*14*/, 1000, 1500, TLM_RATIO_1_64,  FHSS_2, OSD_MODE_250Hz, OTA_PAYLOAD_SX128x, 1, HWCRC_DIS}, // 3.10ms
    /* 125Hz */
    {RADIO_LORA, SX1280_LORA_BW_1600, SX1280_LORA_SF8, SX1280_LORA_CR_LI_4_7,  8000, 500000u, 125, 0b11010 /*20*/, 1000, 2000, TLM_RATIO_1_32,  FHSS_1, OSD_MODE_125Hz, OTA_PAYLOAD_SX128x, 1, HWCRC_DIS},
    /* 50Hz */
    {RADIO_LORA, SX1280_LORA_BW_1600, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_7, 20000, 750000u,  50, 0b11010 /*20*/, 1000, 2500, TLM_RATIO_1_16,  FHSS_1, OSD_MODE_50Hz,  OTA_PAYLOAD_SX128x, 1, HWCRC_DIS},
#endif // RADIO_SX128x_BW800
};

#if TARGET_HANDSET && OTA_VANILLA_ENABLED
static expresslrs_mod_settings_t DRAM_FORCE_ATTR ExpressLRS_AirRateConfig_128x_VANILLA[] = {
    // Adjust \ref OTA_vanilla_SyncPacketData accordingly for index mapping
    // FLRC - DVDA only
    {RADIO_FLRC_VANILLA, SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2, 1000, 60000u, 500, 32, 1000, 1200, TLM_RATIO_NO_TLM, FHSS_2, OSD_MODE_500Hz_FLRC, OTA_VANILLA_SIZE, 2, HWCRC_EN},
    {RADIO_FLRC_VANILLA, SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2, 1000, 60000u, 250, 32, 1000, 1200, TLM_RATIO_NO_TLM, FHSS_2, OSD_MODE_250Hz_FLRC, OTA_VANILLA_SIZE, 4, HWCRC_EN},
    // LoRa
    {RADIO_LORA, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, 2000, 100000u, 500, 12, 1000, 1200, TLM_RATIO_NO_TLM, FHSS_4, OSD_MODE_500Hz, OTA_VANILLA_SIZE, 1, HWCRC_IGNORE},
    {RADIO_LORA, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, 4000, 100000u, 250, 14, 1000, 1500, TLM_RATIO_NO_TLM, FHSS_4, OSD_MODE_250Hz, OTA_VANILLA_SIZE, 1, HWCRC_IGNORE},
};
#endif // OTA_VANILLA_ENABLED

#endif /* RADIO_SX128x */

#if RADIO_SX127x
static expresslrs_mod_settings_t DRAM_FORCE_ATTR ExpressLRS_AirRateConfig_127x[] = {
    /* 200Hz */
    {RADIO_LORA, SX127X_BW_500_00_KHZ,  SX127X_SF_6, SX127X_CR_4_7,   5000, 250000u, 200, 8, 1000, 1000, TLM_RATIO_1_64, FHSS_1, OSD_MODE_200Hz, OTA_PAYLOAD_SX127x, 1, HWCRC_DIS}, // 4.38ms
    /* 100Hz */
    {RADIO_LORA, SX127X_BW_500_00_KHZ,  SX127X_SF_7, SX127X_CR_4_8,  10000, 500000u, 100, 8, 1000, 1500, TLM_RATIO_1_32, FHSS_1, OSD_MODE_100Hz, OTA_PAYLOAD_SX127x, 1, HWCRC_DIS}, // 9.28ms
    /* 50Hz */
    {RADIO_LORA, SX127X_BW_500_00_KHZ,  SX127X_SF_8, SX127X_CR_4_8,  20000, 750000u,  50, 8, 1000, 2000, TLM_RATIO_1_16, FHSS_1, OSD_MODE_50Hz,  OTA_PAYLOAD_SX127x, 1, HWCRC_DIS}, // 18.56ms
};
#endif /* RADIO_SX127x */
/* clang-format on */

static expresslrs_mod_settings_t* get_air_rate_config(uint8_t const type)
{
    switch (type) {
#if RADIO_SX127x
        case RADIO_TYPE_127x:
            return ExpressLRS_AirRateConfig_127x;
#endif
#if RADIO_SX128x
        case RADIO_TYPE_128x:
            return ExpressLRS_AirRateConfig_128x;
#if RADIO_SX128x_FLRC
        case RADIO_TYPE_128x_FLRC:
            return ExpressLRS_AirRateConfig_128x_FLRC;
#endif
#if TARGET_HANDSET && OTA_VANILLA_ENABLED
        case RADIO_TYPE_128x_VANILLA_v3:
        case RADIO_TYPE_128x_VANILLA_v4:
            return ExpressLRS_AirRateConfig_128x_VANILLA;
#endif
#endif
        default:
            return NULL;
    }
}

static uint8_t get_elrs_airRateMaxByConfig(expresslrs_mod_settings_t const * const cfg)
{
#if RADIO_SX127x
    if (cfg == ExpressLRS_AirRateConfig_127x)
        return ARRAY_SIZE(ExpressLRS_AirRateConfig_127x);
#endif
#if RADIO_SX128x
    if (cfg == ExpressLRS_AirRateConfig_128x)
        return ARRAY_SIZE(ExpressLRS_AirRateConfig_128x);
#if RADIO_SX128x_FLRC
    if (cfg == ExpressLRS_AirRateConfig_128x_FLRC)
        return ARRAY_SIZE(ExpressLRS_AirRateConfig_128x_FLRC);
#endif
#if TARGET_HANDSET && OTA_VANILLA_ENABLED
    if (cfg == ExpressLRS_AirRateConfig_128x_VANILLA)
        return ARRAY_SIZE(ExpressLRS_AirRateConfig_128x_VANILLA);
#endif
#endif
    return 0;
}

const expresslrs_mod_settings_t *get_elrs_airRateConfig(uint8_t rate)
{
    if (get_elrs_airRateMax() <= rate || !current_settings)
        return NULL;
    return &current_settings[rate];
}

uint8_t get_elrs_airRateIndex(void * current)
{
    if (!current_settings || !current)
        return 0;
    return ((uintptr_t)current - (uintptr_t)current_settings) / sizeof(expresslrs_mod_settings_t);
}

uint8_t get_elrs_airRateMax(void)
{
    return get_elrs_airRateMaxByConfig(current_settings);
}

uint8_t get_elrs_current_radio_type(void)
{
#if RADIO_SX127x
    if (current_settings == ExpressLRS_AirRateConfig_127x)
        return RADIO_TYPE_127x;
#endif
#if RADIO_SX128x
    if (current_settings == ExpressLRS_AirRateConfig_128x)
        return RADIO_TYPE_128x;
#if RADIO_SX128x_FLRC
    if (current_settings == ExpressLRS_AirRateConfig_128x_FLRC)
        return RADIO_TYPE_128x_FLRC;
#endif
#if TARGET_HANDSET && OTA_VANILLA_ENABLED
    // TODO: vanilla v4
    if (current_settings == ExpressLRS_AirRateConfig_128x_VANILLA)
        return RADIO_TYPE_128x_VANILLA_v3;
#endif
#endif
    return RADIO_TYPE_MAX;
}

uint8_t get_elrs_default_tlm_interval(uint8_t type, uint8_t rate)
{
    expresslrs_mod_settings_t const * const cfg = get_air_rate_config(type);
    if (cfg && rate < get_elrs_airRateMaxByConfig(cfg)) {
        return cfg[rate].TLMinterval;
    }
    return TLM_RATIO_NO_TLM;
}

#ifndef MY_UID
#error "UID is mandatory!"
#endif

void my_uid_print(void)
{
#if defined(DEBUG_SERIAL)
    uint8_t UID[6] = {MY_UID};
    DEBUG_PRINTF("MY_ID: 0x%X,0x%X,0x%X,0x%X,0x%X,0x%X\n",
                 UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);
#endif
}

uint8_t my_uid_crc8(void)
{
    uint8_t UID[6] = {MY_UID};
    return CalcCRC8len(UID, sizeof(UID), 0, 0xD5);
}

uint16_t my_uid_crc16(void)
{
    uint8_t UID[6] = {MY_UID};
    return CalcCRC16(UID, sizeof(UID), 0);
}

uint32_t my_uid_crc32(void)
{
    uint8_t UID[6] = {MY_UID};
    return CalcCRC32(UID, sizeof(UID));
}

uint32_t my_uid_to_u32(uint8_t _xor)
{
    uint32_t ret;
    uint8_t UID[6] = {MY_UID};
    ret = UID[2];
    ret <<= 8;
    ret += UID[3];
    ret <<= 8;
    ret += UID[4];
    ret <<= 8;
    ret += (UID[5] ^ _xor);
    return ret;
}

typedef struct {
    int rst, dio0, dio1, dio2;
    int busy, txen, rxen, pactrl;
    uint32_t nss;
    RadioInterface* radio_if;
    const char * str;
} RadioParameters_t;

#if RADIO_SX127x
SX127xDriver DRAM_FORCE_ATTR Radio127x;
#endif
#if RADIO_SX128x
SX1280Driver DRAM_FORCE_ATTR Radio128x;
#endif
#if !RADIO_SX127x && !RADIO_SX128x
#error "NO VALID RADIO!"
#endif
RadioInterface DRAM_FORCE_ATTR *Radio;

static RadioParameters_t DRAM_FORCE_ATTR RadioType[] = {
#if RADIO_SX127x
    {GPIO_PIN_RST_127x, GPIO_PIN_DIO0_127x, GPIO_PIN_DIO1_127x, GPIO_PIN_DIO2_127x,
     UNDEF_PIN, GPIO_PIN_TXEN_127x, GPIO_PIN_RXEN_127x, GPIO_PIN_PAEN_127x,
     GPIO_PIN_NSS_127x,
     &Radio127x, "SX127x"},
#endif
#if RADIO_SX128x
    {GPIO_PIN_RST_128x, GPIO_PIN_DIO0_128x, GPIO_PIN_DIO1_128x, GPIO_PIN_DIO2_128x,
     GPIO_PIN_BUSY, GPIO_PIN_TXEN_128x, GPIO_PIN_RXEN_128x, GPIO_PIN_PAEN_128x,
     GPIO_PIN_NSS_128x,
     &Radio128x, "SX128x"},
#endif
};

static_assert(0 < ARRAY_SIZE(RadioType), "INVALID CONFIG");

static RadioParameters_t* get_radio_type_cfg(uint8_t type)
{
#if RADIO_SX127x && RADIO_SX128x
    switch (type) {
        case RADIO_TYPE_127x:
            return &RadioType[RADIO_TYPE_127x];
        case RADIO_TYPE_128x:
        case RADIO_TYPE_128x_FLRC:
        case RADIO_TYPE_128x_VANILLA_v3:
        case RADIO_TYPE_128x_VANILLA_v4:
            return &RadioType[RADIO_TYPE_128x];
    }
#endif
    return &RadioType[0];
}

uint8_t common_config_get_radio_type(uint8_t mode)
{
    switch (mode) {
#if RADIO_SX127x
        case RADIO_RF_MODE_915_AU_FCC:
        case RADIO_RF_MODE_868_EU:
        case RADIO_RF_MODE_433_AU_EU:
            return RADIO_TYPE_127x;
#endif
#if RADIO_SX128x
        case RADIO_RF_MODE_2400_ISM:
        case RADIO_RF_MODE_2400_ISM_500Hz:
            return RADIO_TYPE_128x;
#if RADIO_SX128x_FLRC
        case RADIO_RF_MODE_2400_ISM_FLRC:
            return RADIO_TYPE_128x_FLRC;
#endif
#if TARGET_HANDSET && OTA_VANILLA_ENABLED
        case RADIO_RF_MODE_2400_ISM_VANILLA_v3:
            return RADIO_TYPE_128x_VANILLA_v3;
        case RADIO_RF_MODE_2400_ISM_VANILLA_v4:
            return RADIO_TYPE_128x_VANILLA_v4;
#endif
#endif
        default:
            break;
    };
    return RADIO_TYPE_MAX;
}

RadioInterface* common_config_radio(uint8_t type)
{
    RadioParameters_t * config;
    RadioInterface* radio;
    uint8_t iter;

    if (!current_settings) {
        /* Init both radio first */
        for (iter = 0; iter < ARRAY_SIZE(RadioType); iter++) {
            config = &RadioType[iter];
            radio = config->radio_if;
            radio->SetPins(config->rst, config->dio0, config->dio1, config->dio2,
                           config->busy, config->txen, config->rxen, config->nss,
                           config->pactrl);
            radio->SetSyncWord(my_uid_crc8());
            radio->SetSyncWordLong(my_uid_crc32());
            //radio->End();
        }
    }

    config = get_radio_type_cfg(type);
    radio = config->radio_if;

    if (0 > radio->Begin(GPIO_PIN_SCK, GPIO_PIN_MISO, GPIO_PIN_MOSI)) {
        DEBUG_PRINTF("[ERROR] Radio config failed!\n");
        return NULL;
    }

    DEBUG_PRINTF("RF: %s\n", config->str);
    FHSS_init(type);

    current_settings = get_air_rate_config(type);
    return radio;
}

int8_t validate_bl_indentifier(const uint8_t * info)
{
#ifdef TARGET_INDENTIFIER
    static const char bootloader_id[] = TARGET_INDENTIFIER;
    if (info) {
        uint8_t *ptr = (uint8_t*)bootloader_id;
        uint8_t count = sizeof(bootloader_id) - 1;
        while (count--) {
            if (*ptr++ != *info++)
                return -1;
        }
    }
#endif // TARGET_INDENTIFIER
#if RX_MODULE
    uint32_t irq = _SAVE_IRQ();
    char end[] = "_RX\r\n";
    CrsfSerial.write((uint8_t*)target_name, target_name_len-1);
    CrsfSerial.write((uint8_t*)end, sizeof(end)-1);
    _RESTORE_IRQ(irq);
#endif
    //DEBUG_PRINTF("Jumping to Bootloader...\n");
    delay(200);
    return 0;
}
