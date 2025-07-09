#include "FHSS.h"
#include "debug_elrs.h"
#include "common.h"
#include "utils.h"
#include "crc.h"
#include "targets.h"
#if RADIO_SX127x
#include "LoRa_SX127x_Regs.h"
#endif
#if RADIO_SX128x
#include "SX1280_Regs.h"
#endif
#if OTA_VANILLA_ENABLED
#include "OTAvanilla.h"
#endif

// #define FHSS_TABLE_PRINT    1
#define FHSS_SEQ_TABLE_SIZE 256
#define FREQ_CORRECTION_MAX 100000

// Actual sequence of hops as indexes into the frequency list
static uint32_t DRAM_FORCE_ATTR FHSS_sync_channel;
static uint32_t DRAM_FORCE_ATTR FHSS_sequence_len;
static uint8_t DRAM_FORCE_ATTR FHSS_sequence_lut[FHSS_SEQ_TABLE_SIZE];
static int32_t DRAM_FORCE_ATTR FreqCorrectionMin;
static int32_t DRAM_FORCE_ATTR FreqCorrectionMax;

// Runtime variables
static uint32_t DRAM_ATTR FHSS_sequence_index;
static int32_t DRAM_ATTR FreqCorrection;
static uint32_t DRAM_ATTR FHSS_frequencies[FHSS_SEQ_TABLE_SIZE];

static uint32_t FHSSupdateFrequencies(uint8_t mode, uint32_t & rng_seed);
static void FHSSrandomiseSequence(uint32_t nbr_fhss_freqs, uint32_t rng_seed, uint8_t const mode);

static void FHSSprintSequence(const uint8_t sequence_len)
{
#if defined(DEBUG_SERIAL) && (TARGET_HANDSET || defined(FHSS_TABLE_PRINT))
    // output FHSS sequence
    uint8_t iter;
    DEBUG_PRINTF("FHSS Sequence:");
    for (iter = 0; iter < sequence_len; iter++) {
        if ((iter % 10) == 0) {
            DEBUG_PRINTF("\n");
            delay(5);
        }
        DEBUG_PRINTF(" %3u", FHSS_sequence_lut[iter]);
    }
    DEBUG_PRINTF("\n");
    delay(5);
#else
    (void)sequence_len;
#endif
}

void FHSS_init(uint8_t const mode)
{
    uint32_t rng_seed = 0;
    uint32_t band_count = FHSSupdateFrequencies(mode, rng_seed);
    FHSSrandomiseSequence(band_count, rng_seed, mode);
    FHSSprintSequence(FHSS_sequence_len);
}

void FAST_CODE_1 FHSSfreqCorrectionReset(void)
{
    FreqCorrection = 0;
}

int32_t FAST_CODE_1 FHSSfreqCorrectionApply(int32_t const error)
{
    FreqCorrection += error;
    if (FreqCorrectionMax < FreqCorrection || FreqCorrection < FreqCorrectionMin) {
        FreqCorrection = 0;
        DEBUG_PRINTF("Max FreqCorrection reached!\n");
    }
    return FreqCorrection;
}

void FAST_CODE_1 FHSSsetCurrIndex(uint32_t const value)
{ // set the current index of the FHSS pointer
    FHSS_sequence_index = value % FHSS_sequence_len;
}

uint32_t FAST_CODE_1 FHSSgetCurrIndex()
{ // get the current index of the FHSS pointer
    return FHSS_sequence_index;
}

void FAST_CODE_1 FHSSincCurrIndex()
{
#if !STAY_ON_INIT_CHANNEL
    FHSSsetCurrIndex(FHSS_sequence_index + 1);
#endif
}

uint8_t FAST_CODE_1 FHSScurrSequenceIndex()
{
    return (FHSS_sequence_lut[FHSS_sequence_index]);
}

uint8_t FAST_CODE_1 FHSScurrSequenceIndexIsSyncChannel()
{
    return (FHSScurrSequenceIndex() == FHSS_sync_channel);
}

uint32_t FAST_CODE_1 FHSSgetCurrFreq()
{
    return (FHSS_frequencies[FHSScurrSequenceIndex()] - FreqCorrection);
}

uint32_t FAST_CODE_1 FHSSgetNextFreq()
{
    FHSSincCurrIndex();
    return FHSSgetCurrFreq();
}

static void CalculateFhssFrequencies(uint32_t base, uint32_t const bw, size_t const count, float const step)
{
    size_t iter;
    for (iter = 0; iter < count; iter++) {
        FHSS_frequencies[iter] = (float)base / step;
        base += bw;
    }
}

static uint32_t FHSSupdateFrequencies(uint8_t const mode, uint32_t & rng_seed)
{
    uint32_t freq_base = 0;
    uint32_t bandwidth = 0;
    uint32_t band_count = 0;

    /* RF step is used to calculate suitable frequency for the radio */
    float const rf_step =
#if RADIO_SX127x && RADIO_SX128x
        (mode == RADIO_TYPE_127x) ? SX127X_FREQ_STEP : SX1280_FREQ_STEP;
#elif RADIO_SX127x
        SX127X_FREQ_STEP;
#elif RADIO_SX128x
        SX1280_FREQ_STEP;
#endif

    rng_seed = 0;

    /* Fill/calc FHSS frequencies base on requested mode */
    switch (mode) {
        case RADIO_TYPE_127x: {
#if defined(Regulatory_Domain_AU_433)
            freq_base = 433420000;
            bandwidth = 500000;
            band_count = 3;
#elif defined(Regulatory_Domain_EU_433)
            // Variable bandwidth, skip later calculation
            FHSS_frequencies[0] = 433100000. / rf_step;
            FHSS_frequencies[1] = 433925000. / rf_step;
            FHSS_frequencies[2] = 434450000. / rf_step;
            band_count = 3;
#elif defined(Regulatory_Domain_AU_915)
            freq_base = 915500000;
            bandwidth = 600000;
            band_count = 20;
#elif defined(Regulatory_Domain_FCC_915)
            freq_base = 903500000;
            bandwidth = 600000;
            band_count = 40;
#elif defined(Regulatory_Domain_EU_868)
            freq_base = 863275000;
            bandwidth = 525000;
            band_count = 18;
#elif defined(Regulatory_Domain_EU_868_R9)
            freq_base = 859504989;
            bandwidth = 500000;
            band_count = 26;
#endif
            break;
        }
        case RADIO_TYPE_128x: {
#if defined(Regulatory_Domain_ISM_2400_800kHz)
            freq_base = 2400400000;
            bandwidth = 850000;
            band_count = 94;
#else
            freq_base = 2400400000;
            bandwidth = 1650000;
            band_count = 49;
#endif
            break;
        }
        case RADIO_TYPE_128x_FLRC: {
            freq_base = 2400400000;
            bandwidth = 330000;
            band_count = 242;
            break;
        }
#if OTA_VANILLA_ENABLED
        case RADIO_TYPE_128x_VANILLA_v4:
        case RADIO_TYPE_128x_VANILLA_v3: {
            freq_base = 2400400000;
            bandwidth = 1000000;
            band_count = 80;
            rng_seed = my_uid_to_u32(OTA_VERSION_ID);
            break;
        }
#endif
    }
    if (!rng_seed)
        rng_seed = my_uid_crc32();

    DEBUG_PRINTF("FHSS freq base:%u, bw:%u, count:%u\n", freq_base, bandwidth, band_count);

    if (freq_base)
        CalculateFhssFrequencies(freq_base, bandwidth, band_count, rf_step);
    FreqCorrectionMin = -FREQ_CORRECTION_MAX / rf_step;
    FreqCorrectionMax = FREQ_CORRECTION_MAX / rf_step;
    return band_count;
}

/**
Requirements:
1. 0 every n hops
2. No two repeated channels
3. Equal occurance of each (or as even as possible) of each channel
4. Pseudorandom

Approach:
  Fill the sequence array with the sync channel every FHSS_FREQ_CNT
  Iterate through the array, and for each block, swap each entry in it with
  another random entry, excluding the sync channel.
*/
static void FHSSrandomiseSequence(const uint32_t nbr_fhss_freqs, const uint32_t rng_seed, uint8_t const mode)
{
    uint8_t const sync_channel =
        nbr_fhss_freqs / 2 + (mode >= RADIO_TYPE_128x_VANILLA_v3);
    // Number of hops in the FHSS_sequence_lut list before circling back around,
    // even multiple of the number of frequencies
    const uint32_t fhss_sequence_len = (FHSS_SEQ_TABLE_SIZE / nbr_fhss_freqs) * nbr_fhss_freqs;
    uint8_t temp, iter, offset, rand;

    DEBUG_PRINTF("FHSS sequence len: %u, sync channel: %u\n", fhss_sequence_len, sync_channel);

    rngSeed(rng_seed);

    // initialize the sequence array
    for (iter = 0; iter < fhss_sequence_len; iter++) {
        if ((iter % nbr_fhss_freqs) == 0) {
            FHSS_sequence_lut[iter] = sync_channel;
        } else if ((iter % nbr_fhss_freqs) == sync_channel) {
            FHSS_sequence_lut[iter] = 0;
        } else {
            FHSS_sequence_lut[iter] = iter % nbr_fhss_freqs;
        }
    }

    // Randomize sequence
    for (iter = 0; iter < fhss_sequence_len; iter++) {
        // if it's not the sync channel
        if ((iter % nbr_fhss_freqs) != 0) {
            // offset to start of current block
            offset = (iter / nbr_fhss_freqs) * nbr_fhss_freqs;
            // random number between 1 and nbr_fhss_freqs
            rand = rngN(nbr_fhss_freqs - 1) + 1;
            // switch this entry and another random entry in the same block
            temp = FHSS_sequence_lut[iter];
            FHSS_sequence_lut[iter] = FHSS_sequence_lut[offset + rand];
            FHSS_sequence_lut[offset + rand] = temp;
        }
    }

    // Update global params
    FHSS_sequence_len = fhss_sequence_len;
    FHSS_sync_channel = sync_channel;
}
