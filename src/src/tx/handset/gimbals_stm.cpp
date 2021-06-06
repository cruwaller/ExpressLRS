#include "gimbals.h"
#include "gpio.h"
#include "internal.h"
#include "stm32_def.h"
#include "helpers.h"
#include "targets.h"
#include "1AUDfilter.h"
#include "debug_elrs.h"
#include "rc_channels.h"
#include "priorities.h"
#include "common_defs.h"

#if defined(STM32F7xx)
#include "stm32f7xx_hal_adc.h"
#include "stm32f7xx_hal_adc_ex.h"

#include "stm32f7xx_ll_adc.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_gpio.h"
#endif

#if (HANDSET_GIMBALS == GIMBALS_ANALOG)


/** Filter types */
#define FILTER_1AUD 1
#define FILTER_NO   2

#define FILTER_ENABLE   FILTER_NO

#define ADC_ISR_EN      0
#define TIMx_ISR_EN     0

// Just defaults
#define GIMBAL_LOW      194
#define GIMBAL_HIGH     3622
//#define GIMBAL_MID      (GIMBAL_LOW + (GIMBAL_HIGH - GIMBAL_LOW)/2) // 1906
#define GIMBAL_MID      1858
#define GIMBAL_CALIB_TOL 1000

#define TIMx            TIM6
#define TIMx_IRQn       TIM6_DAC_IRQn
#define TIMx_IRQx_FUNC  TIM6_DAC_IRQHandler
#define TIM_INVERVAL_US 1000U

/**
 * Filters enabled:
 *   0 = ADC_SAMPLETIME_480CYCLES = ~309us
 *   1 = ADC_SAMPLETIME_144CYCLES = ~110us
 *   2 = ADC_SAMPLETIME_112CYCLES =  ~90us
 *   3 = ADC_SAMPLETIME_84CYCLES  =  ~74us
 * No filter:
 *   0 = ADC_SAMPLETIME_480CYCLES = ~291us
 *   1 = ADC_SAMPLETIME_144CYCLES = ~97us
 */
#define ADC_SAMPLE_TIME 1
#define ADC_MARGIN      40U
#if ADC_SAMPLE_TIME == 0
#define ADC_SAMPLE_TIME_VAL ADC_SAMPLETIME_480CYCLES
#define TIM_MARGIN_US   (310U + ADC_MARGIN)
#elif ADC_SAMPLE_TIME == 1
#define ADC_SAMPLE_TIME_VAL ADC_SAMPLETIME_144CYCLES
#define TIM_MARGIN_US   (110U + ADC_MARGIN)
#elif ADC_SAMPLE_TIME == 2
#define ADC_SAMPLE_TIME_VAL ADC_SAMPLETIME_112CYCLES
#define TIM_MARGIN_US   (90U + ADC_MARGIN)
#else
#define ADC_SAMPLE_TIME_VAL ADC_SAMPLETIME_84CYCLES
#define TIM_MARGIN_US   (75U + ADC_MARGIN)
#endif

#if TIMx_ISR_EN
#define DBG_PIN SWITCH_6_1
#ifdef DBG_PIN
static struct gpio_out debug_pin;
#endif
#endif

/* Contains:
 *  [ 0... 3] = channel1
 *  [ 4... 7] = channel2
 *  [ 8...11] = channel3
 *  [12...15] = channel4
*/
#define OVERSAMPLES 4
static uint16_t DMA_ATTR raw_values[NUM_ANALOGS * OVERSAMPLES];

#define DEBUG_VARS 0
#if DEBUG_VARS
static volatile uint16_t DMA_ATTR DEBUG_VAL[NUM_ANALOGS];
#define DEBUG_VARS_PRINT 2
#define DEBUG_PRINT_GIMBAL 0
#endif

/* STM32 ADC pins in channel order 0...n */
static uint32_t adc_pins[] = {
    GPIO('A', 0), GPIO('A', 1), GPIO('A', 2), GPIO('A', 3),
    GPIO('A', 4), GPIO('A', 5), GPIO('A', 6), GPIO('A', 7),
    GPIO('B', 0), GPIO('B', 1),
    GPIO('C', 0), GPIO('C', 1), GPIO('C', 2), GPIO('C', 3),
    GPIO('C', 4), GPIO('C', 5),
};

class GimbalNoFilter
{
public:
    GimbalNoFilter() {
        output = GIMBAL_MID;
    }
    uint32_t FAST_CODE_1 getCurrent(void) {
        return output;
    }
    uint32_t FAST_CODE_1 update(uint32_t input) {
        output = input;
        return input;
    }
private:
    uint32_t output;
};


/* Create a filters */
#if FILTER_ENABLE == FILTER_1AUD
#define F_MIN   80
#define F_MAX   150
#define F_BETA  0.01f

// 1AUDFilter (JamesK):
//filterSpec_s raceFilter =      {200, 500, 0.01f}; // NOK!
//filterSpec_s freestyleFilter = {100, 250, 0.01f};
//filterSpec_s cinematicFilter = {10,   50, 0.01f};

static OneAUDfilter DRAM_FORCE_ATTR filters[NUM_ANALOGS] = {
    OneAUDfilter(F_MIN, F_MAX, F_BETA, TIM_INVERVAL_US),
    OneAUDfilter(F_MIN, F_MAX, F_BETA, TIM_INVERVAL_US),
    OneAUDfilter(F_MIN, F_MAX, F_BETA, TIM_INVERVAL_US),
    OneAUDfilter(F_MIN, F_MAX, F_BETA, TIM_INVERVAL_US),
};

#elif FILTER_ENABLE == FILTER_NO
static GimbalNoFilter DRAM_FORCE_ATTR filters[NUM_ANALOGS] = {
    GimbalNoFilter(),
    GimbalNoFilter(),
    GimbalNoFilter(),
    GimbalNoFilter(),
};

#else
#error "invalid filter selected"
#endif

static FORCED_INLINE void
timer_reset_period(void)
{
    TIMx->ARR = TIM_INVERVAL_US - 1;
}

void FAST_CODE_1 handle_dma_isr(void)
{
    uint32_t val;
    uint_fast8_t iter, index;
    for (iter = 0; iter < ARRAY_SIZE(raw_values); iter+=OVERSAMPLES) {
        val = raw_values[iter];
#if 1 < OVERSAMPLES
        val += raw_values[iter+1];
#endif
#if 2 < OVERSAMPLES
        val += raw_values[iter+2];
#endif
#if 3 < OVERSAMPLES
        val += raw_values[iter+3];
#endif
        val /= OVERSAMPLES;
        index = iter / OVERSAMPLES;
#if DEBUG_VARS
        DEBUG_VAL[index] = val;
#endif
        filters[index].update(val);
    }
#if DEBUG_VARS
#if (DEBUG_VARS_PRINT == 1)
#if DEBUG_PRINT_GIMBAL < 4
    DEBUG_PRINTF("%u,%u\n",
        DEBUG_VAL[DEBUG_PRINT_GIMBAL],
        (uint32_t)filters[DEBUG_PRINT_GIMBAL].getCurrent());
#else // !(DEBUG_PRINT_GIMBAL < 4)
    DEBUG_PRINTF("%u,%u,%u,%u,%u,%u,%u,%u\n",
        DEBUG_VAL[0], (uint32_t)filters[0].getCurrent(),
        DEBUG_VAL[1], (uint32_t)filters[1].getCurrent(),
        DEBUG_VAL[2], (uint32_t)filters[2].getCurrent(),
        DEBUG_VAL[3], (uint32_t)filters[3].getCurrent()
        );
#endif // DEBUG_PRINT_GIMBAL
#elif (DEBUG_VARS_PRINT == 2)
    DEBUG_PRINTF("%u,%u,%u,%u\n",
        DEBUG_VAL[0],DEBUG_VAL[1],DEBUG_VAL[2],DEBUG_VAL[3]);
#endif // DEBUG_VARS_PRINT
#endif // DEBUG_VARS
}


extern "C" {
void FAST_CODE_1 ADC_IRQHandler(void)
{
    if (READ_BIT(ADC1->SR, ADC_SR_OVR)) {
        WRITE_REG(ADC1->SR , ADC_SR_OVR);
    }
}

void FAST_CODE_1 DMA2_Stream0_IRQHandler(void)
{
    // DMA transfer complete.
    if (READ_BIT(DMA2->LISR, DMA_LISR_TCIF0)) {
        WRITE_REG(DMA2->LIFCR , DMA_LIFCR_CTCIF0);
        handle_dma_isr();
#ifdef DBG_PIN
        gpio_out_write(debug_pin, 0);
#endif
    }

    // DMA transfer error.
    if (READ_BIT(DMA2->LISR, DMA_LISR_TEIF0)) {
        WRITE_REG(DMA2->LIFCR , DMA_LIFCR_CTEIF0);
        DEBUG_PRINTF("DMA ERROR\n");
    }
}

void FAST_CODE_1 TIMx_IRQx_FUNC(void)
{
    uint16_t SR = TIMx->SR;
    if (SR & TIM_SR_UIF) {
        TIMx->SR = SR & ~(TIM_SR_UIF);
#ifdef DBG_PIN
        gpio_out_write(debug_pin, 1);
#endif
    }
}
}


static void configure_dma(void) {
    // Enable DMA peripheral clock.
    enable_pclock((uint32_t)DMA2_Stream0);

    // Configure DMA transfer:
    //  - DMA transfer in circular mode to match with ADC configuration:
    //    DMA unlimited requests.
    //  - DMA transfer from ADC without address increment.
    //  - DMA transfer to memory with address increment.
    //  - DMA transfer from ADC by half-word to match with ADC configuration:
    //    ADC resolution 12 bits.
    //  - DMA transfer to memory by half-word to match with ADC conversion data
    //    buffer variable type: half-word.
    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_CHSEL, 0x00);
    MODIFY_REG(DMA2_Stream0->CR,
                DMA_SxCR_DIR | DMA_SxCR_CIRC | DMA_SxCR_PINC | DMA_SxCR_MINC |
                DMA_SxCR_PSIZE | DMA_SxCR_MSIZE | DMA_SxCR_PL | DMA_SxCR_PFCTRL,
                0x00000000U |      // Direction: peripheral to memory
                DMA_SxCR_CIRC |    // Mode: circular
                0x00000000U |      // Peripheral: no increment
                DMA_SxCR_MINC |    // Memory: increment
                DMA_SxCR_PSIZE_0 | // Peripheral data align: halfword
                DMA_SxCR_MSIZE_0 | // Memory data align: halfword
                DMA_SxCR_PL_1);    // Priority: high

    // Set DMA transfer addresses.
    WRITE_REG(DMA2_Stream0->PAR, (uint32_t)&(ADC1->DR));
    WRITE_REG(DMA2_Stream0->M0AR, (uint32_t)&raw_values[0]);

    // Set DMA transfer size.
    MODIFY_REG(DMA2_Stream0->NDTR, DMA_SxNDT, ARRAY_SIZE(raw_values));

    // Enable DMA interrupts.
    NVIC_SetPriority(DMA2_Stream0_IRQn, ISR_PRIO_ADC);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    SET_BIT(DMA2_Stream0->CR, DMA_SxCR_TCIE);
    SET_BIT(DMA2_Stream0->CR, DMA_SxCR_TEIE);

    // Enable DMA transfer.
    SET_BIT(DMA2_Stream0->CR, DMA_SxCR_EN);
}


static void configure_timer(void)
{
    enable_pclock((uint32_t)TIMx);

    /* Config timer to trigger conversions */
    TIMx->CR1 = 0;
    TIMx->DIER = 0;
    TIMx->SR &= ~(TIM_SR_UIF);
    // Set clock prescaler to 1us
    TIMx->PSC = (2 * get_pclock_frequency((uint32_t)TIMx) / 1000000) - 1;
    TIMx->ARR = TIM_INVERVAL_US - 1;
    TIMx->CNT = 0;
    MODIFY_REG(TIMx->CR2, TIM_CR2_MMS, 0x2 << TIM_CR2_MMS_Pos); // TRGO update
#if TIMx_ISR_EN
    //TIMx->EGR = TIM_EGR_UG;
    TIMx->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIMx_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(TIMx_IRQn);
#endif
    TIMx->CR1 = TIM_CR1_CEN | TIM_CR1_URS | TIM_CR1_DIR;
}

static void configure_adc_channel(uint32_t channel, uint8_t rank)
{
    //DEBUG_PRINTF("ADC cfg: ch %u, rank %u\n", channel, rank);

    /** Configure for the selected ADC regular channel its corresponding
     * rank in the sequencer and its sample time.
     */
    uint32_t mask;
    __IO uint32_t * REG;
    rank -= 1;

    /* Set channel time */
    if (rank < 6) {
        REG = &ADC1->SQR3;
    } else if (rank < 12) {
        rank -= 6;
        REG = &ADC1->SQR2;
    } else if (rank < 16) {
        rank -= 12;
        REG = &ADC1->SQR1;
    }
    rank *= 5;
    mask = ADC_SQR3_SQ1_Msk << rank;
    MODIFY_REG(*REG, mask, channel << rank);

    /* Set sampling time */
    if (channel < 10) {
        REG = &ADC1->SMPR2;
    } else {
        channel -= 10;
        REG = &ADC1->SMPR1;
    }
    channel *= 3;
    mask = ADC_SMPR1_SMP10_Msk << channel;
    MODIFY_REG(*REG, mask, ADC_SAMPLE_TIME_VAL << channel);
}

static void configure_adc(void)
{
    uint32_t channels[4];
    uint32_t pin;
    uint8_t iter, chan, jter;

    channels[GIMBAL_IDX_L1] = GIMBAL_L1;
    channels[GIMBAL_IDX_L2] = GIMBAL_L2;
    channels[GIMBAL_IDX_R1] = GIMBAL_R1;
    channels[GIMBAL_IDX_R2] = GIMBAL_R2;

    configure_dma();
    configure_timer();

    /* Peripheral clock enable */
    enable_pclock((uint32_t)ADC1);

#if ADC_ISR_EN
    NVIC_SetPriority(ADC_IRQn, 0);
    NVIC_EnableIRQ(ADC_IRQn);
#endif

    // Set ADC clock source: APB2 clock / 2.
    MODIFY_REG(ADC123_COMMON->CCR, ADC_CCR_ADCPRE, 0);


    for (iter = 0; iter < ARRAY_SIZE(channels); iter++) {
        for (chan = 0; chan < ARRAY_SIZE(adc_pins); chan++) {
            pin = channels[iter];
            if (adc_pins[chan] == pin) {
                gpio_peripheral(pin, GPIO_ANALOG, 0);
                for (jter = 0; jter < 4; jter++) {
                    configure_adc_channel(chan, ((iter * 4) + jter + 1));
                }
            }
        }
    }

    //uint32_t CR1 = ADC1->CR1 & ~0x03C0FFFF;
    uint32_t CR2 = ADC1->CR2 & ~0x7F7F0F03;

    // Set ADC group regular continuous mode: off.
    //MODIFY_REG(ADC1->CR2, ADC_CR2_CONT, 0);

    // Set ADC group regular trigger source: rising edge of TIM2 TRGO.
    //MODIFY_REG(ADC1->CR2, ADC_CR2_EXTSEL, 0x0B << ADC_CR2_EXTSEL_Pos);
    //MODIFY_REG(ADC1->CR2, ADC_CR2_EXTEN, 0x01 << ADC_CR2_EXTEN_Pos);
    //CR2 |= (ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_3); // TIM4
    CR2 |= (ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_3 |ADC_CR2_EXTSEL_0); // TIM6
    CR2 |= 0x01 << ADC_CR2_EXTEN_Pos;

    // Set ADC group regular sequencer length: four channels.
    MODIFY_REG(ADC1->SQR1, ADC_SQR1_L, 15LU << ADC_SQR1_L_Pos);

    // Enable multiple conversions.
    SET_BIT(ADC1->CR1, (ADC_CR1_SCAN | ADC_IT_OVR));
    // Enable DMA transfer for ADC. and start
    //MODIFY_REG(ADC1->CR2, ADC_CR2_DMA | ADC_CR2_DDS, ADC_CR2_DDS | ADC_CR2_DMA);
    CR2 |= (ADC_CR2_DDS | ADC_CR2_DMA | ADC_CR2_ADON);
    ADC1->SR &= ~(ADC_FLAG_EOC | ADC_FLAG_OVR);
    ADC1->CR2 = CR2;

    //DEBUG_PRINTF("ADC started!\n");
}


void gimbals_init(void)
{
#ifdef DBG_PIN
    debug_pin = gpio_out_setup(DBG_PIN, 0);
#endif
    configure_adc();

    //uint32_t CFGR = RCC->DCKCFGR1;
    //DEBUG_PRINTF("RCC bit: %u\n", (CFGR & RCC_DCKCFGR1_TIMPRE_Msk));
#if 0
    while (1) {
#if DEBUG_VARS
        DEBUG_PRINTF("DBG! [0]=%u, [1]=%u, [2]=%u, [3]=%u\n",
            DEBUG_VAL[0], DEBUG_VAL[1], DEBUG_VAL[2], DEBUG_VAL[3]);
#else
        DEBUG_PRINTF("Filter: %u, %u, %u, %u\n",
            (uint32_t)filters[0].getCurrent(), (uint32_t)filters[1].getCurrent(),
            (uint32_t)filters[2].getCurrent(), (uint32_t)filters[3].getCurrent());
#endif
        delay(10);
    }
#endif
}

void FAST_CODE_1
gimbals_timer_adjust(uint32_t us)
{
    // Counts up
    TIMx->CNT = TIM_MARGIN_US;
}

void FAST_CODE_1
gimbals_get(uint16_t * const out)
{
    struct gimbal_limit * limit;
    uint32_t curr;
    uint8_t iter;
    for (iter = 0; iter < ARRAY_SIZE(pl_config.gimbals); iter++) {
        limit = &pl_config.gimbals[iter];
        curr = filters[iter].getCurrent();
        if (curr <= limit->mid)
            out[iter] = MAP_U16((uint16_t)curr, limit->low, limit->mid,
                                ANALOG_MIN_VAL, ANALOG_MID_VAL);
        else
            out[iter] = MAP_U16((uint16_t)curr, limit->mid+1, limit->high,
                                ANALOG_MID_VAL, ANALOG_MAX_VAL);
    }
}

void FAST_CODE_1 gimbals_process_loop(void) { /*dummy*/ }

uint8_t gimbals_calibrate_mid_point(uint8_t idx)
{
    uint32_t value = 0, current;
    uint32_t start;

    start = 0;
    while (start < 64) {
        current = filters[idx].getCurrent();
        if ((GIMBAL_MID - 500) < current && current < (GIMBAL_MID + 500)) {
            value += current;
            start++;
        }
        delay(10);
    }
    pl_config.gimbals[idx].mid = (value / start);
    return 1;
}

uint8_t gimbals_calibrate_min_point(uint8_t idx)
{
    uint32_t value = 0, current;
    uint32_t start = 0;
    while (start < 64) {
        current = filters[idx].getCurrent();
        if (current < (GIMBAL_LOW + GIMBAL_CALIB_TOL)) {
            value += current;
            start++;
        }
        delay(10);
    }
    pl_config.gimbals[idx].low = (value / start);
    return 1;
}

uint8_t gimbals_calibrate_max_point(uint8_t idx)
{
    uint32_t value = 0, current;
    uint32_t start = 0;
    while (start < 64) {
        current = filters[idx].getCurrent();
        if ((GIMBAL_HIGH - GIMBAL_CALIB_TOL) < current) {
            value += current;
            start++;
        }
        delay(10);
    }
    pl_config.gimbals[idx].high = (value / start);
    return 1;
}

uint8_t gimbals_calibrate(uint8_t * data)
{
    uint8_t type = data[0], res = 0;
    if (data[1] == 0) {
        /* Stop if needed... */
        return 0;
    }
    if (type & GIMBAL_CALIB_LOW)
        res = gimbals_calibrate_min_point(type & GIMBAL_CALIB_MAX);
    else if (type & GIMBAL_CALIB_MID)
        res = gimbals_calibrate_mid_point(type & GIMBAL_CALIB_MAX);
    else if (type & GIMBAL_CALIB_HIGH)
        res = gimbals_calibrate_max_point(type & GIMBAL_CALIB_MAX);
    data[0] = res;
    return 1;
}

#endif /* HANDSET_GIMBALS == GIMBALS_ANALOG */
