#include <hal.h>
#include <modules/timing/timing.h>
#include <common/ctor.h>
#include <stdlib.h>
#include <math.h>
#include <adc.h>
#include <adc_clock.h>

bool sdc_lld_is_card_inserted(SDCDriver *sdcp) { (void)sdcp; return true; }
bool sdc_lld_is_write_protected(SDCDriver *sdcp) { (void)sdcp; return false; }

#define STRING2(x) #x
#define STRING(x) STRING2(x)
#pragma message(STRING(STM32_SDMMC1CLK))
#pragma message(STRING(STM32_SDMMC_MAXCLK))

void boardInit(void) {
    rccResetAHB4(STM32_GPIO_EN_MASK);
    rccEnableAHB4(STM32_GPIO_EN_MASK, true);

    palSetLineMode(BOARD_PAL_LINE_CAN1_RX, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(BOARD_PAL_LINE_CAN1_TX, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);

    palSetLineMode(BOARD_PAL_LINE_SDMMC_D0, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
    palSetLineMode(BOARD_PAL_LINE_SDMMC_D1, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
    palSetLineMode(BOARD_PAL_LINE_SDMMC_D2, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
    palSetLineMode(BOARD_PAL_LINE_SDMMC_D3, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
    palSetLineMode(BOARD_PAL_LINE_SDMMC_CK, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(BOARD_PAL_LINE_SDMMC_CMD, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);

    palSetLineMode(BOARD_PAL_LINE_USB_M, PAL_MODE_ALTERNATE(10));
    palSetLineMode(BOARD_PAL_LINE_USB_P, PAL_MODE_ALTERNATE(10));


    palSetLineMode(BOARD_PAL_LINE_VBAT_H, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(BOARD_PAL_LINE_VBAT_L, PAL_MODE_INPUT_ANALOG);

    palSetLineMode(BOARD_PAL_LINE_I1_H, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(BOARD_PAL_LINE_I1_L, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(BOARD_PAL_LINE_I2_H, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(BOARD_PAL_LINE_I2_L, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(BOARD_PAL_LINE_I3_H, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(BOARD_PAL_LINE_I3_L, PAL_MODE_INPUT_ANALOG);

    palSetLineMode(BOARD_PAL_LINE_V1_H, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(BOARD_PAL_LINE_V1_L, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(BOARD_PAL_LINE_V2_H, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(BOARD_PAL_LINE_V2_L, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(BOARD_PAL_LINE_V3_H, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(BOARD_PAL_LINE_V3_L, PAL_MODE_INPUT_ANALOG);

// #ifdef BOARD_PAL_LINE_ADC_DEBUG_PIN
//     palSetLineMode(BOARD_PAL_LINE_ADC_DEBUG_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
// #endif
//     palSetLineMode(BOARD_PAL_LINE_WORKER_LPWORK, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
//     palSetLineMode(BOARD_PAL_LINE_WORKER_SLCAN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
//     palSetLineMode(BOARD_PAL_LINE_WORKER_CAN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
//     palSetLineMode(BOARD_PAL_LINE_WORKER_HPWORK, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
// 
//     palSetLineMode(BOARD_PAL_LINE_DBG1, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
//     palSetLineMode(BOARD_PAL_LINE_PWM_DEBUG, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
}

#if !defined(TARGET_BOOTLOADER) && defined(MODULE_INVERTER_ENABLED)

#include <memory_sections.h>
#include <modules/inverter/inverter.h>

Inverter _inverters[1] DTCM_BSS;

static PWMConfig pwmcfg = {
  0, // frequency
  0, // period
  NULL, // callback (TIM1 update/reset)
  { // channels
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  0b111 << TIM_CR2_MMS_Pos, // cr2
  0, // bdtr
  0 // dier
};

static const GPTConfig gpt3_cfg = {
    .frequency = 10000,  // doesn't matter as clock is external
    .callback  = NULL,
    .cr2       = (0x2 << TIM_CR2_MMS_Pos),  // MMS=010: TRGO on update
    .dier      = 0U
};

static const GPTConfig gpt4_cfg = {
    .frequency = 1000000,
    .callback  = NULL,
    .cr2       = 0,
    .dier      = 0
};

static GPTConfig gpt5_cfg = {
    .frequency = 10000,  // doesn't matter as clock is external
    .callback  = NULL,
    .cr2       = (0x2 << TIM_CR2_MMS_Pos),  // MMS=010: TRGO on update
    .dier      = 0U
};

static void start_adc(void);
#endif // TARGET_BOOTLOADER



#if !defined(TARGET_BOOTLOADER) && defined(MODULE_INVERTER_ENABLED)
uint32_t adc12_latency DTCM_BSS;
uint32_t adc3_latency DTCM_BSS;

static uint8_t encode_dtg(uint16_t dt);

ITCM_CODE void inverter_lld_init(Inverter* instance) {
    // Start ADCs
    start_adc();

    // Start TIM3, divides ADC sample rate
    // Clocked by TIM1 CH4
    gptStart(&GPTD3, &gpt3_cfg);
    GPTD3.tim->PSC = 0;
    GPTD3.tim->SMCR |= (0x7 << TIM_SMCR_SMS_Pos);
    if (instance->meas_tofs >= 0) { GPTD3.tim->SMCR |= TIM_SMCR_ETP; }
    gptStartContinuous(&GPTD3, instance->meas_fdiv);

    // Set up TIM4, used to measure ADC interrupt latency
    // Resets and starts on TIM3 TRGO
    gptStart(&GPTD4, &gpt4_cfg);
    GPTD4.tim->SMCR |= TIM_SMCR_SMS_3 | TIM_SMCR_ETP | STM32_TIM_SMCR_TS(2); // reset and start on TIM3 TRGO
    GPTD4.tim->ARR = 0xffffffff;

    // Set up TIM1, used for PWM
    pwmcfg.frequency = STM32_TIMCLK2;
    pwmcfg.period = pwmcfg.frequency/(2*instance->frequency);
    pwmStart(&PWMD1, &pwmcfg);
    PWMD1.tim->CR1 |= STM32_TIM_CR1_CMS(3); // Center-aligned mode 3. output-compare interrupt flags
    PWMD1.tim->CCER |= STM32_TIM_CCER_CC1NE | STM32_TIM_CCER_CC2NE | STM32_TIM_CCER_CC3NE; // Complementary output enable.

    // Set up TIM5, divides inverter update frequency
    gpt5_cfg.frequency = STM32_TIMCLK2;
    gptStart(&GPTD5, &gpt5_cfg);
    
    GPTD5.tim->SMCR |= STM32_TIM_SMCR_SMS(6);
}

ITCM_CODE void inverter_lld_start_I(Inverter* instance) {
    // update meas_fdiv
    gptStopTimerI(&GPTD3);
    gptStartContinuousI(&GPTD3, instance->meas_fdiv);

    // Set up deadtime generation
    uint8_t dtg = encode_dtg(instance->deadtime*PWMD1.clock);
    PWMD1.tim->BDTR = (PWMD1.tim->BDTR&(~TIM_BDTR_DTG_Msk))|dtg;

    pwmChangePeriodI(&PWMD1, STM32_TIMCLK2/(2*instance->frequency));

    pwmDisableChannelI(&PWMD1, 3);

    // Set up channel 4, used to trigger TIM3
    int32_t meas_tofs_ticks = labs((int32_t)roundf(pwmcfg.frequency*instance->meas_tofs));
    if (meas_tofs_ticks < 1) { meas_tofs_ticks = 1; }
    if (meas_tofs_ticks > (int32_t)PWMD1.tim->ARR - 1) { meas_tofs_ticks = (int32_t)PWMD1.tim->ARR - 1; }
    pwmEnableChannelI(&PWMD1, 3, meas_tofs_ticks);
}

ITCM_CODE void inverter_lld_enable(Inverter* instance) {
    if (instance != &INVD1) { return; }
    palSetLineMode(BOARD_PAL_LINE_PH1_H, PAL_MODE_ALTERNATE(1));
    palSetLineMode(BOARD_PAL_LINE_PH1_L, PAL_MODE_ALTERNATE(1));
    palSetLineMode(BOARD_PAL_LINE_PH2_H, PAL_MODE_ALTERNATE(1));
    palSetLineMode(BOARD_PAL_LINE_PH2_L, PAL_MODE_ALTERNATE(1));
    palSetLineMode(BOARD_PAL_LINE_PH3_H, PAL_MODE_ALTERNATE(1));
    palSetLineMode(BOARD_PAL_LINE_PH3_L, PAL_MODE_ALTERNATE(1));
}

ITCM_CODE void inverter_lld_disable(Inverter* instance) {
    if (instance != &INVD1) { return; }
    palSetLineMode(BOARD_PAL_LINE_PH1_H, PAL_MODE_INPUT);
    palSetLineMode(BOARD_PAL_LINE_PH1_L, PAL_MODE_INPUT);
    palSetLineMode(BOARD_PAL_LINE_PH2_H, PAL_MODE_INPUT);
    palSetLineMode(BOARD_PAL_LINE_PH2_L, PAL_MODE_INPUT);
    palSetLineMode(BOARD_PAL_LINE_PH3_H, PAL_MODE_INPUT);
    palSetLineMode(BOARD_PAL_LINE_PH3_L, PAL_MODE_INPUT);
}

ITCM_CODE void inverter_lld_setDuty_I(Inverter* instance, float dutyA, float dutyB, float dutyC) {
    if (instance != &INVD1) { return; }
    pwmEnableChannelI(&PWMD1, 0, dutyA*(PWMD1.tim->ARR));
    pwmEnableChannelI(&PWMD1, 1, dutyB*(PWMD1.tim->ARR));
    pwmEnableChannelI(&PWMD1, 2, dutyC*(PWMD1.tim->ARR));
}

ITCM_CODE static uint8_t encode_dtg(uint16_t dt) {
    if (dt <= 127) {
        return (uint8_t)dt;
    } else if (dt < 256) {
        return 0x80 | ((dt / 2) - 64);
    } else if (dt < 512) {
        return 0xC0 | ((dt / 8) - 32);
    } else if (dt <= 1008) {
        return 0xE0 | ((dt / 16) - 32);
    } else {
        return 0xFF;  // invalid input
    }
}

typedef uint32_t adc12_sample_t;
typedef uint16_t adc3_sample_t;

#define SEQ_LEN_ADC12 3
#define SEQ_LEN_ADC3 4
static adc12_sample_t adc12_buffers[2][SEQ_LEN_ADC12];
static adc3_sample_t adc3_buffers[2][SEQ_LEN_ADC3];
static const stm32_dma_stream_t *adc12_dma DTCM_BSS;
static const stm32_dma_stream_t *adc3_dma DTCM_BSS;

static void adc12_dma_callback(void *p, uint32_t flags);
static void adc3_dma_callback(void *p, uint32_t flags);

static void start_adc(void) {
    rccEnableADC12(true);
    rccEnableADC3(true);

    rccResetADC12();
    rccResetADC3();

    ADC1->CR = ADC_CR_ADVREGEN;
    ADC2->CR = ADC_CR_ADVREGEN;
    ADC3->CR = ADC_CR_ADVREGEN;
    osalSysPolledDelayX(OSAL_US2RTC(STM32_SYS_CK, 10U));

    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);
    ADC2->CR |= ADC_CR_ADCAL;
    while (ADC2->CR & ADC_CR_ADCAL);
    ADC3->CR |= ADC_CR_ADCAL;
    while (ADC3->CR & ADC_CR_ADCAL);

    ADC12_COMMON->CCR = ADC_CCR_DUAL_REG_SIMULT | STM32_ADC_ADC12_CLOCK_MODE | (2U << ADC_CCR_DAMDF_Pos);
    ADC3_COMMON->CCR = STM32_ADC_ADC3_CLOCK_MODE | ADC_CCR_VREFEN | ADC_CCR_TSEN;

    ADC1->CR |= STM32_ADC12_BOOST;
    ADC2->CR |= STM32_ADC12_BOOST;
    ADC3->CR |= STM32_ADC3_BOOST;

    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
    ADC2->ISR = ADC_ISR_ADRDY;
    ADC2->CR |= ADC_CR_ADEN;
    while (!(ADC2->ISR & ADC_ISR_ADRDY));
    ADC3->ISR = ADC_ISR_ADRDY;
    ADC3->CR |= ADC_CR_ADEN;
    while (!(ADC3->ISR & ADC_ISR_ADRDY));

    uint32_t ier = ADC_IER_OVRIE | ADC_IER_AWD1IE | ADC_IER_AWD2IE | ADC_IER_AWD3IE;
    ADC1->IER = ier;
    ADC2->IER = ier;
    ADC3->IER = ier;

    uint32_t cfgr = (1 << ADC_CFGR_EXTEN_Pos) | (0b100 << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_DMNGT_CIRCULAR;
    ADC1->CFGR = cfgr;
    ADC2->CFGR = cfgr;
    ADC3->CFGR = cfgr;

    ADC1->CFGR2 = 0;
    ADC2->CFGR2 = 0;
    ADC3->CFGR2 = 0;

    uint32_t difsel = 0;
#ifdef PHASE_VOLTAGE_DIFFERENTIAL
    difsel |= (1u << ADC_CH_V1) | (1u << ADC_CH_V2) | (1u << ADC_CH_V3);
#endif
#ifdef PHASE_CURRENT_DIFFERENTIAL
    difsel |= (1u << ADC_CH_I1) | (1u << ADC_CH_I2) | (1u << ADC_CH_I3);
#endif
#ifdef BATT_VOLTAGE_DIFFERENTIAL
    difsel |= (1u << ADC_CH_VBAT);
#endif

    ADC1->DIFSEL = difsel;
    ADC2->DIFSEL = difsel;
    ADC3->DIFSEL = difsel;

    ADC1->PCSEL = (1u << ADC_CH_I1) | (1u << ADC_CH_V1) | (1u << ADC_CH_VBAT);
    ADC2->PCSEL = (1u << ADC_CH_I2) | (1u << ADC_CH_V2);
    ADC3->PCSEL = (1u << ADC_CH_I3) | (1u << ADC_CH_V3) | (1u << ADC_CH_VREFINT) | (1u << ADC_CH_VSENSE);

    ADC1->LTR1 = 0; ADC1->HTR1 = 0; ADC1->LTR2 = 0; ADC1->HTR2 = 0; ADC1->LTR3 = 0; ADC1->HTR3 = 0;
    ADC1->AWD2CR = 0; ADC1->AWD3CR = 0;
    ADC2->LTR1 = 0; ADC2->HTR1 = 0; ADC2->LTR2 = 0; ADC2->HTR2 = 0; ADC2->LTR3 = 0; ADC2->HTR3 = 0;
    ADC2->AWD2CR = 0; ADC2->AWD3CR = 0;
    ADC3->LTR1 = 0; ADC3->HTR1 = 0; ADC3->LTR2 = 0; ADC3->HTR2 = 0; ADC3->LTR3 = 0; ADC3->HTR3 = 0;
    ADC3->AWD2CR = 0; ADC3->AWD3CR = 0;


    uint32_t smpr = 0;
    ADC1->SMPR1 = smpr; ADC1->SMPR2 = smpr;
    ADC2->SMPR1 = smpr; ADC2->SMPR2 = smpr;
    ADC3->SMPR1 = smpr; ADC3->SMPR2 = smpr;

    // NOTE: RM says "No overlapping sampling times for the two ADCs when converting the same channel."
    // NOTE: sequence length for ADC1 and ADC2 must be the same
    ADC1->SQR1 = (ADC_CH_I1 << 6U) | (ADC_CH_V1 << 12U) | (ADC_CH_VBAT    <<18U) | (SEQ_LEN_ADC12-1);
    ADC2->SQR1 = (ADC_CH_I2 << 6U) | (ADC_CH_V2 << 12U) | (ADC_CH_V2      <<18U) | (SEQ_LEN_ADC12-1);
    ADC3->SQR1 = (ADC_CH_I3 << 6U) | (ADC_CH_V3 << 12U) | (ADC_CH_VREFINT <<18U) | (ADC_CH_VSENSE << 24U) | (SEQ_LEN_ADC3-1);
    ADC1->SQR2 = 0; ADC1->SQR3 = 0; ADC1->SQR4 = 0;
    ADC2->SQR2 = 0; ADC2->SQR3 = 0; ADC2->SQR4 = 0;
    ADC3->SQR2 = 0; ADC3->SQR3 = 0; ADC3->SQR4 = 0;

    nvicEnableVector(STM32_ADC12_NUMBER, STM32_ADC_ADC12_IRQ_PRIORITY);
    nvicEnableVector(STM32_ADC3_NUMBER, STM32_ADC_ADC3_IRQ_PRIORITY);

    // Allocate DMA streams
    adc12_dma = dmaStreamAlloc(STM32_ADC_ADC12_DMA_STREAM,
                                STM32_ADC_ADC12_IRQ_PRIORITY,
                                adc12_dma_callback,
                                NULL);
    if (!adc12_dma) return; // handle this better later
    dmaSetRequestSource(adc12_dma, STM32_DMAMUX1_ADC1);

    adc3_dma = dmaStreamAlloc(STM32_ADC_ADC3_DMA_STREAM,
                               STM32_ADC_ADC3_IRQ_PRIORITY,
                               adc3_dma_callback,
                               NULL);
    if (!adc3_dma) return; // handle this better later

    dmaSetRequestSource(adc3_dma, STM32_DMAMUX1_ADC3);

    // Setup ADC12 DMA (32-bit transfers from CDR)
    dmaStreamSetPeripheral(adc12_dma, &ADC12_COMMON->CDR);
    dmaStreamSetMemory0(adc12_dma, adc12_buffers[0]);
    dmaStreamSetMemory1(adc12_dma, adc12_buffers[1]);
    dmaStreamSetTransactionSize(adc12_dma, SEQ_LEN_ADC12);
    uint32_t dma_mode12 = STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_MINC |
                          STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD |
                          STM32_DMA_CR_PL(STM32_ADC_ADC12_DMA_PRIORITY) |
                          STM32_DMA_CR_DBM | STM32_DMA_CR_CIRC |
                          STM32_DMA_CR_TCIE | STM32_DMA_CR_TEIE | STM32_DMA_CR_DMEIE;
    dmaStreamSetMode(adc12_dma, dma_mode12);
    dmaStreamEnable(adc12_dma);

    // Setup ADC3 DMA (16-bit transfers from DR)
    dmaStreamSetPeripheral(adc3_dma, &ADC3->DR);
    dmaStreamSetMemory0(adc3_dma, adc3_buffers[0]);
    dmaStreamSetMemory1(adc3_dma, adc3_buffers[1]);
    dmaStreamSetTransactionSize(adc3_dma, SEQ_LEN_ADC3);
    uint32_t dma_mode3 = STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_MINC |
                         STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD |
                         STM32_DMA_CR_PL(STM32_ADC_ADC3_DMA_PRIORITY) |
                         STM32_DMA_CR_DBM | STM32_DMA_CR_CIRC |
                         STM32_DMA_CR_TCIE | STM32_DMA_CR_TEIE | STM32_DMA_CR_DMEIE;
    dmaStreamSetMode(adc3_dma, dma_mode3);
    dmaStreamEnable(adc3_dma);

    ADC1->CR |= ADC_CR_ADSTART;
    ADC3->CR |= ADC_CR_ADSTART;
}

#if defined(__GNUC__)
#pragma GCC push_options
#pragma GCC optimize ("O3")
#endif

uint32_t errorcount DTCM_BSS;

ITCM_CODE OSAL_IRQ_HANDLER(STM32_ADC12_HANDLER) {
    OSAL_IRQ_PROLOGUE();

    uint32_t isr = ADC1->ISR;
    ADC1->ISR = isr;
    uint32_t isr2 = ADC2->ISR;
    ADC2->ISR = isr2;
    isr |= isr2;

    if (isr & (ADC_ISR_OVR | ADC_ISR_AWD1 | ADC_ISR_AWD2 | ADC_ISR_AWD3)) {
        errorcount++;
        OSAL_IRQ_EPILOGUE();
        return;
    }
    OSAL_IRQ_EPILOGUE();
}

ITCM_CODE OSAL_IRQ_HANDLER(STM32_ADC3_HANDLER) {
    OSAL_IRQ_PROLOGUE();

    uint32_t isr = ADC3->ISR;
    ADC3->ISR = isr;

    if (isr & (ADC_ISR_OVR | ADC_ISR_AWD1 | ADC_ISR_AWD2 | ADC_ISR_AWD3)) {
        errorcount++;
        OSAL_IRQ_EPILOGUE();
        return;
    }

    OSAL_IRQ_EPILOGUE();
}

static struct {
    uint16_t i_a, i_b, v_a, v_b, v_bus;
    uint32_t t_us;
} adc12_data DTCM_BSS;

static struct {
    uint16_t i_c, v_c, vrefint, vsense;
    uint32_t t_us;
} adc3_data DTCM_BSS;

float Vref DTCM_BSS;
float mcutemp DTCM_BSS;

ITCM_CODE static void sample_complete(void) {
    InverterSenseData meas;

    Vref = 3.3 * VREFINT_CAL / adc3_data.vrefint;
    mcutemp = (TS_CAL2_TEMP-TS_CAL1_TEMP)/(TS_CAL2-TS_CAL1) * (adc3_data.vsense-TS_CAL1)+TS_CAL1_TEMP;

    meas.v_ref = Vref;

#ifdef PHASE_VOLTAGE_DIFFERENTIAL
    meas.v_a = (adc12_data.v_a/32768.0-1)*Vref/INVD1_PHASE_VOLTAGE_SENSITIVITY;
    meas.v_b = (adc12_data.v_b/32768.0-1)*Vref/INVD1_PHASE_VOLTAGE_SENSITIVITY;
    meas.v_c = (adc3_data.v_c /32768.0-1)*Vref/INVD1_PHASE_VOLTAGE_SENSITIVITY;
#else
    meas.v_a = (adc12_data.v_a/65536.0)*Vref/INVD1_PHASE_VOLTAGE_SENSITIVITY;
    meas.v_b = (adc12_data.v_b/65536.0)*Vref/INVD1_PHASE_VOLTAGE_SENSITIVITY;
    meas.v_c = (adc3_data.v_c /65536.0)*Vref/INVD1_PHASE_VOLTAGE_SENSITIVITY;
#endif

#ifdef PHASE_CURRENT_DIFFERENTIAL
    meas.i_a = (adc12_data.i_a/32768.0-1)*Vref/INVD1_PHASE_CURRENT_SENSITIVITY;
    meas.i_b = (adc12_data.i_b/32768.0-1)*Vref/INVD1_PHASE_CURRENT_SENSITIVITY;
    meas.i_c = (adc3_data.i_c /32768.0-1)*Vref/INVD1_PHASE_CURRENT_SENSITIVITY;
#else
    meas.i_a = (adc12_data.i_a/65536.0)*Vref/INVD1_PHASE_CURRENT_SENSITIVITY;
    meas.i_b = (adc12_data.i_b/65536.0)*Vref/INVD1_PHASE_CURRENT_SENSITIVITY;
    meas.i_c = (adc3_data.i_c /65536.0)*Vref/INVD1_PHASE_CURRENT_SENSITIVITY;
#endif

#ifdef BATT_VOLTAGE_DIFFERENTIAL
    meas.v_bus = (adc12_data.v_bus/32768.0-1)*Vref/INVD1_BUS_VOLTAGE_SENSITIVITY;
#else
    meas.v_bus = (adc12_data.v_bus/65536.0)*Vref/INVD1_BUS_VOLTAGE_SENSITIVITY;
#endif

    meas.t_us = adc12_data.t_us;
    inverter_update_I(&INVD1, meas);
}

ITCM_CODE static void adc12_dma_callback(void *p, uint32_t flags) {
    (void)p;

#ifdef BOARD_PAL_LINE_ADC_DEBUG_PIN
    // Set ADC debug pin high during ADC DMA interrupt
    palSetLine(BOARD_PAL_LINE_ADC_DEBUG_PIN);
#endif

    if (flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) {
        errorcount++;
        return;
    }

    // Get the completed buffer index (opposite of current target)
    uint32_t cr = adc12_dma->stream->CR;
    uint8_t buf_idx = (cr & STM32_DMA_CR_CT) ? 0 : 1;
    const adc12_sample_t *buf = adc12_buffers[buf_idx];

    // Extract samples from 32-bit CDR format
    adc12_data.i_a = buf[0] & 0xFFFF;
    adc12_data.i_b = buf[0] >> 16;

    adc12_data.v_a = buf[1] & 0xFFFF;
    adc12_data.v_b = buf[1] >> 16;
    adc12_data.v_bus = buf[2] & 0xFFFF;

    // Compute timestamp (time of TRGO)
    chSysLockFromISR();
    adc12_data.t_us = micros() - GPTD4.tim->CNT;
    chSysUnlockFromISR();

    // Check if ADC3 data is ready and timestamps are close
    if (labs((int32_t)(adc12_data.t_us - adc3_data.t_us)) <= 2) {
        sample_complete();
    }

#ifdef BOARD_PAL_LINE_ADC_DEBUG_PIN
    // Set ADC debug pin high during ADC DMA interrupt
    palClearLine(BOARD_PAL_LINE_ADC_DEBUG_PIN);
#endif
}

// DMA callback for ADC3
ITCM_CODE static void adc3_dma_callback(void *p, uint32_t flags) {
    (void)p;

#ifdef BOARD_PAL_LINE_ADC_DEBUG_PIN
    // Set ADC debug pin high during ADC DMA interrupt
    palSetLine(BOARD_PAL_LINE_ADC_DEBUG_PIN);
#endif

    if (flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) {
        errorcount++;
        return;
    }

    // Get the completed buffer index (opposite of current target)
    uint32_t cr = adc3_dma->stream->CR;
    uint8_t buf_idx = (cr & STM32_DMA_CR_CT) ? 0 : 1;
    const adc3_sample_t *buf = adc3_buffers[buf_idx];

    // Extract samples
    adc3_data.i_c = (float)(buf[0]);
    adc3_data.v_c = (float)(buf[1]);
    adc3_data.vrefint = (float)(buf[2]);
    adc3_data.vsense = (float)(buf[3]);

    // Compute timestamp (time of TRGO)
    chSysLockFromISR();
    adc3_data.t_us = micros() - GPTD4.tim->CNT;
    chSysUnlockFromISR();

    // Check if ADC12 data is ready and timestamps are close
    if (labs((int32_t)(adc12_data.t_us - adc3_data.t_us)) <= 2) {
        sample_complete();
    }

#ifdef BOARD_PAL_LINE_ADC_DEBUG_PIN
    // Set ADC debug pin high during ADC DMA interrupt
    palClearLine(BOARD_PAL_LINE_ADC_DEBUG_PIN);
#endif
}
#if defined(__GNUC__)
#pragma GCC pop_options
#endif
#endif // !defined(TARGET_BOOTLOADER) && defined(MODULE_INVERTER_ENABLED)

