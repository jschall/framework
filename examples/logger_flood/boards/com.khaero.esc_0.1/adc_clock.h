#if !defined(STM32_ENFORCE_H7_REV_XY)
/* ADC clock source checks.*/
#if (STM32_D1HPRE == STM32_D1HPRE_DIV1)
#define STM32_ADC_SCLK                  STM32_SYS_CK
#else
#define STM32_ADC_SCLK                  (STM32_SYS_CK / 2)
#endif

#if STM32_ADC_ADC12_CLOCK_MODE == ADC_CCR_CKMODE_ADCCK
/* CHTODO: also check ADC_CCR_PRESC.*/
#define STM32_ADC12_CLOCK               (STM32_ADCCLK / 2)
#elif STM32_ADC_ADC12_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV1
#define STM32_ADC12_CLOCK               (STM32_ADC_SCLK / 1 / 2)
#elif STM32_ADC_ADC12_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV2
#define STM32_ADC12_CLOCK               (STM32_ADC_SCLK / 2 / 2)
#elif STM32_ADC_ADC12_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV4
#define STM32_ADC12_CLOCK               (STM32_ADC_SCLK / 4 / 2)
#else
#error "invalid clock mode selected for STM32_ADC_ADC12_CLOCK_MODE"
#endif

#if STM32_ADC_ADC3_CLOCK_MODE == ADC_CCR_CKMODE_ADCCK
/* CHTODO: also check ADC_CCR_PRESC.*/
#define STM32_ADC3_CLOCK               (STM32_ADCCLK / 2)
#elif STM32_ADC_ADC3_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV1
#define STM32_ADC3_CLOCK               (STM32_ADC_SCLK / 1 / 2)
#elif STM32_ADC_ADC3_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV2
#define STM32_ADC3_CLOCK               (STM32_ADC_SCLK / 2 / 2)
#elif STM32_ADC_ADC3_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV4
#define STM32_ADC3_CLOCK               (STM32_ADC_SCLK / 4 / 2)
#else
#error "invalid clock mode selected for STM32_ADC_ADC3_CLOCK_MODE"
#endif

#else /* defined(STM32_ENFORCE_H7_REV_XY) */

#if STM32_ADC_ADC12_CLOCK_MODE == ADC_CCR_CKMODE_ADCCK
#define STM32_ADC12_CLOCK               STM32_ADCCLK
#elif STM32_ADC_ADC12_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV1
#define STM32_ADC12_CLOCK               (STM32_HCLK / 1)
#elif STM32_ADC_ADC12_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV2
#define STM32_ADC12_CLOCK               (STM32_HCLK / 2)
#elif STM32_ADC_ADC12_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV4
#define STM32_ADC12_CLOCK               (STM32_HCLK / 4)
#else
#error "invalid clock mode selected for STM32_ADC_ADC12_CLOCK_MODE"
#endif

#if STM32_ADC_ADC3_CLOCK_MODE == ADC_CCR_CKMODE_ADCCK
#define STM32_ADC3_CLOCK                STM32_ADCCLK
#elif STM32_ADC_ADC3_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV1
#define STM32_ADC3_CLOCK                (STM32_HCLK / 1)
#elif STM32_ADC_ADC3_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV2
#define STM32_ADC3_CLOCK                (STM32_HCLK / 2)
#elif STM32_ADC_ADC3_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV4
#define STM32_ADC3_CLOCK                (STM32_HCLK / 4)
#else
#error "invalid clock mode selected for STM32_ADC_ADC3_CLOCK_MODE"
#endif

#endif /* defined(STM32_ENFORCE_H7_REV_XY) */

#if STM32_ADC12_CLOCK > STM32_ADCCLK_MAX
#error "STM32_ADC12_CLOCK exceeding maximum frequency (STM32_ADCCLK_MAX)"
#endif

#if STM32_ADC3_CLOCK > STM32_ADCCLK_MAX
#error "STM32_ADC3_CLOCK exceeding maximum frequency (STM32_ADCCLK_MAX)"
#endif

#if !defined(STM32_ENFORCE_H7_REV_XY)
/* ADC boost checks.*/
#if   STM32_ADC12_CLOCK >  6250000
#define STM32_ADC12_BOOST               (1U << 8U)
#elif STM32_ADC12_CLOCK > 12500000
#define STM32_ADC12_BOOST               (2U << 8U)
#elif STM32_ADC12_CLOCK > 25000000
#define STM32_ADC12_BOOST               (3U << 8U)
#else
#define STM32_ADC12_BOOST               (0U << 8U)
#endif

#if   STM32_ADC3_CLOCK >  6250000
#define STM32_ADC3_BOOST                (1U << 8U)
#elif STM32_ADC3_CLOCK > 12500000
#define STM32_ADC3_BOOST                (2U << 8U)
#elif STM32_ADC3_CLOCK > 25000000
#define STM32_ADC3_BOOST                (3U << 8U)
#else
#define STM32_ADC3_BOOST                (0U << 8U)
#endif

#else /* defined(STM32_ENFORCE_H7_REV_XY) */

#if STM32_ADC12_CLOCK > 20000000
#define STM32_ADC12_BOOST               (1U << 8U)
#else
#define STM32_ADC12_BOOST               (0U << 8U)
#endif

#if STM32_ADC3_CLOCK > 20000000
#define STM32_ADC3_BOOST                (1U << 8U)
#else
#define STM32_ADC3_BOOST                (0U << 8U)
#endif

#endif /* defined(STM32_ENFORCE_H7_REV_XY) */
