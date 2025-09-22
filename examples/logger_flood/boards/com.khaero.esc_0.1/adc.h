#pragma once

#define ADC_CCR_DUAL_MASK               (31U << 0U)
#define ADC_CCR_DUAL_FIELD(n)           ((n) << 0U)
#define ADC_CCR_DUAL_INDEPENDENT        (0U << 0U)  /**< @brief Independent, dual mode disabled.                             */
#define ADC_CCR_DUAL_REG_SIMULT         (6U << 0U)  /**< @brief Regular simultaneous.                                        */
#define ADC_CCR_DUAL_REG_INTERL         (7U << 0U)  /**< @brief Regular interleaved.                                         */
#define ADC_CCR_DUAL_INJ_SIMULT         (5U << 0U)  /**< @brief Injected simultaneous.                                       */
#define ADC_CCR_DUAL_INJ_ALTERNATE      (9U << 0U)  /**< @brief Injected alternate trigger.                                  */
#define ADC_CCR_DUAL_REG_SIM_INJ_SIM    (1U << 0U)  /**< @brief Combined regular simultaneous + injected simultaneous.       */
#define ADC_CCR_DUAL_REG_SIM_INJ_ALT    (2U << 0U)  /**< @brief Combined regular simultaneous + injected alternate trigger.  */
#define ADC_CCR_DUAL_REG_INT_INJ_SIM    (3U << 0U)  /**< @brief Combined regular interleaved  + injected simultaneous.       */
#define ADC_CCR_DELAY_MASK              (15U << 8U)
#define ADC_CCR_DELAY_FIELD(n)          ((n) << 8U)
#define ADC_CCR_DAMDF_MASK              (3U << 14U)
#define ADC_CCR_DAMDF_DISABLED          (0U << 14U)
#define ADC_CCR_DAMDF_HWORD             (2U << 14U)
#define ADC_CCR_DAMDF_BYTE              (3U << 14U)
#define ADC_CCR_CKMODE_MASK             (3U << 16U)
#define ADC_CCR_CKMODE_ADCCK            (0U << 16U)
#define ADC_CCR_CKMODE_AHB_DIV1         (1U << 16U)
#define ADC_CCR_CKMODE_AHB_DIV2         (2U << 16U)
#define ADC_CCR_CKMODE_AHB_DIV4         (3U << 16U)

#define ADC_CFGR_DMNGT_MASK             (3U << 0U)
#define ADC_CFGR_DMNGT_NODMA            (0U << 0U)
#define ADC_CFGR_DMNGT_ONESHOT          (1U << 0U)
#define ADC_CFGR_DMNGT_DFSDM            (2U << 0U)
#define ADC_CFGR_DMNGT_CIRCULAR         (3U << 0U)
