#pragma once

#define STM32F7xx_MCUCONF
#define STM32F767_MCUCONF

#define STM32_HSECLK 8000000U
#define STM32_VDD 330U

#define STM32_NO_INIT                       FALSE
#define STM32_HSI_ENABLED                   TRUE
#define STM32_LSI_ENABLED                   TRUE
#define STM32_HSE_ENABLED                   TRUE
#define STM32_LSE_ENABLED                   FALSE
#define STM32_CLOCK48_REQUIRED              TRUE
#define HAL_USE_USB                          TRUE
#define HAL_USE_SERIAL_USB                   TRUE
#define STM32_USB_USE_OTG1                  TRUE
#define STM32_USB_USE_OTG2                  FALSE

#define STM32_SW                            STM32_SW_PLL
#define STM32_PLLSRC                        STM32_PLLSRC_HSE
#define STM32_PLLM_VALUE                    8
#define STM32_PLLN_VALUE                    432
#define STM32_PLLP_VALUE                    2
#define STM32_PLLQ_VALUE                    9
#define STM32_HPRE                          STM32_HPRE_DIV1
#define STM32_PPRE1                         STM32_PPRE1_DIV4
#define STM32_PPRE2                         STM32_PPRE2_DIV2
#define STM32_RTCSEL                        STM32_RTCSEL_LSI

#define STM32_ST_IRQ_PRIORITY               8
#define STM32_ST_USE_TIMER                  2

/* IRQ priorities required by ChibiOS drivers on F7 */
#define STM32_IRQ_EXTI0_PRIORITY            6
#define STM32_IRQ_EXTI1_PRIORITY            6
#define STM32_IRQ_EXTI2_PRIORITY            6
#define STM32_IRQ_EXTI3_PRIORITY            6
#define STM32_IRQ_EXTI4_PRIORITY            6
#define STM32_IRQ_EXTI5_9_PRIORITY          6
#define STM32_IRQ_EXTI10_15_PRIORITY        6
#define STM32_IRQ_EXTI16_PRIORITY           6
#define STM32_IRQ_EXTI17_PRIORITY           6
#define STM32_IRQ_EXTI18_PRIORITY           6
#define STM32_IRQ_EXTI19_PRIORITY           6
#define STM32_IRQ_EXTI20_PRIORITY           6
#define STM32_IRQ_EXTI21_PRIORITY           6
#define STM32_IRQ_EXTI22_PRIORITY           6
#define STM32_IRQ_EXTI23_PRIORITY           6

#define STM32_IRQ_USART1_PRIORITY           12
#define STM32_IRQ_USART2_PRIORITY           12
#define STM32_IRQ_USART3_PRIORITY           12
#define STM32_IRQ_USART6_PRIORITY           12
#define STM32_IRQ_UART4_PRIORITY            12
#define STM32_IRQ_UART5_PRIORITY            12
#define STM32_IRQ_UART7_PRIORITY            12
#define STM32_IRQ_UART8_PRIORITY            12

#define STM32_IRQ_TIM1_BRK_TIM9_PRIORITY    8
#define STM32_IRQ_TIM1_UP_TIM10_PRIORITY    8
#define STM32_IRQ_TIM1_TRGCO_TIM11_PRIORITY 8
#define STM32_IRQ_TIM1_CC_PRIORITY          8
#define STM32_IRQ_TIM2_PRIORITY             8
#define STM32_IRQ_TIM3_PRIORITY             8
#define STM32_IRQ_TIM4_PRIORITY             8
#define STM32_IRQ_TIM5_PRIORITY             8
#define STM32_IRQ_TIM6_PRIORITY             8
#define STM32_IRQ_TIM7_PRIORITY             8
#define STM32_IRQ_TIM8_BRK_TIM12_PRIORITY   8
#define STM32_IRQ_TIM8_UP_TIM13_PRIORITY    8
#define STM32_IRQ_TIM8_TRGCO_TIM14_PRIORITY 8
#define STM32_IRQ_TIM8_CC_PRIORITY          8

/* SDC/SDMMC configuration */
#define STM32_SDC_USE_SDMMC1                TRUE
/* SDMMC1 DMA stream mapping for F767, matches ChibiOS reference configs */
#define STM32_SDC_SDMMC1_DMA_STREAM         STM32_DMA_STREAM_ID(2, 3)
#define STM32_IRQ_EXTI0_PRIORITY            6
#define STM32_IRQ_EXTI1_PRIORITY            6
#define STM32_IRQ_EXTI2_PRIORITY            6
#define STM32_IRQ_EXTI3_PRIORITY            6
#define STM32_IRQ_EXTI4_PRIORITY            6
#define STM32_IRQ_EXTI5_9_PRIORITY          6
#define STM32_IRQ_EXTI10_15_PRIORITY        6

#define STM32_CAN_USE_CAN1                  TRUE
#define STM32_CAN_CAN1_IRQ_PRIORITY         11
