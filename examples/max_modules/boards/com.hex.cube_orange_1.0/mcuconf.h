#pragma once

#define STM32H7xx_MCUCONF
#define STM32H743_MCUCONF

#define STM32_HSECLK 24000000U
#define STM32_VDD 330U

#define STM32_NO_INIT                       FALSE
#define STM32_HSI_ENABLED                   TRUE
#define STM32_LSI_ENABLED                   TRUE
#define STM32_HSE_ENABLED                   TRUE
#define STM32_LSE_ENABLED                   FALSE
#define HAL_USE_USB                          TRUE
#define HAL_USE_SERIAL_USB                   TRUE

/* Basic clock tree derived from 24 MHz HSE, conservative speeds */
#define STM32_MCO1SEL                       STM32_MCO1SEL_HSI_CK
#define STM32_MCO1PRE_VALUE                 1

#define STM32_RTCSEL                        STM32_RTCSEL_LSI_CK
#define STM32_SW                            STM32_SW_PLL1_P_CK
#define STM32_PLLSRC                        STM32_PLLSRC_HSE_CK

#define STM32_PLL1_DIVM_VALUE               12
#define STM32_PLL1_DIVN_VALUE               200
#define STM32_PLL1_DIVP_VALUE               2
#define STM32_PLL1_DIVQ_VALUE               12
#define STM32_PLL1_DIVR_VALUE               2

#define STM32_PLL3_DIVM_VALUE               5
#define STM32_PLL3_DIVN_VALUE               72
#define STM32_PLL3_DIVP_VALUE               4
#define STM32_PLL3_DIVQ_VALUE               6
#define STM32_PLL3_DIVR_VALUE               9

/* Provide safe PLL2 values even if disabled due to unconditional checks */
#define STM32_PLL2_ENABLED                   FALSE
#define STM32_PLL2_DIVM_VALUE                12
#define STM32_PLL2_DIVN_VALUE                150
#define STM32_PLL2_DIVP_VALUE                2
#define STM32_PLL2_DIVQ_VALUE                12
#define STM32_PLL2_DIVR_VALUE                2

/* Adjust prescalers for H7 domains */
#define STM32_D1CPRE                        STM32_D1CPRE_DIV1
#define STM32_D1HPRE                        STM32_D1HPRE_DIV4
#define STM32_D1PPRE3                       STM32_D1PPRE3_DIV2
#define STM32_D2PPRE1                       STM32_D2PPRE1_DIV2
#define STM32_D2PPRE2                       STM32_D2PPRE2_DIV2
#define STM32_D3PPRE4                       STM32_D3PPRE4_DIV2

#define STM32_ST_IRQ_PRIORITY               8
#define STM32_ST_USE_TIMER                  2
#define STM32_IRQ_MDMA_PRIORITY             9
#define STM32_USB_USE_OTG1                  TRUE
#define STM32_USB_USE_OTG2                  FALSE
/* Select 48MHz source for USB: HSI48 */
#define STM32_USBSEL                         STM32_USBSEL_HSI48_CK

/* SDMMC configuration */
#define STM32_SDC_USE_SDMMC1                TRUE
#define STM32_SDMMCSEL                      STM32_SDMMCSEL_PLL1_Q_CK

/* Disable PLL2 to avoid VCO checks if unused */
#define STM32_PLL2_ENABLED                   FALSE

/* IRQ priorities required by ChibiOS H7 drivers */
#define STM32_IRQ_EXTI0_PRIORITY            6
#define STM32_IRQ_EXTI1_PRIORITY            6
#define STM32_IRQ_EXTI2_PRIORITY            6
#define STM32_IRQ_EXTI3_PRIORITY            6
#define STM32_IRQ_EXTI4_PRIORITY            6
#define STM32_IRQ_EXTI5_9_PRIORITY          6
#define STM32_IRQ_EXTI10_15_PRIORITY        6
#define STM32_IRQ_EXTI16_PRIORITY           6
#define STM32_IRQ_EXTI17_PRIORITY           15
#define STM32_IRQ_EXTI18_PRIORITY           6
#define STM32_IRQ_EXTI19_PRIORITY           6
#define STM32_IRQ_EXTI20_21_PRIORITY        6
#define STM32_IRQ_EXTI22_PRIORITY           15
#define STM32_IRQ_FDCAN1_PRIORITY           10
#define STM32_IRQ_FDCAN2_PRIORITY           10
#define STM32_IRQ_QUADSPI1_PRIORITY         10
#define STM32_IRQ_SDMMC1_PRIORITY           9
#define STM32_IRQ_SDMMC2_PRIORITY           9
#define STM32_IRQ_TIM1_UP_PRIORITY          7
#define STM32_IRQ_TIM1_CC_PRIORITY          7
#define STM32_IRQ_TIM2_PRIORITY             7
#define STM32_IRQ_TIM3_PRIORITY             7
#define STM32_IRQ_TIM4_PRIORITY             7
#define STM32_IRQ_TIM5_PRIORITY             7
#define STM32_IRQ_TIM6_PRIORITY             7
#define STM32_IRQ_TIM7_PRIORITY             7
#define STM32_IRQ_TIM8_BRK_TIM12_PRIORITY   7
#define STM32_IRQ_TIM8_UP_TIM13_PRIORITY    7
#define STM32_IRQ_TIM8_TRGCO_TIM14_PRIORITY 7
#define STM32_IRQ_TIM8_CC_PRIORITY          7
#define STM32_IRQ_USART1_PRIORITY           12
#define STM32_IRQ_USART2_PRIORITY           12
#define STM32_IRQ_USART3_PRIORITY           12
#define STM32_IRQ_UART4_PRIORITY            12
#define STM32_IRQ_UART5_PRIORITY            12
#define STM32_IRQ_USART6_PRIORITY           12
#define STM32_IRQ_UART7_PRIORITY            12
#define STM32_IRQ_UART8_PRIORITY            12
#define STM32_IRQ_LPUART1_PRIORITY          12

/* Alias needed by can_driver_stm32h7 */
#define STM32_CAN_CAN1_IRQ_PRIORITY         STM32_IRQ_FDCAN1_PRIORITY
