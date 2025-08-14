/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
/*
  this header is modelled on the one for the Nucleo-144 H743 board from ChibiOS
 */
#pragma once
#define STM32H7xx_MCUCONF
#define STM32H743_MCUCONF

/* Disable USB for this example to avoid requiring OTG configuration */
#define HAL_USE_USB                         FALSE

#define STM32_HSECLK 24000000U

#define STM32_LSECLK 32768U

#define STM32_LSEDRV (3U << 3U)

/*
 * General settings.
 */
#define STM32_NO_INIT                       FALSE
#define STM32_SYS_CK_ENFORCED_VALUE         STM32_HSICLK

/*
 * Memory attributes settings.
 */
#define STM32_NOCACHE_SRAM1_SRAM2           FALSE
#define STM32_NOCACHE_SRAM3                 TRUE

/*
 * PWR system settings.
 * Reading STM32 Reference Manual is required.
 * Register constants are taken from the ST header.
 */
#define STM32_VOS                           STM32_VOS_SCALE1
#define STM32_PWR_CR1                       (PWR_CR1_SVOS_1 | PWR_CR1_SVOS_0)
#define STM32_PWR_CR2                       (PWR_CR2_BREN)
#define STM32_PWR_CR3                       (PWR_CR3_LDOEN | PWR_CR3_USB33DEN)
#define STM32_PWR_CPUCR                     0

/*
 * Clock tree static settings.
 * Reading STM32 Reference Manual is required.
 */
#define STM32_HSI_ENABLED                   FALSE
#define STM32_LSI_ENABLED                   FALSE
#define STM32_CSI_ENABLED                   TRUE
#define STM32_HSI48_ENABLED                 TRUE
#define STM32_HSE_ENABLED                   TRUE
#define STM32_LSE_ENABLED                   FALSE
#define STM32_HSIDIV                        STM32_HSIDIV_DIV1

/*
 * PLLs static settings.
 * Reading STM32 Reference Manual is required.
 */
#define STM32_PLLSRC                        STM32_PLLSRC_HSE_CK
#define STM32_PLLCFGR_MASK                  ~0

/*
  setup PLLs based on HSE clock
 */
#if STM32_HSECLK == 8000000U
// this gives 400MHz system clock
#define STM32_PLL1_DIVM_VALUE               1
#define STM32_PLL2_DIVM_VALUE               1
#define STM32_PLL3_DIVM_VALUE               2
#elif STM32_HSECLK == 16000000U
// this gives 400MHz system clock
#define STM32_PLL1_DIVM_VALUE               2
#define STM32_PLL2_DIVM_VALUE               2
#define STM32_PLL3_DIVM_VALUE               4
#elif STM32_HSECLK == 24000000U
// this gives 400MHz system clock
#define STM32_PLL1_DIVM_VALUE               3
#define STM32_PLL2_DIVM_VALUE               3
#define STM32_PLL3_DIVM_VALUE               6
#else
#error "Unsupported HSE clock"
#endif

#if (STM32_HSECLK == 8000000U) || (STM32_HSECLK == 16000000U) || (STM32_HSECLK == 24000000U)
// common clock tree for multiples of 8MHz crystals
#define STM32_PLL1_DIVN_VALUE               100
#define STM32_PLL1_DIVP_VALUE               2
#define STM32_PLL1_DIVQ_VALUE               8
#define STM32_PLL1_DIVR_VALUE               2

#define STM32_PLL2_DIVN_VALUE               25
#define STM32_PLL2_DIVP_VALUE               2
#define STM32_PLL2_DIVQ_VALUE               2
#define STM32_PLL2_DIVR_VALUE               2

#define STM32_PLL3_DIVN_VALUE               72
#define STM32_PLL3_DIVP_VALUE               4
#define STM32_PLL3_DIVQ_VALUE               6
#define STM32_PLL3_DIVR_VALUE               9
#endif // 8MHz clock multiples

#define STM32_PLL1_ENABLED                  TRUE
#define STM32_PLL1_P_ENABLED                TRUE
#define STM32_PLL1_Q_ENABLED                TRUE
#define STM32_PLL1_R_ENABLED                TRUE
#define STM32_PLL1_FRACN_VALUE              0

#define STM32_PLL2_ENABLED                  TRUE
#define STM32_PLL2_P_ENABLED                TRUE
#define STM32_PLL2_Q_ENABLED                TRUE
#define STM32_PLL2_R_ENABLED                TRUE
#define STM32_PLL2_FRACN_VALUE              0

#define STM32_PLL3_ENABLED                  TRUE
#define STM32_PLL3_P_ENABLED                TRUE
#define STM32_PLL3_Q_ENABLED                TRUE
#define STM32_PLL3_R_ENABLED                TRUE
#define STM32_PLL3_FRACN_VALUE              0

/*
 * Core clocks dynamic settings (can be changed at runtime).
 * Reading STM32 Reference Manual is required.
 */
#define STM32_SW                            STM32_SW_PLL1_P_CK
#define STM32_RTCSEL                        STM32_RTCSEL_NOCLK
#define STM32_D1CPRE                        STM32_D1CPRE_DIV1
#define STM32_D1HPRE                        STM32_D1HPRE_DIV4
#define STM32_D1PPRE3                       STM32_D1PPRE3_DIV1
#define STM32_D2PPRE1                       STM32_D2PPRE1_DIV1
#define STM32_D2PPRE2                       STM32_D2PPRE2_DIV1
#define STM32_D3PPRE4                       STM32_D3PPRE4_DIV1

/*
 * Peripherals clocks static settings.
 * Reading STM32 Reference Manual is required.
 */
#define STM32_MCO1SEL                       STM32_MCO1SEL_HSE_CK
#define STM32_MCO1PRE_VALUE                 4
#define STM32_MCO2SEL                       STM32_MCO2SEL_SYS_CK
#define STM32_MCO2PRE_VALUE                 4
#define STM32_TIMPRE_ENABLE                 TRUE
#define STM32_HRTIMSEL                      0
#define STM32_STOPKERWUCK                   0
#define STM32_STOPWUCK                      0
#define STM32_RTCPRE_VALUE                  8
#define STM32_CKPERSEL                      STM32_CKPERSEL_HSE_CK
#define STM32_SDMMCSEL                      STM32_SDMMCSEL_PLL1_Q_CK
#define STM32_QSPISEL                       STM32_QSPISEL_HCLK
#define STM32_FMCSEL                        STM32_QSPISEL_HCLK
#define STM32_SWPSEL                        STM32_SWPSEL_PCLK1
#define STM32_FDCANSEL                      STM32_FDCANSEL_PLL1_Q_CK
#define STM32_DFSDM1SEL                     STM32_DFSDM1SEL_PCLK2
#define STM32_SPDIFSEL                      STM32_SPDIFSEL_PLL1_Q_CK
#define STM32_SPI45SEL                      STM32_SPI45SEL_PLL2_Q_CK
#define STM32_SPI123SEL                     STM32_SPI123SEL_PLL1_Q_CK
#define STM32_SAI23SEL                      STM32_SAI23SEL_PLL1_Q_CK
#define STM32_SAI1SEL                       STM32_SAI1SEL_PLL1_Q_CK
#define STM32_LPTIM1SEL                     STM32_LPTIM1SEL_PCLK1
#define STM32_CECSEL                        STM32_CECSEL_DISABLE
#define STM32_USBSEL                        STM32_USBSEL_PLL3_Q_CK
#define STM32_I2C123SEL                     STM32_I2C123SEL_PLL3_R_CK
#define STM32_RNGSEL                        STM32_RNGSEL_HSI48_CK
#define STM32_USART16SEL                    STM32_USART16SEL_PCLK2
#define STM32_USART234578SEL                STM32_USART234578SEL_PCLK1
#define STM32_SPI6SEL                       STM32_SPI6SEL_PLL2_Q_CK
#define STM32_SAI4BSEL                      STM32_SAI4BSEL_PLL1_Q_CK
#define STM32_SAI4ASEL                      STM32_SAI4ASEL_PLL1_Q_CK
#define STM32_ADCSEL                        STM32_ADCSEL_PLL3_R_CK
#define STM32_LPTIM345SEL                   STM32_LPTIM345SEL_PCLK4
#define STM32_LPTIM2SEL                     STM32_LPTIM2SEL_PCLK4
#define STM32_I2C4SEL                       STM32_I2C4SEL_PCLK4
#define STM32_LPUART1SEL                    STM32_LPUART1SEL_PCLK4
#define STM32_SDMMCSEL                      STM32_SDMMCSEL_PLL1_Q_CK

#define STM32_CAN_CAN1_IRQ_PRIORITY         11

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               8
#define STM32_ST_USE_TIMER                  2

/* MDMA IRQ priority required by ChibiOS MDMA driver */
#define STM32_IRQ_MDMA_PRIORITY             9

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
