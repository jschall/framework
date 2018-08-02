/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

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

#pragma once
/*
 * STM32F4xx drivers configuration.
 * The following settings override the default settings present in
 * the various device driver implementation headers.
 * Note that the settings for each driver only have effect if the whole
 * driver is enabled in halconf.h.
 *
 * IRQ priorities:
 * 15...0       Lowest...Highest.
 *
 * DMA priorities:
 * 0...3        Lowest...Highest.
 */
#define STM32F4xx_MCUCONF
#define STM32_HSECLK 24000000U

// Board voltage. Required for performance limits calculation
#define STM32_VDD 330U
/*
 * HAL driver system settings.
 */
#define STM32_NO_INIT                       FALSE
#define STM32_HSI_ENABLED                   TRUE
#define STM32_LSI_ENABLED                   TRUE
#define STM32_HSE_ENABLED                   TRUE
#define STM32_LSE_ENABLED                   FALSE
#define STM32_CLOCK48_REQUIRED              TRUE
#define STM32_SW                            STM32_SW_PLL
#define STM32_PLLSRC                        STM32_PLLSRC_HSE
#define STM32_PLLM_VALUE                    24
#define STM32_PLLN_VALUE                    288
#define STM32_PLLP_VALUE                    4
#define STM32_PLLQ_VALUE                    6
#define STM32_HPRE                          STM32_HPRE_DIV1
#define STM32_PPRE1                         STM32_PPRE1_DIV2
#define STM32_PPRE2                         STM32_PPRE2_DIV1
#define STM32_RTCSEL                        STM32_RTCSEL_LSI
#define STM32_RTCPRE_VALUE                  8
#define STM32_MCO1SEL                       STM32_MCO1SEL_HSI
#define STM32_MCO1PRE                       STM32_MCO1PRE_DIV1
#define STM32_MCO2SEL                       STM32_MCO2SEL_SYSCLK
#define STM32_MCO2PRE                       STM32_MCO2PRE_DIV5
#define STM32_I2SSRC                        STM32_I2SSRC_CKIN
#define STM32_PLLI2SN_VALUE                 192
#define STM32_PLLI2SR_VALUE                 5
#define STM32_PVD_ENABLE                    FALSE
#define STM32_PLS                           STM32_PLS_LEV0
#define STM32_BKPRAM_ENABLE                 FALSE

#ifndef TARGET_BOOTLOADER
  #define STM32_SPI_USE_SPI4                  TRUE
  #define STM32_SPI_SPI4_DMA_PRIORITY         1
  #define STM32_SPI_SPI4_IRQ_PRIORITY         10
  #define STM32_SPI_DMA_ERROR_HOOK(spip)      osalSysHalt("DMA failure")
  #define STM32_SPI_SPI4_RX_DMA_STREAM   STM32_DMA_STREAM_ID(2, 3)
  #define STM32_SPI_SPI4_TX_DMA_STREAM   STM32_DMA_STREAM_ID(2, 1)
#endif


#define STM32_USB_USE_OTG1                  TRUE
#define STM32_USB_USE_OTG2                  TRUE
#define STM32_CAN_USE_CAN1                  TRUE
#define STM32_CAN_CAN1_IRQ_PRIORITY         11

#define STM32_CAN_USE_CAN2                  TRUE
#define STM32_CAN_CAN2_IRQ_PRIORITY         11

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               8
#define STM32_ST_USE_TIMER                  2


/*
 * EXT driver system settings.
 */
#define STM32_EXT_EXTI0_IRQ_PRIORITY        6
#define STM32_EXT_EXTI1_IRQ_PRIORITY        6
#define STM32_EXT_EXTI2_IRQ_PRIORITY        6
#define STM32_EXT_EXTI3_IRQ_PRIORITY        6
#define STM32_EXT_EXTI4_IRQ_PRIORITY        6
#define STM32_EXT_EXTI5_9_IRQ_PRIORITY      6
#define STM32_EXT_EXTI10_15_IRQ_PRIORITY    6
#define STM32_EXT_EXTI16_IRQ_PRIORITY       6
#define STM32_EXT_EXTI17_IRQ_PRIORITY       6
#define STM32_EXT_EXTI18_IRQ_PRIORITY       6
#define STM32_EXT_EXTI19_IRQ_PRIORITY       6
#define STM32_EXT_EXTI20_IRQ_PRIORITY       6
#define STM32_EXT_EXTI21_22_29_IRQ_PRIORITY 6
#define STM32_EXT_EXTI30_32_IRQ_PRIORITY    6
#define STM32_EXT_EXTI33_IRQ_PRIORITY       6
