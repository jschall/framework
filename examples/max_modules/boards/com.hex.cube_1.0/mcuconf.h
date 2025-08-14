#pragma once

#define STM32F4xx_MCUCONF
#define STM32F427_MCUCONF

#define STM32_HSECLK 24000000U
#ifndef BOARD_FLASH_SIZE
#define BOARD_FLASH_SIZE 2048
#endif
#define STM32_VDD 330U

#define STM32_NO_INIT                       FALSE
#define STM32_HSI_ENABLED                   TRUE
#define STM32_LSI_ENABLED                   TRUE
#define STM32_HSE_ENABLED                   TRUE
#define STM32_LSE_ENABLED                   FALSE
#define STM32_CLOCK48_REQUIRED              TRUE
/* Enable HAL USB because modules like usb_slcan and SerialUSB require it */
#define HAL_USE_USB                          TRUE
#define STM32_SW                            STM32_SW_PLL
#define STM32_PLLSRC                        STM32_PLLSRC_HSE
#define STM32_PLLM_VALUE                    24
#define STM32_PLLN_VALUE                    336
#define STM32_PLLP_VALUE                    2
#define STM32_PLLQ_VALUE                    7
#define STM32_HPRE                          STM32_HPRE_DIV1
#define STM32_PPRE1                         STM32_PPRE1_DIV4
#define STM32_PPRE2                         STM32_PPRE2_DIV2
#define STM32_RTCSEL                        STM32_RTCSEL_LSI
#define STM32_RTCPRE_VALUE                  8
#define STM32_I2SSRC                        STM32_I2SSRC_CKIN
#define STM32_PLLI2SN_VALUE                 192
#define STM32_PLLI2SR_VALUE                 5

/* SPI3 enable and DMA mapping for F427 */
#define STM32_SPI_USE_SPI1                  FALSE
#define STM32_SPI_USE_SPI2                  FALSE
#define STM32_SPI_USE_SPI3                  TRUE
#define STM32_SPI_SPI3_DMA_PRIORITY         1
#define STM32_SPI_SPI3_IRQ_PRIORITY         10
#define STM32_SPI_SPI3_RX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 0)
#define STM32_SPI_SPI3_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 7)

/* IRQ priorities required by drivers */
#define STM32_ST_IRQ_PRIORITY               8
#define STM32_ST_USE_TIMER                  2
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
#define STM32_USB_USE_OTG1                  TRUE
#define STM32_USB_USE_OTG2                  FALSE

#define STM32_CAN_USE_CAN1                  TRUE
#define STM32_CAN_CAN1_IRQ_PRIORITY         11

/* I2C off, SPI1 off; use SPI3 (above) */
#define STM32_ST_IRQ_PRIORITY               8
#define STM32_ST_USE_TIMER                  2
#define STM32_I2C_USE_I2C1                  FALSE
#define STM32_I2C_USE_I2C2                  FALSE
#define STM32_I2C_USE_I2C3                  FALSE

#define STM32_SPI_USE_SPI1                  FALSE
#define STM32_SPI_USE_SPI2                  FALSE
#define STM32_SPI_USE_SPI3                  TRUE

#define STM32_SDC_SDIO_DMA_STREAM           STM32_DMA_STREAM_ID(2, 3)

/* IRQ priorities required by ChibiOS 21.11 for F4xx */
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

/* Prefer a single set of IRQ priorities to avoid redefinition warnings */
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

#define STM32_IRQ_USART1_PRIORITY           12
#define STM32_IRQ_USART2_PRIORITY           12
#define STM32_IRQ_USART3_PRIORITY           12
#define STM32_IRQ_UART4_PRIORITY            12
#define STM32_IRQ_UART5_PRIORITY            12
#define STM32_IRQ_USART6_PRIORITY           12
#define STM32_IRQ_UART7_PRIORITY            12
#define STM32_IRQ_UART8_PRIORITY            12

