#include "flash.h"

#include <ch.h>
#include <hal.h>
#include <stdint.h>

#if defined(STM32H7)

// #pragma GCC optimize("O0")

/*
  this driver has been tested with STM32F427 and STM32F412
 */
#define FLASH_WORD_SIZE 32U

#ifndef BOARD_FLASH_SIZE
#error "You must define BOARD_FLASH_SIZE in kbyte"
#endif

#define KB(x)   ((x*1024))
// Refer Flash memory map in the User Manual to fill the following fields per microcontroller
#define STM32_FLASH_BASE    0x08000000
#define STM32_FLASH_SIZE    KB(BOARD_FLASH_SIZE)

// the 2nd bank of flash needs to be handled differently
#define STM32_FLASH_BANK2_START (STM32_FLASH_BASE+0x00080000)

#define STM32_FLASH_NPAGES  (BOARD_FLASH_SIZE / 128)
#define STM32_FLASH_FIXED_PAGE_SIZE 128


static uint32_t stm32_flash_getpagesize(uint32_t page);

#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB

static void stm32_flash_wait_idle(void)
{
    while ((FLASH->SR1 & (FLASH_SR_BSY|FLASH_SR_QW|FLASH_SR_WBNE)) ||
           (FLASH->SR2 & (FLASH_SR_BSY|FLASH_SR_QW|FLASH_SR_WBNE))) {
        __asm__("nop");
        // nop
    }
}

static void stm32_flash_unlock(void)
{
    stm32_flash_wait_idle();

    if (FLASH->CR1 & FLASH_CR_LOCK) {
        /* Unlock sequence */
        FLASH->KEYR1 = FLASH_KEY1;
        FLASH->KEYR1 = FLASH_KEY2;
    }
    if (FLASH->CR2 & FLASH_CR_LOCK) {
        /* Unlock sequence */
        FLASH->KEYR2 = FLASH_KEY1;
        FLASH->KEYR2 = FLASH_KEY2;
    }

    // disable the data cache - see stm32 errata 2.1.11
#ifdef FLASH_ACR_DCEN
    FLASH->ACR &= ~FLASH_ACR_DCEN;
#endif
}

void stm32_flash_lock(void)
{
    stm32_flash_wait_idle();
    if (FLASH->SR1 & FLASH_SR_QW) {
        FLASH->CR1 |= FLASH_CR_FW;
    }
    if (FLASH->SR2 & FLASH_SR_QW) {
        FLASH->CR2 |= FLASH_CR_FW;
    }
    stm32_flash_wait_idle();
    FLASH->CR1 |= FLASH_CR_LOCK;
    FLASH->CR2 |= FLASH_CR_LOCK;
    // reset and re-enable the data cache - see stm32 errata 2.1.11
#ifdef FLASH_ACR_DCEN
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |= FLASH_ACR_DCEN;
#endif
}

/*
  get the memory address of a page
 */
void* flash_get_page_addr(uint32_t page)
{
    if (page >= STM32_FLASH_NPAGES) {
        return 0;
    }
    return (void*)(uintptr_t)(STM32_FLASH_BASE + page * STM32_FLASH_FIXED_PAGE_SIZE * 1024U);
}

uint32_t flash_get_page_ofs(uint32_t page)
{
    return (uint32_t)flash_get_page_addr(page) - (uint32_t)STM32_FLASH_BASE;
}

/*
  get size in bytes of a page
 */
uint32_t stm32_flash_getpagesize(uint32_t page)
{
    (void)page;
    return STM32_FLASH_FIXED_PAGE_SIZE;
}


int16_t flash_get_page_num(void* address)
{
    uint16_t ret = 0;
    while((uint32_t)flash_get_page_addr(ret) <= (uint32_t)address) {
        ret++;
        if (ret >= STM32_FLASH_NPAGES) {
            return -1;
        }
    }
    return ret - 1;
}

static bool stm32_flash_ispageerased(uint32_t page)
{
    uint32_t addr;
    uint32_t count;

    if (page >= STM32_FLASH_NPAGES) {
        return false;
    }

    for (addr = (uint32_t)flash_get_page_addr(page), count = stm32_flash_getpagesize(page);
        count; count--, addr++) {
        if ((*(volatile uint8_t *)(addr)) != 0xff) {
            return false;
        }
    }

    return true;
}

/*
  erase a page
 */
bool flash_erase_page(void* address)
{
    uint16_t page = flash_get_page_num(address);
    if (page >= STM32_FLASH_NPAGES) {
        return false;
    }

    stm32_flash_wait_idle();
    stm32_flash_unlock();
    
    stm32_flash_wait_idle();

    if (page < 8) {
        // first bank
        FLASH->SR1 = 0x1FEF000E;

        stm32_flash_wait_idle();

        uint32_t snb = page << 8;

        // use 32 bit operations
        FLASH->CR1 = FLASH_CR_PSIZE_1 | snb | FLASH_CR_SER;
        FLASH->CR1 |= FLASH_CR_START;
        while (FLASH->SR1 & FLASH_SR_QW) ;

        if (FLASH->SR1) {
            // an error occurred
            stm32_flash_wait_idle();    
            FLASH->SR1 = 0x1FEF000E;
            stm32_flash_lock();
            return false;
        }
    } else {
        // second bank
        FLASH->SR2 = 0x1FEF000E;

        stm32_flash_wait_idle();

        uint32_t snb = (page-8) << 8;

        // use 32 bit operations
        FLASH->CR2 = FLASH_CR_PSIZE_1 | snb | FLASH_CR_SER;
        FLASH->CR2 |= FLASH_CR_START;
        while (FLASH->SR2 & FLASH_SR_QW);

        if (FLASH->SR2) {
            // an error occurred
            stm32_flash_wait_idle();    
            FLASH->SR2 = 0x1FEF000E;
            stm32_flash_lock();
            return false;
        }
    }

    stm32_flash_wait_idle();    
    stm32_flash_lock();
    return stm32_flash_ispageerased(page);
}

bool flash_write(void* address, volatile uint8_t num_bufs, struct flash_write_buf_s* volatile bufs)
{
    if (num_bufs == 0 || !address || (size_t)address % FLASH_WORD_SIZE != 0) {
        return false;
    }

    if (!(RCC->CR & RCC_CR_HSION)) {
        return false;
    }
    volatile uint32_t *CR, *CCR, *SR;

    stm32_flash_unlock();
    if (((uint32_t)address - STM32_FLASH_BASE) < (8 * STM32_FLASH_FIXED_PAGE_SIZE * 1024)) {
        CR = &FLASH->CR1;
        CCR = &FLASH->CCR1;
        SR = &FLASH->SR1;
    } else {
        CR = &FLASH->CR2;
        CCR = &FLASH->CCR2;
        SR = &FLASH->SR2;
    }

    // clear previous errors
    *SR = 0x1FEF000E;

    *CR = FLASH_CR_PSIZE_0;

    bool success = true;
    uint32_t* target_word_ptr = address;
    uint8_t buf_idx = 0;
    size_t buf_data_idx = 0;

    while (buf_data_idx >= bufs[buf_idx].len) {
        buf_idx++;
    }

    while (buf_idx < num_bufs) {
        union {
            uint32_t word_value[FLASH_WORD_SIZE/4];
            uint8_t bytes_value[FLASH_WORD_SIZE];
        } source_value;

        memset(&source_value,0,sizeof(source_value));

        for (uint8_t i=0; i<FLASH_WORD_SIZE; i++) {
            if (buf_idx >= num_bufs) {
                break;
            }
            source_value.bytes_value[i] = ((uint8_t*)bufs[buf_idx].data)[buf_data_idx];
            buf_data_idx++;
            while (buf_data_idx >= bufs[buf_idx].len) {
                buf_idx++;
                buf_data_idx = 0;
            }
        }
        *CCR = ~0;
        *CR |= FLASH_CR_PG;
        for (uint8_t i=0; i<FLASH_WORD_SIZE/4; i++) {
            while (*SR & (FLASH_SR_BSY|FLASH_SR_QW));
            target_word_ptr[i] = source_value.word_value[i];
        }

        __DSB();
        stm32_flash_wait_idle();
        *CCR = ~0;

        if (*SR) {
            // we got an error
            *SR = 0x1FEF000E;
            *CR &= ~(FLASH_CR_PG);
            success = false;
            goto failed;
        }

        for (uint8_t i=0; i<FLASH_WORD_SIZE/4; i++) {
            // confirm data is correctly written or not
            if (target_word_ptr[i] != source_value.word_value[i]) {
                *CR &= ~(FLASH_CR_PG);
                success = false;
                goto failed;
            }
        }
        target_word_ptr += FLASH_WORD_SIZE/4;
    }

failed:
    stm32_flash_lock();

    return success;
}

#endif // defined(STM32H7)
