#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

struct flash_write_buf_s {
    size_t len;
    const void* data;
};

#if defined(STM32F4) || defined(STM32F7) || defined(STM32F3)
#define FLASH_WORD_SIZE 2U
#elif defined(STM32H7)
#define FLASH_WORD_SIZE 32U
#endif

bool flash_erase_page(void* page_addr);
bool flash_write(void* address, uint8_t num_bufs, struct flash_write_buf_s* bufs);
int16_t flash_get_page_num(void *address);
void* flash_get_page_addr(uint32_t page);
uint32_t flash_get_page_ofs(uint32_t page);

// On STM32H7, check a 32-byte flash word for ECC events.
// Returns true on success (no double-bit error). If provided, outputs whether a
// single-bit error was corrected and whether a double-bit error was detected.
bool flash_check_word_ecc(const void* address, bool* single_bit_corrected, bool* double_bit_error);