/**
 ******************************************************************************
 * @file           : platform_storage.h
 * @brief          : Platform flash storage interface
 ******************************************************************************
 */

#ifndef __PLATFORM_STORAGE_H
#define __PLATFORM_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Flash storage address - last sector of Bank 2 (0x081E0000)
 * STM32H743 has 8 sectors per bank, each 128KB */
#define STORAGE_FLASH_ADDRESS 0x081E0000

/**
 * @brief Read data from flash storage
 * @param start Offset from storage base address
 * @param size Number of bytes to read
 * @param data Buffer to store read data
 * @return PLATFORM_OK on success, PLATFORM_ERROR on failure
 */
char platform_storage_read(uint16_t start, uint16_t size, uint8_t *data);

/**
 * @brief Write data to flash storage (erases sector first)
 * @param start Offset from storage base address
 * @param size Number of bytes to write
 * @param data Buffer containing data to write
 * @return PLATFORM_OK on success, PLATFORM_ERROR on failure
 */
char platform_storage_write(uint16_t start, uint16_t size, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_STORAGE_H */
