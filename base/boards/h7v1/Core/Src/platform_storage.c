/**
 ******************************************************************************
 * @file           : platform_storage.c
 * @brief          : Platform flash storage implementation
 ******************************************************************************
 */

#include "platform_storage.h"
#include "main.h"
#include <platform.h>
#include <string.h>

/**
 * @brief Read data from flash storage
 */
char platform_storage_read(uint16_t start, uint16_t size, uint8_t *data) {
	uint32_t flash_addr = STORAGE_FLASH_ADDRESS + start;
	
	// Read directly from flash memory
	for (uint16_t i = 0; i < size; i++) {
		data[i] = *(uint8_t *)(flash_addr + i);
	}
	
	return PLATFORM_OK;
}

/**
 * @brief Write data to flash storage (erases sector first)
 */
char platform_storage_write(uint16_t start, uint16_t size, uint8_t *data) {
	uint32_t flash_addr = STORAGE_FLASH_ADDRESS + start;
	HAL_StatusTypeDef status;
	
	// Unlock flash
	HAL_FLASH_Unlock();
	
	// Erase the sector first
	FLASH_EraseInitTypeDef erase_init;
	uint32_t sector_error = 0;
	
	erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase_init.Banks = FLASH_BANK_2;
	erase_init.Sector = FLASH_SECTOR_7;  // Last sector of Bank 2
	erase_init.NbSectors = 1;
	erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // 2.7V to 3.6V
	
	status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
	if (status != HAL_OK) {
		HAL_FLASH_Lock();
		return PLATFORM_ERROR;
	}
	
	// Program the flash (32-byte words at a time)
	for (uint16_t i = 0; i < size; i += 32) {
		uint32_t data_word[8];  // 32 bytes = 8 words (256-bit flash word for H7)
		
		// Prepare 32 bytes (256 bits) for programming
		for (int j = 0; j < 8; j++) {
			data_word[j] = 0xFFFFFFFF;  // Default erased value
		}
		
		// Copy actual data
		uint16_t bytes_to_copy = (size - i) < 32 ? (size - i) : 32;
		memcpy(data_word, &data[i], bytes_to_copy);
		
		// Program flash
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, 
		                           flash_addr + i, 
		                           (uint32_t)data_word);
		
		if (status != HAL_OK) {
			HAL_FLASH_Lock();
			return PLATFORM_ERROR;
		}
	}
	
	// Lock flash
	HAL_FLASH_Lock();
	
	return PLATFORM_OK;
}
