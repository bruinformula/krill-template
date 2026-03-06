/*
 * eeprom.h
 *
 *  Created on: Mar 5, 2026
 *      Author: Jonathan Jiang
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "main.h"

#define EEPROM_PAGE_SIZE 8   // The MAX amount of bytes you can write at once
#define EEPROM_TIMEOUT   100 // 100ms timeout for safety

/**
  * @brief  Initializes the EEPROM driver with the specific I2C bus and device address.
  * Must be called once in main() before using Read or Write functions.
  * @param  hi2c: Pointer to a I2C_HandleTypeDef structure (e.g., &hi2c1).
  * @param  dev_addr: The 8-bit shifted I2C address of the EEPROM (e.g., 0x50 << 1).
  * @retval None
  */
void EEPROM_Init(I2C_HandleTypeDef *hi2c, uint16_t dev_addr);

/**
  * @brief  Writes an array of bytes to the EEPROM.
  * Automatically handles the mandatory 10ms write delay.
  * @param  mem_addr: The internal starting memory address to write to (e.g., 0x0000 to 0x00FF).
  * @param  data: Pointer to the array or variable containing the data to be written.
  * @param  size: Number of bytes to write. Must NOT exceed EEPROM_PAGE_SIZE (8 bytes).
  * @retval HAL_StatusTypeDef: HAL_OK if successful, HAL_ERROR if size is too large or bus fails.
  */
HAL_StatusTypeDef EEPROM_Write(uint16_t mem_addr, uint8_t *data, uint16_t size);

/**
  * @brief  Reads an array of bytes from the EEPROM into a buffer.
  * @param  mem_addr: The internal starting memory address to read from.
  * @param  buffer: Pointer to the empty array where the incoming data will be saved.
  * @param  size: Number of bytes to read (no page limit for reading).
  * @retval HAL_StatusTypeDef: HAL_OK if successful, HAL_ERROR if bus fails.
  */
HAL_StatusTypeDef EEPROM_Read(uint16_t mem_addr, uint8_t *buffer, uint16_t size);

#endif /* INC_EEPROM_H_ */
