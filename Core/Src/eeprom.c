/*
 * eeprom.c
 *
 *  Created on: Mar 5, 2026
 *      Author: Jonathan Jiang
 */


#include "eeprom.h"


static I2C_HandleTypeDef *eeprom_i2c;
static uint16_t eeprom_addr;

void EEPROM_Init(I2C_HandleTypeDef *hi2c, uint16_t dev_addr)
{
    eeprom_i2c = hi2c;
    eeprom_addr = dev_addr;
}

HAL_StatusTypeDef EEPROM_Write(uint16_t mem_addr, uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef status = HAL_OK;

    uint16_t current_addr = mem_addr;   // Where we are currently writing
    uint8_t *current_data = data;       // Pointer to the current chunk of data
    uint16_t remaining_size = size;     // How many bytes are left to write

    while (remaining_size > 0)
    {
        // 1. Math Magic: Find out how much space is left on the CURRENT page.
        // The modulo operator (%) finds our exact position inside the 8-byte page.
        uint16_t space_on_page = EEPROM_PAGE_SIZE - (current_addr % EEPROM_PAGE_SIZE);

        // 2. Decide chunk size: If we have less data than the space remaining,
        // just write the data. Otherwise, fill the rest of the page.
        uint16_t chunk_size = (remaining_size < space_on_page) ? remaining_size : space_on_page;

        // 3. Command the hardware to write just this chunk
        status = HAL_I2C_Mem_Write(eeprom_i2c, eeprom_addr, current_addr,
                                   I2C_MEMADD_SIZE_8BIT, current_data, chunk_size, EEPROM_TIMEOUT);

        // If the bus crashes or the chip gets unplugged, stop and report the error
        if (status != HAL_OK) return status;

        HAL_Delay(10);

        // 5. Move all our pointers forward for the next loop iteration
        current_addr += chunk_size;     // Move the "Apartment" number up
        current_data += chunk_size;     // Move further down our data array
        remaining_size -= chunk_size;   // Cross these bytes off our to-do list
    }

    return HAL_OK;
}

HAL_StatusTypeDef EEPROM_Read(uint16_t mem_addr, uint8_t *buffer, uint16_t size)
{
    // Note: Reading doesn't have a page limit. You can read the whole chip at once
    return HAL_I2C_Mem_Read(eeprom_i2c, eeprom_addr, mem_addr,
                            I2C_MEMADD_SIZE_8BIT, buffer, size, EEPROM_TIMEOUT);
}
