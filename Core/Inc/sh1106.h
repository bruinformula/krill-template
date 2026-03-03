/*
 * sh1106.h
 *
 *  Created on: Mar 2, 2026
 *      Author: Krishay Bhople
 */

#ifndef INC_SH1106_H_
#define INC_SH1106_H_

#include "stm32l5xx_hal.h"

#define SH1106_I2C_ADDR (0x3D << 1)
#define SH1106_WIDTH 128
#define SH1106_HEIGHT 64

#define SH1106_COLUMN_OFFSET 2

void SH1106_Init(I2C_HandleTypeDef *hi2c);
void SH1106_Fill(uint8_t color);
void SH1106_UpdateScreen(I2C_HandleTypeDef *hi2c);
void SH1106_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void SH1106_WriteChar(uint8_t x, uint8_t y, char ch, uint8_t size);
void SH1106_WriteString(uint8_t x, uint8_t y, const char *str, uint8_t size);


#endif /* INC_SH1106_H_ */
