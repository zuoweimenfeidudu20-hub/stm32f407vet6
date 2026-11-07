#ifndef OLED_H_
#define OLED_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* SSD1306 128x64, four-wire SPI. Call after MX_GPIO_Init/MX_SPI1_Init. */
#define OLED_WIDTH 128U
#define OLED_HEIGHT 64U

HAL_StatusTypeDef OLED_Init(void);
void OLED_Clear(void);
void OLED_Fill(void);
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t on);
/* One byte per vertical group of eight pixels, least significant bit at top. */
void OLED_DrawBitmap(uint8_t x, uint8_t page, const uint8_t *data,
                     uint8_t width, uint8_t pages);
HAL_StatusTypeDef OLED_Update(void);

#endif
