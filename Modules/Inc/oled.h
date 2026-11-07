#ifndef OLED_H_
#define OLED_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* ST7735S 0.96-inch module defaults; panel offsets vary by manufacturer. */
#define OLED_WIDTH       80U
#define OLED_HEIGHT      160U
#define OLED_X_OFFSET    26U
#define OLED_Y_OFFSET    1U
#define OLED_MADCTL      0xC8U

#define OLED_BLACK       0x0000U
#define OLED_WHITE       0xFFFFU
#define OLED_RED         0xF800U
#define OLED_GREEN       0x07E0U
#define OLED_BLUE        0x001FU
#define OLED_RGB565(r,g,b) ((uint16_t)((((uint16_t)(r) & 0xF8U) << 8) | \
                                      (((uint16_t)(g) & 0xFCU) << 3) | \
                                      ((uint16_t)(b) >> 3)))

/* Call after MX_GPIO_Init, MX_SPI1_Init and MX_TIM5_Init. */
HAL_StatusTypeDef OLED_Init(void);
HAL_StatusTypeDef OLED_Fill(uint16_t color);
HAL_StatusTypeDef OLED_Clear(void);
HAL_StatusTypeDef OLED_FillRect(uint16_t x, uint16_t y, uint16_t width,
                                uint16_t height, uint16_t color);
HAL_StatusTypeDef OLED_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
/* Row-major RGB565 pixels in host uint16_t format; the driver sends MSB first. */
HAL_StatusTypeDef OLED_DrawBitmap(uint16_t x, uint16_t y, uint16_t width,
                                  uint16_t height, const uint16_t *pixels);
#endif
