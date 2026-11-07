#include "oled.h"
#include "main.h"
#include "spi.h"
#include <string.h>

/* The framebuffer uses SSD1306 page layout: bit 0 is the top pixel. */
static uint8_t oled_buffer[OLED_WIDTH * (OLED_HEIGHT / 8U)];

static HAL_StatusTypeDef OLED_Write(const uint8_t *data, uint16_t size,
                                    GPIO_PinState dc)
{
    HAL_GPIO_WritePin(LCD_NDC_GPIO_Port, LCD_NDC_Pin, dc);
    return HAL_SPI_Transmit(&hspi1, (uint8_t *)data, size, 100U);
}

HAL_StatusTypeDef OLED_Init(void)
{
    static const uint8_t init_commands[] = {
        0xAE,             /* display off */
        0xD5, 0x80,       /* clock */
        0xA8, 0x3F,       /* 64 multiplex lines */
        0xD3, 0x00, 0x40, /* offset and start line */
        0x8D, 0x14,       /* internal charge pump */
        0x20, 0x00,       /* horizontal addressing */
        0xA1, 0xC8,       /* segment and COM direction */
        0xDA, 0x12,       /* COM pin configuration */
        0x81, 0x7F,       /* contrast */
        0xD9, 0xF1, 0xDB, 0x40,
        0xA4, 0xA6, 0x2E /* RAM display, normal, scrolling off */
    };
    static const uint8_t display_on = 0xAF;
    HAL_StatusTypeDef status;

    /* CubeMX configures SPI1 at 42 MHz. Limit it to about 5.25 MHz. */
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    status = HAL_SPI_Init(&hspi1);
    if (status != HAL_OK) return status;

    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    status = OLED_Write(init_commands, sizeof(init_commands), GPIO_PIN_RESET);
    if (status != HAL_OK) return status;
    OLED_Clear();
    status = OLED_Update();
    if (status != HAL_OK) return status;
    return OLED_Write(&display_on, 1U, GPIO_PIN_RESET);
}

void OLED_Clear(void)
{
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

void OLED_Fill(void)
{
    memset(oled_buffer, 0xFF, sizeof(oled_buffer));
}

void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t on)
{
    uint16_t index;
    uint8_t mask;
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    index = (uint16_t)(y / 8U) * OLED_WIDTH + x;
    mask = (uint8_t)(1U << (y % 8U));
    if (on) oled_buffer[index] |= mask;
    else oled_buffer[index] &= (uint8_t)~mask;
}

void OLED_DrawBitmap(uint8_t x, uint8_t page, const uint8_t *data,
                     uint8_t width, uint8_t pages)
{
    uint16_t row, col;
    if (data == NULL || x >= OLED_WIDTH || page >= OLED_HEIGHT / 8U) return;
    for (row = 0; row < pages && page + row < OLED_HEIGHT / 8U; ++row) {
        for (col = 0; col < width && x + col < OLED_WIDTH; ++col) {
            oled_buffer[(page + row) * OLED_WIDTH + x + col] = data[row * width + col];
        }
    }
}

HAL_StatusTypeDef OLED_Update(void)
{
    static const uint8_t address[] = {0x21, 0, OLED_WIDTH - 1U,
                                      0x22, 0, OLED_HEIGHT / 8U - 1U};
    HAL_StatusTypeDef status = OLED_Write(address, sizeof(address), GPIO_PIN_RESET);
    if (status != HAL_OK) return status;
    return OLED_Write(oled_buffer, sizeof(oled_buffer), GPIO_PIN_SET);
}
