#include "oled.h"
#include "main.h"
#include "spi.h"
#include "tim.h"

/* Four-wire SPI, hardware NSS on PA4, D/C on PC4, reset on PC5. */
static HAL_StatusTypeDef write_bytes(const uint8_t *data, uint16_t count,
                                    GPIO_PinState dc)
{
    HAL_GPIO_WritePin(LCD_NDC_GPIO_Port, LCD_NDC_Pin, dc);
    return HAL_SPI_Transmit(&hspi1, (uint8_t *)data, count, 100U);
}

static HAL_StatusTypeDef command(uint8_t cmd, const uint8_t *data, uint16_t count)
{
    HAL_StatusTypeDef status = write_bytes(&cmd, 1U, GPIO_PIN_RESET);
    if (status != HAL_OK || count == 0U) return status;
    return write_bytes(data, count, GPIO_PIN_SET);
}

static HAL_StatusTypeDef set_window(uint16_t x, uint16_t y,
                                    uint16_t width, uint16_t height)
{
    uint16_t x0 = x + OLED_X_OFFSET, x1 = x0 + width - 1U;
    uint16_t y0 = y + OLED_Y_OFFSET, y1 = y0 + height - 1U;
    uint8_t bytes[4];
    HAL_StatusTypeDef status;
    bytes[0] = (uint8_t)(x0 >> 8); bytes[1] = (uint8_t)x0;
    bytes[2] = (uint8_t)(x1 >> 8); bytes[3] = (uint8_t)x1;
    status = command(0x2A, bytes, 4U);
    if (status != HAL_OK) return status;
    bytes[0] = (uint8_t)(y0 >> 8); bytes[1] = (uint8_t)y0;
    bytes[2] = (uint8_t)(y1 >> 8); bytes[3] = (uint8_t)y1;
    status = command(0x2B, bytes, 4U);
    if (status != HAL_OK) return status;
    return command(0x2C, NULL, 0U);
}

HAL_StatusTypeDef OLED_Init(void)
{
    static const uint8_t frame[] = {0x01, 0x2C, 0x2D};
    static const uint8_t frame3[] = {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D};
    static const uint8_t inv[] = {0x07};
    static const uint8_t power1[] = {0xA2, 0x02, 0x84};
    static const uint8_t power2[] = {0xC5};
    static const uint8_t power3[] = {0x0A, 0x00};
    static const uint8_t power4[] = {0x8A, 0x2A};
    static const uint8_t power5[] = {0x8A, 0xEE};
    static const uint8_t vcom[] = {0x0E};
    static const uint8_t madctl[] = {OLED_MADCTL};
    static const uint8_t format[] = {0x05};
    static const uint8_t gamma_pos[] = {0x02,0x1C,0x07,0x12,0x37,0x32,0x29,0x2D,0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10};
    static const uint8_t gamma_neg[] = {0x03,0x1D,0x07,0x06,0x2E,0x2C,0x29,0x2D,0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10};
    HAL_StatusTypeDef status;
#define SEND(c,d,n) do { status = command((c), (d), (n)); if (status != HAL_OK) return status; } while (0)
    /* CubeMX initializes SPI1 at 42 MHz; use about 5.25 MHz for the LCD. */
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    status = HAL_SPI_Init(&hspi1);
    if (status != HAL_OK) return status;
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(120);
    SEND(0x01, NULL, 0); HAL_Delay(150);
    SEND(0x11, NULL, 0); HAL_Delay(120);
    SEND(0xB1, frame, sizeof(frame));
    SEND(0xB2, frame, sizeof(frame));
    SEND(0xB3, frame3, sizeof(frame3));
    SEND(0xB4, inv, sizeof(inv));
    SEND(0xC0, power1, sizeof(power1));
    SEND(0xC1, power2, sizeof(power2));
    SEND(0xC2, power3, sizeof(power3));
    SEND(0xC3, power4, sizeof(power4));
    SEND(0xC4, power5, sizeof(power5));
    SEND(0xC5, vcom, sizeof(vcom));
    SEND(0x36, madctl, sizeof(madctl));
    SEND(0x3A, format, sizeof(format));
    SEND(0xE0, gamma_pos, sizeof(gamma_pos));
    SEND(0xE1, gamma_neg, sizeof(gamma_neg));
    SEND(0x21, NULL, 0); /* inversion: typical 80x160 ST7735S module */
    SEND(0x13, NULL, 0); HAL_Delay(10);
    SEND(0x29, NULL, 0); HAL_Delay(100);
#undef SEND
    status = OLED_Clear();
    if (status != HAL_OK) return status;
    /* PA3 is TIM5_CH4 in CubeMX: active-high backlight at full duty. */
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, __HAL_TIM_GET_AUTORELOAD(&htim5) + 1U);
    return HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

HAL_StatusTypeDef OLED_FillRect(uint16_t x, uint16_t y, uint16_t width,
                                uint16_t height, uint16_t color)
{
    uint8_t buffer[128];
    uint32_t remaining;
    uint16_t i, count;
    HAL_StatusTypeDef status;
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT || width == 0U || height == 0U)
        return HAL_ERROR;
    if (width > OLED_WIDTH - x) width = OLED_WIDTH - x;
    if (height > OLED_HEIGHT - y) height = OLED_HEIGHT - y;
    status = set_window(x, y, width, height);
    if (status != HAL_OK) return status;
    for (i = 0; i < sizeof(buffer); i += 2U) {
        buffer[i] = (uint8_t)(color >> 8); buffer[i + 1U] = (uint8_t)color;
    }
    remaining = (uint32_t)width * height;
    while (remaining != 0U) {
        count = remaining > sizeof(buffer) / 2U ? sizeof(buffer) / 2U : (uint16_t)remaining;
        status = write_bytes(buffer, (uint16_t)(count * 2U), GPIO_PIN_SET);
        if (status != HAL_OK) return status;
        remaining -= count;
    }
    return HAL_OK;
}

HAL_StatusTypeDef OLED_Fill(uint16_t color) { return OLED_FillRect(0, 0, OLED_WIDTH, OLED_HEIGHT, color); }
HAL_StatusTypeDef OLED_Clear(void) { return OLED_Fill(OLED_BLACK); }
HAL_StatusTypeDef OLED_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{ return OLED_FillRect(x, y, 1U, 1U, color); }

HAL_StatusTypeDef OLED_DrawBitmap(uint16_t x, uint16_t y, uint16_t width,
                                  uint16_t height, const uint16_t *pixels)
{
    uint8_t buffer[128];
    uint16_t row, col, used = 0, clipped_width, clipped_height;
    HAL_StatusTypeDef status;
    if (pixels == NULL || x >= OLED_WIDTH || y >= OLED_HEIGHT || width == 0U || height == 0U)
        return HAL_ERROR;
    clipped_width = width > OLED_WIDTH - x ? OLED_WIDTH - x : width;
    clipped_height = height > OLED_HEIGHT - y ? OLED_HEIGHT - y : height;
    status = set_window(x, y, clipped_width, clipped_height);
    if (status != HAL_OK) return status;
    for (row = 0; row < clipped_height; ++row) {
        for (col = 0; col < clipped_width; ++col) {
            uint16_t color = pixels[(uint32_t)row * width + col];
            buffer[used++] = (uint8_t)(color >> 8);
            buffer[used++] = (uint8_t)color;
            if (used == sizeof(buffer)) {
                status = write_bytes(buffer, used, GPIO_PIN_SET);
                if (status != HAL_OK) return status;
                used = 0;
            }
        }
    }
    return used ? write_bytes(buffer, used, GPIO_PIN_SET) : HAL_OK;
}
