
#include "main.h"
#include <stdint.h>

#ifndef USE_DECIMALS_DISPLAY_FILL_ZERO
#define USE_DECIMALS_DISPLAY_FILL_ZERO 0
#endif

class SSD1306 {
private:
    #define SSD1306_CTRL_CMD    0x00
    #define SSD1306_CTRL_DATA   0x40
    #define SSD1306_I2C_ADDR    0x3C

    #define SSD1306_WIDTH       128
    #define SSD1306_HEIGHT      64
    #define SSD1306_PAGES       (SSD1306_HEIGHT/8)

    I2C_HandleTypeDef* hi2c;
    uint8_t SSD1306_Display_Buffer[SSD1306_WIDTH * SSD1306_PAGES];

    HAL_StatusTypeDef WriteCommand(uint8_t command);
    HAL_StatusTypeDef WriteCommands(uint8_t *commands, uint16_t len);
    HAL_StatusTypeDef WriteData(uint8_t *data, uint16_t len);

public:
    explicit SSD1306(I2C_HandleTypeDef* hi2cHandle);
    HAL_StatusTypeDef Init(void);
    HAL_StatusTypeDef SetDisplayArea(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
    void ClearBuffer(void);
    void UpdateScreen(void);
    void DrawPixel(uint8_t x, uint8_t y, uint8_t color);
    void DrawHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color);
    void DrawVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color);
    void DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
    void FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
    void DrawChar(uint8_t x, uint8_t y, char ch, uint8_t color);
    void DrawString(uint8_t x, uint8_t y, const char* str, uint8_t color);
    void DrawNumber(uint8_t x, uint8_t y, int32_t num, uint8_t color);
    void DrawFloat(uint16_t x, uint16_t y, float decimals, uint8_t len, uint8_t decs, uint8_t color);
};