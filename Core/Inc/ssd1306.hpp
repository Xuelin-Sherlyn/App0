
#include "main.h"
#include <stdint.h>

class SSD1306 {
private:
    #define SSD1306_CTRL_CMD    0x00
    #define SSD1306_CTRL_DATA   0x40
    #define SSD1306_I2C_ADDR    0x3C

    #define SSD1306_WIDTH       128
    #define SSD1306_HEIGHT      64
    #define SSD1306_PAGES       (SSD1306_HEIGHT/8)

    I2C_HandleTypeDef* hi2c;
    uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_PAGES];

    HAL_StatusTypeDef SSD1306_WriteCommand(uint8_t command);
    HAL_StatusTypeDef SSD1306_WriteCommands(uint8_t *commands, uint16_t len);
    HAL_StatusTypeDef SSD1306_WriteData(uint8_t *data, uint16_t len);

public:
    explicit SSD1306(I2C_HandleTypeDef* hi2cHandle);
    HAL_StatusTypeDef SSD1306_Init(void);
    HAL_StatusTypeDef SSD1306_SetDisplayArea(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
    void SSD1306_ClearBuffer(void);
    void SSD1306_UpdateScreen(void);
    void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
    void SSD1306_DrawHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color);
    void SSD1306_DrawVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color);
    void SSD1306_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
    void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
    void SSD1306_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t color);
    void SSD1306_DrawString(uint8_t x, uint8_t y, const char* str, uint8_t color);
};