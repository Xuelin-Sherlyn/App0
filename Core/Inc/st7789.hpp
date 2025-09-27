#include "main.h"
#include "stdint.h"
#include "stm32h7xx_hal_spi.h"
#include <cstdint>
#include "display_font.h"

#ifndef USE_DECIMALS_DISPLAY_FILL_ZERO
#define USE_DECIMALS_DISPLAY_FILL_ZERO 0
#endif

class ST7789{
private:
    #define LCD_WIDTH  320
    #define LCD_HEIGHT 240

    SPI_HandleTypeDef* hspi;
    uint16_t ForgColor = 0x0000;
    uint16_t BackColor = 0x0000;
    uint16_t ST7789_Display_Buffer[1024];
    pFONT* ASCII_Font;

    HAL_StatusTypeDef WriteCommand(uint8_t command);
    HAL_StatusTypeDef WriteCommands(uint8_t *commands, uint16_t len);
    HAL_StatusTypeDef WriteData_8bit(uint8_t data);
    HAL_StatusTypeDef WriteData_16bit(uint16_t data);
    HAL_StatusTypeDef SPI_Transmit(SPI_HandleTypeDef *hspi, uint16_t pData, uint32_t Size);
    HAL_StatusTypeDef SPI_TransmitBuffer (SPI_HandleTypeDef *hspi, uint16_t *pData, uint32_t Size);
    HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status,
                                                    uint32_t Tickstart, uint32_t Timeout);
    void SPI_CloseTransfer(SPI_HandleTypeDef *hspi);
    void WriteBuff(uint16_t *DataBuff, uint16_t DataSize);
    void SetAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);

public:
    explicit ST7789(SPI_HandleTypeDef* hspiHandle);
    HAL_StatusTypeDef Init(void);
    void SetColor(uint32_t color);
    void SetBackColor(uint32_t color);
    void SetFont(pFONT *font);
    void Clear(void);
    void FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);
    void DrawImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pImage);
    void CopyBuffer(uint16_t x, uint16_t y,uint16_t width,uint16_t height,uint16_t *DataBuff);
    void DrawChar(uint16_t x, uint16_t y, char ch);
    void DrawString(uint16_t x, uint16_t y, const char* str);
    void DrawNumber(uint8_t x, uint8_t y, int32_t num);
    void DrawFloat(uint16_t x, uint16_t y, float decimals, uint8_t len, uint8_t decs);
};