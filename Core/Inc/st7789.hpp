#include "main.h"
#include "stdint.h"
#include "stm32h7xx_hal_spi.h"
#include <cstdint>

class ST7789{
private:
    SPI_HandleTypeDef* hspi;
    uint16_t ForgColor = 0x0000;
    uint16_t BackColor = 0x0000;
    uint16_t LCD_Buff[1024];

    HAL_StatusTypeDef ST7789_WriteCommand(uint8_t command);
    HAL_StatusTypeDef ST7789_WriteCommands(uint8_t *commands, uint16_t len);
    HAL_StatusTypeDef ST7789_WriteData_8bit(uint8_t data);
    HAL_StatusTypeDef ST7789_WriteData_16bit(uint16_t data);
    HAL_StatusTypeDef ST7789_SPI_Transmit(SPI_HandleTypeDef *hspi,uint16_t pData, uint32_t Size);
    HAL_StatusTypeDef ST7789_SPI_TransmitBuffer (SPI_HandleTypeDef *hspi, uint16_t *pData, uint32_t Size);
    HAL_StatusTypeDef ST7789_SPI_WaitOnFlagUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status,
                                                    uint32_t Tickstart, uint32_t Timeout);
    void ST7789_SPI_CloseTransfer(SPI_HandleTypeDef *hspi);
    void ST7789_WriteBuff(uint16_t *DataBuff, uint16_t DataSize);
    void ST7789_SetAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);

public:
    explicit ST7789(SPI_HandleTypeDef* hspiHandle);
    HAL_StatusTypeDef LCD_Init(void);
    void ST7789_SetColor(uint32_t color);
    void ST7789_SetBackColor(uint32_t color);
    void ST7789_Clear(void);
    void ST7789_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);
    void ST7789_DrawImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pImage);
    void ST7789_CopyBuffer(uint16_t x, uint16_t y,uint16_t width,uint16_t height,uint16_t *DataBuff);
};