#include "main.h"
#include "stdint.h"
#include "stm32h7xx_hal_spi.h"
#include <cstdint>
#include "display_font.h"

#ifndef USE_DECIMALS_DISPLAY_FILL_ZERO
#define USE_DECIMALS_DISPLAY_FILL_ZERO 0
#endif

    // 滚动方向枚举
    typedef enum {
    SCROLL_LEFT = 0,
    SCROLL_RIGHT,
    SCROLL_UP, 
    SCROLL_DOWN
    } ScrollDirection_t;

    // 图片信息结构体
    typedef struct {
    uint16_t width;
    uint16_t height;
    uint8_t *data;        // RGB565格式数据指针
    uint32_t data_size;    // 数据大小（字节）
    } Image_t;


class ST7789{
private:
    #define ST7789_WIDTH  320
    #define ST7789_HEIGHT 240
    static const uint32_t BUFFER_SIZE = ST7789_WIDTH * ST7789_HEIGHT;
    
    SPI_HandleTypeDef* hspi;
    uint16_t ForgColor = 0x0000;
    uint16_t BackColor = 0x0000;
    uint16_t ST7789_Display_Buffer[1024];
    pFONT* ASCII_Font;
    pFONT* Chinese_Font;

    // 滚动相关变量
    int16_t scroll_offset_x = 0;
    int16_t scroll_offset_y = 0;
    uint8_t current_image_index = 0;
    uint8_t total_images = 0;
    uint16_t scroll_speed = 2;
    ScrollDirection_t scroll_direction = SCROLL_LEFT;
    uint8_t is_scrolling = 0;
    Image_t* image_list = nullptr;
    uint16_t* double_buffer[2];
    uint8_t active_buffer = 0;

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

    // 滚动相关私有函数
    uint16_t GetPixelColor(Image_t* img, int16_t x, int16_t y);
    void RenderScrollFrame(void);
    void SwapBuffers(void);

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
    void DrawChineseChar(uint16_t x, uint16_t y, const char* ch);
    void DrawChineseString(uint16_t x, uint16_t y, const char* str);

    // 新增的图片滚动函数
    uint8_t LoadImages(Image_t* images, uint8_t count);
    void StartScroll(ScrollDirection_t direction, uint16_t speed);
    void StopScroll(void);
    void UpdateScroll(void);  // 在主循环中调用
    void SetScrollSpeed(uint16_t speed);
    void SetScrollDirection(ScrollDirection_t direction);
    void NextImage(void);
    void PreviousImage(void);
    uint8_t GetCurrentImageIndex(void);
    uint8_t GetTotalImages(void);
};