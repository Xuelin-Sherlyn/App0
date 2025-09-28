#include "ssd1306.hpp"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_i2c.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string.h>
#include <sys/_intsup.h>

SSD1306::SSD1306(I2C_HandleTypeDef* hi2cHandle) : hi2c(hi2cHandle){}

/**
  * @brief  SSD1306初始化函数
  * @retval HAL状态
  */
HAL_StatusTypeDef SSD1306::Init(void) {
    uint8_t init_cmds[] = {
    0xAE,           // 关闭显示
    0xD5, 0x80, // 设置显示时钟分频/振荡器频率
    0xA8, 0x3F, // 设置多路复用率（1/64）
    0xD3, 0x00, // 设置显示偏移
    0x40,           // 设置显示起始行
    0x8D, 0x14, // 启用电荷泵
    0x20, 0x00, // 设置内存模式（水平寻址）
    0xA1,       // 段重映射（SEG0->SEG127）
    0xC8,       // 扫描方向（COM0->COM63）
    0xDA, 0x12, // COM引脚硬件配置
    0x81, 0xCF, // 设置对比度
    0xD9, 0xF1, // 设置预充电周期
    0xDB, 0x40, // 设置VCOMH电平
    0xA4,       // 全部像素点亮（禁用）
    0xA6,       // 正常显示（非反色）
    0xAF        // 开启显示
    };
    return WriteCommands(init_cmds, sizeof(init_cmds));
}

/**
  * @brief  向SSD1306写入单个命令
  * @param  command: 要写入的命令字节
  * @retval HAL状态
  */
HAL_StatusTypeDef SSD1306::WriteCommand(uint8_t command) {
    uint8_t buf[2] = {SSD1306_CTRL_CMD, command};
    return HAL_I2C_Master_Transmit(hi2c, SSD1306_I2C_ADDR << 1, buf, sizeof(buf), HAL_MAX_DELAY);
}

/**
  * @brief  向SSD1306写入多个命令
  * @param  commands: 命令数组指针
  * @param  len: 命令数量
  * @retval HAL状态
  */
HAL_StatusTypeDef SSD1306::WriteCommands(uint8_t *commands, uint16_t len) {
        // 显式类型转换解决编译错误
    uint8_t *buf = (uint8_t*)malloc(len + 1);
    if (buf == NULL) return HAL_ERROR;
    
    buf[0] = SSD1306_CTRL_CMD;
    memcpy(buf + 1, commands, len);
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, SSD1306_I2C_ADDR << 1, buf, len + 1, HAL_MAX_DELAY);
    free(buf);
    return status;
}

/**
  * @brief  向SSD1306写入数据
  * @param  data: 数据数组指针
  * @param  len: 数据长度
  * @retval HAL状态
  */
HAL_StatusTypeDef SSD1306::WriteData(uint8_t *data, uint16_t len) {
    uint8_t *buf = (uint8_t*)malloc(len + 1);
    if (buf == NULL) return HAL_ERROR;
    
    buf[0] = SSD1306_CTRL_DATA;
    memcpy(buf + 1, data, len);
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, SSD1306_I2C_ADDR << 1, buf, len + 1, HAL_MAX_DELAY);
    free(buf);
    return status;    
}

/**
  * @brief  设置显示区域
  * @param  x: 起始列 (0-127)
  * @param  y: 起始页 (0-7)
  * @param  w: 宽度
  * @param  h: 高度(页数)
  * @retval HAL状态
  */
HAL_StatusTypeDef SSD1306::SetDisplayArea(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    uint8_t cmds[] = {
        0x21, // 设置列地址
        x,    // 起始列
        static_cast<uint8_t>(x + w - 1), // 结束列
        0x22, // 设置页地址
        y,    // 起始页
        static_cast<uint8_t>(y + h - 1)  // 结束页
    };
    return WriteCommands(cmds, sizeof(cmds));
}

/**
  * @brief  设置SSD1306的字体 
  */
void SSD1306::SetFont(pFONT *font)
{
  switch (font->FontType)
  {
    // ASCII Font
    case FONT_TYPE_ASCII:
    ASCII_Font = font;
    break;

    // Chinese Font
    case FONT_TYPE_GBK:
    Chinese_Font = font;
    break;
  }
}

/**
  * @brief  清除显示缓冲区
  */
void SSD1306::ClearBuffer(void)
{
    memset(SSD1306_Display_Buffer, 0, sizeof(SSD1306_Display_Buffer));
}

/**
  * @brief  更新屏幕显示
  */
void SSD1306::UpdateScreen(void)
{
    SetDisplayArea(0, 0, SSD1306_WIDTH, SSD1306_PAGES);
    WriteData(SSD1306_Display_Buffer, sizeof(SSD1306_Display_Buffer));
}

/**
  * @brief  在缓冲区中绘制一个像素
  * @param  x: X坐标 (0-127)
  * @param  y: Y坐标 (0-63)
  * @param  color: 颜色 (0:清除, 1:设置)
  */
void SSD1306::DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    
    if (color) {
        SSD1306_Display_Buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    } else {
        SSD1306_Display_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

/**
  * @brief  绘制水平线
  * @param  x: 起始X坐标
  * @param  y: Y坐标
  * @param  w: 宽度
  * @param  color: 颜色
  */
void SSD1306::DrawHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color)
{
    for (uint8_t i = 0; i < w; i++) {
        DrawPixel(x + i, y, color);
    }
}

/**
  * @brief  绘制垂直线
  * @param  x: X坐标
  * @param  y: 起始Y坐标
  * @param  h: 高度
  * @param  color: 颜色
  */
void SSD1306::DrawVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color)
{
    for (uint8_t i = 0; i < h; i++) {
        DrawPixel(x, y + i, color);
    }
}

/**
  * @brief  绘制矩形
  * @param  x: 起始X坐标
  * @param  y: 起始Y坐标
  * @param  w: 宽度
  * @param  h: 高度
  * @param  color: 颜色
  */
void SSD1306::DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    DrawHLine(x, y, w, color);
    DrawHLine(x, y + h - 1, w, color);
    DrawVLine(x, y, h, color);
    DrawVLine(x + w - 1, y, h, color);
}

/**
  * @brief  绘制实心矩形
  * @param  x: 起始X坐标
  * @param  y: 起始极市 Y坐标
  * @param  w: 宽度
  * @param  h: 高度
  * @极市 param  color: 颜色
  */
void SSD1306::FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    for (uint8_t i = 0; i < h; i++) {
        DrawHLine(x, y + i, w, color);
    }
}

/**
  * @brief  绘制一个字符
  * @param  x: X坐标
  * @param  y: Y坐标
  * @param  ch: 要绘制的字符
  * @param  color: 颜色 (0:清除, 1:设置)
  */
void SSD1306::DrawChar(uint8_t x, uint8_t y, char ch, uint8_t color)
{
    // 只处理可打印字符
    if (ch < 32 || ch > 127) return;
    
    // 计算字符在字体数组中的偏移量
    uint16_t char_offset = (ch - 32) * ASCII_Font->Sizes;
    
    // 计算每行需要的字节数
    uint8_t bytes_per_row = (ASCII_Font->Width + 7) / 8;
    
    // 绘制字符（固定为阴码、逐行式、高位在前）
    for (uint8_t row = 0; row < ASCII_Font->Height; row++) {
        for (uint8_t byte_idx = 0; byte_idx < bytes_per_row; byte_idx++) {
            uint8_t line_byte = ASCII_Font->pTable[char_offset + row * bytes_per_row + byte_idx];
            uint8_t start_col = byte_idx * 8;
            
            for (uint8_t bit = 0; bit < 8; bit++) {
                uint8_t col = start_col + bit;
                if (col >= ASCII_Font->Width) break;
                
                // 高位在前：从最高位(bit7)开始检查
                if (line_byte & (1 << (7 - bit))) {
                    DrawPixel(x + col, y + row, color);
                }
            }
        }
    }
}

/**
  * @brief  绘制字符串
  * @param  x: 起始X坐标
  * @param  y: 起始Y坐标
  * @param  str: 要绘制的字符串
  * @param  color: 颜色 (0:清除, 1:设置)
  */
void SSD1306::DrawString(uint8_t x, uint8_t y, const char* str, uint8_t color)
{
    uint8_t pos = x;
    while (*str) {
        DrawChar(pos, y, *str++, color);
        pos += ASCII_Font->Width; // 字符宽度(8像素) + 间距(1像素)
        if (pos + ASCII_Font->Width > SSD1306_WIDTH) break; // 防止溢出屏幕
    }
}

/**
  * @brief  绘制数字
  * @param  x: 起始X坐标
  * @param  y: 起始Y坐标
  * @param  num: 要绘制的数字
  * @param  color: 颜色 (0:清除, 1:设置)
  */
void SSD1306::DrawNumber(uint8_t x, uint8_t y, int32_t num, uint8_t color)
{
    char buffer[12];
    snprintf(buffer, sizeof(buffer), "%ld", (long)num);
    DrawString(x, y, buffer, color);
}

/**
  * @brief  绘制浮点数
  * @param  x: X坐标
  * @param  y: Y坐标
  * @param  decimals: 要绘制的浮点数
  * @param  len: 小数长度
  * @param  decs: 小数位数
  * @param  color: 颜色 (0:清除, 1:设置)
  */
void SSD1306::DrawFloat(uint16_t x, uint16_t y, float decimals, uint8_t len, uint8_t decs, uint8_t color)
{
    char buffer[20];

    #if USE_DECIMALS_DISPLAY_FILL_ZERO
    snprintf(buffer, sizeof(buffer), "%0*.*lf", len, decs, decimals);
    #else
    snprintf(buffer, sizeof(buffer), "%*.*lf", len, decs, decimals);
    #endif
    
    DrawString(x, y, buffer, color);
}

/**
 * @brief 显示单个中文字符
 * @param x: 字符左上角X坐标
 * @param y: 字符左上角Y坐标
 * @param ch: 中文字符
 */
void SSD1306::DrawChineseChar(uint8_t x, uint8_t y, const char* ch, uint8_t color)
{
    if (ch == nullptr || Chinese_Font == nullptr) return;
    
    // 只处理可打印字符
    if (ch[0] == '\0' || ch[1] == '\0') return;
    
    // 字符编码到字库索引的映射
    uint16_t char_index = 0;
    uint16_t buffer_index = 0;

    // 通过对比数组中的汉字编码，定位字模地址
    while(1) {		
        // 检查字符编码是否匹配
        if (*(Chinese_Font->pTable + (buffer_index + 1) * Chinese_Font->Sizes + 0) == *ch && 
            *(Chinese_Font->pTable + (buffer_index + 1) * Chinese_Font->Sizes + 1) == *(ch + 1)) {   
            char_index = buffer_index; // 字模地址偏移
            break;
        }				
        buffer_index += 2; // 每个中文字符占两字节

        if(buffer_index >= Chinese_Font->Table_Rows) return; // 字模列表没这个字
    }
    
    // 计算字模数据偏移量
    const uint8_t* char_data = Chinese_Font->pTable + char_index * Chinese_Font->Sizes;
    
    // 边界检查
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    if (x + Chinese_Font->Width > SSD1306_WIDTH || y + Chinese_Font->Height > SSD1306_HEIGHT) return;
    
    // 计算每行需要的字节数
    uint8_t bytes_per_row = (Chinese_Font->Width + 7) / 8;
    
    // SSD1306使用逐像素绘制，不需要缓冲区
    // 遍历每一行
    for(uint8_t row = 0; row < Chinese_Font->Height; row++) {
        // 遍历每行的每个字节
        for(uint8_t byte_idx = 0; byte_idx < bytes_per_row; byte_idx++) {
            uint8_t line_byte = char_data[row * bytes_per_row + byte_idx];
            uint8_t start_col = byte_idx * 8;
            
            // 遍历每个字节的每个位
            for(uint8_t bit = 0; bit < 8; bit++) {
                uint8_t col = start_col + bit;
                if (col >= Chinese_Font->Width) break;
                
                // 检查像素点是否应该点亮（阴码，高位在前）
                if (line_byte & (1 << (7 - bit))) {
                    DrawPixel(x + col, y + row, color);  // 点亮像素
                } 
            }
        }
    }
}

/**
 * @brief 显示中文字符串
 * @param x: 字符串起始X坐标
 * @param y: 字符串起始Y坐标
 * @param str: 中文字符串
 * @note 可以在字符间添加像素间隔，修改 spacing 参数即可
 */
void SSD1306::DrawChineseString(uint8_t x, uint8_t y, const char* str, uint8_t color)
{
    if (str == nullptr || Chinese_Font == nullptr) return;
    
    uint8_t currentX = x;
    uint8_t currentY = y;
     uint8_t spacing = 0;
    
    while (*str != '\0') {
        // 检查是否是中文字符（GB2312编码范围）
        uint8_t current_byte = (uint8_t)*str;
        
        if (current_byte >= 0xA1 && current_byte <= 0xF7) {
            // 确保有完整的双字节
            if (*(str + 1) == '\0') break;
            
            // 显示中文字符
            DrawChineseChar(currentX, currentY, str, color);
            
            // 移动到下一个字符位置
            currentX += Chinese_Font->Width + spacing;
            str += 2; // 跳过2个字节（中文字符）
        } 
        else if (current_byte >= 0x20 && current_byte <= 0x7E) {
            // ASCII字符 - 使用现有的DrawChar函数
            DrawChar(currentX, currentY, *str, color);
            currentX += ASCII_Font->Width + spacing;
            str += 1; // 跳过1个字节（ASCII字符）
        }
        else {
            // 其他字符跳过
            str += 1;
        }
        
        // 换行检查
        if (currentX + fmax(Chinese_Font->Width, ASCII_Font->Width) > SSD1306_WIDTH) {
            currentX = x;
            currentY += fmax(Chinese_Font->Height, ASCII_Font->Height) + 1;
            
            if (currentY + fmax(Chinese_Font->Height, ASCII_Font->Height) > SSD1306_HEIGHT) {
                break;
            }
        }
    }
}