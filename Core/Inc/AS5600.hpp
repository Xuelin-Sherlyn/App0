#include "stm32h7xx.h"
#include "stm32h7xx_hal_i2c.h"
#include <cmath>
#include <cstdint>

class AS5600{
private:
    #define AS5600_I2C_ADDRESS           0x36 // 根据AS5600的数据手册设置正确的地址
    #define AS5600_ANGLE_LOW_REG         0x0D // AS5600高位寄存器
    #define AS5600_ANGLE_HIGH_REG        0x0C // AS5600低位寄存器
    #define I2C_HANDEL_ERROR            32769 // 自定义的错误码

    I2C_HandleTypeDef* hi2c;
    uint8_t ReadByte(uint8_t regAddr);
    HAL_StatusTypeDef WriteByte(uint8_t regAddr, uint8_t data);
public:
    explicit AS5600(I2C_HandleTypeDef* hi2cHandle);
    HAL_StatusTypeDef Init();
    uint16_t ReadRAW();
    float ReadAngle();
};