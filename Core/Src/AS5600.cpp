#include "AS5600.hpp"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_i2c.h"
#include <cmath>
#include <cstdint>

AS5600::AS5600(I2C_HandleTypeDef* hi2cHandle) : hi2c(hi2cHandle){}

HAL_StatusTypeDef AS5600::Init()
{
	uint8_t configData;

	// 确保I2C句柄有效
	if (hi2c == NULL) {
		return HAL_ERROR;
	}

	// 步骤1: 复位AS5600（如果需要，但通常不是通过I2C直接复位）
	// 注意：AS5600没有直接通过I2C复位的命令，复位通常是通过硬件完成的

	// 步骤2: 配置基本参数（如果需要，这里仅作为示例设置电源模式和输出阶段）
	// 设置电源模式为NOM（正常模式），输出阶段为模拟输出
	configData = 0x00; // PM[1:0] = 00, OUTS[1:0] = 00（根据需求调整）
	return HAL_I2C_Mem_Write(hi2c, AS5600_I2C_ADDRESS << 1, 0x07, I2C_MEMADD_SIZE_8BIT, &configData, 1, HAL_MAX_DELAY);

	// 其他可能的配置，如设置PWM频率、滤波器设置等
	// ...

	// 步骤3: 验证配置（可选，通过读取寄存器来确认设置）
	// HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDRESS << 1, 0x07, I2C_MEMADD_SIZE_8BIT, &configData, 1, HAL_MAX_DELAY);
	// 检查configData是否包含预期的配置...

	// 注意：在实际应用中，你可能需要根据AS5600的具体需求调整上述配置步骤
}

uint8_t AS5600::ReadByte(uint8_t regAddr) {
    uint8_t data;

    HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDRESS << 1, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    return data;
}

HAL_StatusTypeDef AS5600::WriteByte(uint8_t regAddr, uint8_t data) {
    return HAL_I2C_Mem_Write(hi2c, AS5600_I2C_ADDRESS << 1, regAddr, 1, &data, 1, HAL_MAX_DELAY);
}

uint16_t AS5600::ReadRAW() {
    uint8_t lowByte, highByte;
    uint16_t angle;

    // 确保I2C句柄有效
    if (hi2c == NULL) {
        return I2C_HANDEL_ERROR; // 或者你可以设置一个错误码
    }

    // 读取低8位
    HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDRESS << 1, AS5600_ANGLE_LOW_REG, I2C_MEMADD_SIZE_8BIT, &lowByte, 1, HAL_MAX_DELAY);
    // 读取高8位
    HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDRESS << 1, AS5600_ANGLE_HIGH_REG, I2C_MEMADD_SIZE_8BIT, &highByte, 1, HAL_MAX_DELAY);

    // 组合高低字节形成16位角度值
    angle = (highByte << 8) | lowByte;

    // 注意：如果需要，你可以将16位角度值转换为实际的度数（例如，angle * 360 / 4096）

    return angle;
}

float AS5600::ReadAngle()
{
    return ReadRAW()*360/4096.0;
}
