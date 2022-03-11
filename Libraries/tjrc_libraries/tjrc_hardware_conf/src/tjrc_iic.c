/*
 * tjrc_iic.c
 *
 *  Created on: 2022年2月23日
 *      Author: 11657
 */

/**
 * @brief 配置I2C总线
 * @return NONE
 */

#include "tjrc_iic.h"

IfxI2c_I2c i2c0_Handler;

/**
 * @brief 初始h化I2C接口
 * 
 */
void tjrc_setIic(void)
{
    rt_kprintf("[iic0]configure iic0 baudrate=400kHz sda=P13.2 scl=P13.1\r\n");
    const float32 I2C_BAUDRATE = 400000;
    /* 步骤1：定义配置结构体并拉取默认参数 */
    IfxI2c_I2c_Config i2cConfig;
    IfxI2c_I2c_initConfig(&i2cConfig, &MODULE_I2C0);

    /* 步骤2：配置输出引脚（因为芯片只有一组I2C的输出引脚，所以直接引用宏定义给出的引脚） */
    const IfxI2c_Pins MCP_PINS =
        {
            &IfxI2c0_SCL_P13_1_INOUT,
            &IfxI2c0_SDA_P13_2_INOUT,
            IfxPort_PadDriver_ttlSpeed1};
    i2cConfig.pins = &MCP_PINS;

    /* 步骤3：配置I2C总线的速率 */
    i2cConfig.baudrate = I2C_BAUDRATE;

    /* 步骤4：输入参数并且给到I2C总线的句柄（这里省略了中断之类的配置因为不需要） */
    IfxI2c_I2c_initModule(&i2c0_Handler, &i2cConfig);
}


