/**
 * @file tjrc_icm20602.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_icm20602.h"

extern IfxI2c_I2c i2c0_Handler;

/**
 * @brief i2c设备MPU6050句柄
 */
IfxI2c_I2c_Device icm20602_Handler;

/**
 * @brief 加速度计，陀螺仪原始数据的全局变量
 */
int16_t icm_gyro_x,icm_gyro_y,icm_gyro_z;
int16_t icm_acc_x,icm_acc_y,icm_acc_z;

static void tjrc_icm20602_writeByte(uint8_t regAddr, uint8_t dataBuff);
static uint8_t tjrc_icm20602_readByte(uint8_t regAddr);

void tjrc_setIcm20602(void)
{
    /* 配置ICM20602设备句柄 */
    icm20602_Handler.i2c = &i2c0_Handler;
    icm20602_Handler.deviceAddress = ICM20602_DEV_ADDR << 1;

    if(tjrc_icm20602_readByte(ICM20602_WHO_AM_I)!=0x12)
    {
        rt_kprintf("[icm20602*]cannot find imu\r\n");
    }
    rt_kprintf("[icm20602]detect imu: icm20602\r\n");
    /* 解除休眠状态 */
    tjrc_icm20602_writeByte(ICM20602_PWR_MGMT_1, 0x80);
    rt_thread_mdelay(2);
    /* 等待复位完成 */
    while(0x80 & tjrc_icm20602_readByte(ICM20602_PWR_MGMT_1));

    /* 配置参数 */
    tjrc_icm20602_writeByte(ICM20602_PWR_MGMT_1, 0x01);             //时钟设置
    tjrc_icm20602_writeByte(ICM20602_PWR_MGMT_2, 0x00);             //开启陀螺仪和加速度计
    tjrc_icm20602_writeByte(ICM20602_CONFIG, 0x01);                 //176HZ 1KHZ
    tjrc_icm20602_writeByte(ICM20602_SMPLRT_DIV, 0x07);             //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    tjrc_icm20602_writeByte(ICM20602_GYRO_CONFIG, 0x18);            //±2000 dps
    tjrc_icm20602_writeByte(ICM20602_ACCEL_CONFIG, 0x10);           //±8g
    tjrc_icm20602_writeByte(ICM20602_ACCEL_CONFIG_2, 0x03);         //Average 4 samples   44.8HZ   //0x23 Average 16 samples

    tjrc_icm20602_writeByte(ICM20602_INT_PIN_CFG, 0x9c);  //0x02
    tjrc_icm20602_writeByte(ICM20602_INT_ENABLE, 0x01);  //数据准备就绪中断
    /* 配置中断信号输入 */
    tjrc_initEru(&IfxScu_REQ1_P15_8_IN);
    rt_kprintf("[icm20602]configurate imu: icm20602\r\n");
}


void tjrc_icm20602_getAccel(void)
{
    uint8_t dat[6];

    dat[0] = tjrc_icm20602_readByte(ICM20602_ACCEL_XOUT_H);
    dat[1] = tjrc_icm20602_readByte(ICM20602_ACCEL_XOUT_L);
    dat[2] = tjrc_icm20602_readByte(ICM20602_ACCEL_YOUT_H);
    dat[3] = tjrc_icm20602_readByte(ICM20602_ACCEL_YOUT_L);
    dat[4] = tjrc_icm20602_readByte(ICM20602_ACCEL_ZOUT_H);
    dat[5] = tjrc_icm20602_readByte(ICM20602_ACCEL_ZOUT_L);

    icm_acc_x = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    icm_acc_y = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    icm_acc_z = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
}


void tjrc_icm20602_getGyro(void)
{
    uint8_t dat[6];

    dat[0] = tjrc_icm20602_readByte(ICM20602_GYRO_XOUT_H);
    dat[1] = tjrc_icm20602_readByte(ICM20602_GYRO_XOUT_L);
    dat[2] = tjrc_icm20602_readByte(ICM20602_GYRO_YOUT_H);
    dat[3] = tjrc_icm20602_readByte(ICM20602_GYRO_YOUT_L);
    dat[4] = tjrc_icm20602_readByte(ICM20602_GYRO_ZOUT_H);
    dat[5] = tjrc_icm20602_readByte(ICM20602_GYRO_ZOUT_L);

    icm_gyro_x = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    icm_gyro_y = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    icm_gyro_z = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
}

/**
 * @brief 向ICM20602的某一寄存器中写入一字节数据
 * @param regAddr      (uint8)寄存器地址
 * @param dataBuff     (uint8)待写入字节
 * @return NONE
 */
static void tjrc_icm20602_writeByte(uint8_t regAddr, uint8_t dataBuff)
{
    /* 设置一个超时变量防止卡死在while里 */
    uint32_t timeOut = 0;
    volatile uint8_t writebuff[2];
    writebuff[0] = regAddr;
    writebuff[1] = dataBuff;
    while (timeOut++ < 500 && IfxI2c_I2c_write(&icm20602_Handler, (uint8_t *)&writebuff, 2) == IfxI2c_I2c_Status_nak)
        ;

}

/**
 * @brief 从MPU6050的某一寄存器读取一字节数据
 * @param i2cDevice    (IfxI2c_I2c_Device*)设备句柄指针，指向要操作的设备
 * @param regAddr      (uint8)寄存器地址
 * @return (uint8)读取的字节
 */
static uint8_t tjrc_icm20602_readByte(uint8_t regAddr)
{
    uint32_t timeOut = 0;
    volatile uint8_t writeBuff = regAddr, readBuff = 0x00;
    while (timeOut++ < 500 && IfxI2c_I2c_write(&icm20602_Handler, &writeBuff, 1) == IfxI2c_I2c_Status_nak)
        ;
    timeOut = 0;
    while (timeOut++ < 500 && IfxI2c_I2c_read(&icm20602_Handler, &readBuff, 1) == IfxI2c_I2c_Status_nak)
        ;
    return readBuff;
}
