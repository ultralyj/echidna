/*
 * tjrc_softI2c.c
 *
 *  Created on: 2022年4月26日
 *      Author: 11657
 */
#include "tjrc_softI2c.h"

uint16 simiic_delay_time = 20;   //ICM等传感器应设置为20
#define ack 1      //主应答
#define no_ack 0   //从应答


static void simiic_start(void);
static void simiic_stop(void);
static void simiic_sendack(unsigned char ack_dat);
static int sccb_waitack(void);
static void send_ch(uint8 c);
static uint8 read_ch(uint8 ack_x);

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC延时 时间设置
//  @return     void
//  @since      v1.0
//  Sample usage:               如果IIC通讯失败可以尝试增加simiic_delay_time的值
//-------------------------------------------------------------------------------------------------------------------
void simiic_delay_set(uint16 time)
{
    simiic_delay_time = time;           //ICM等传感器应设置为100
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC延时
//  @return     void
//  @since      v1.0
//  Sample usage:               如果IIC通讯失败可以尝试增加simiic_delay_time的值
//-------------------------------------------------------------------------------------------------------------------
void simiic_delay(void)
{
    uint16 delay_time;
    delay_time = simiic_delay_time;
    while(delay_time--);
}


//内部使用，用户无需调用
static void simiic_start(void)
{
    IfxPort_setPinHigh(I2C_SDA_PIN);
    IfxPort_setPinHigh(I2C_SCL_PIN);
    simiic_delay();
    IfxPort_setPinLow(I2C_SDA_PIN);
    simiic_delay();
    IfxPort_setPinLow(I2C_SCL_PIN);
}

//内部使用，用户无需调用
static void simiic_stop(void)
{
    IfxPort_setPinLow(I2C_SDA_PIN);
    IfxPort_setPinLow(I2C_SCL_PIN);
    simiic_delay();
    IfxPort_setPinHigh(I2C_SCL_PIN);
    simiic_delay();
    IfxPort_setPinHigh(I2C_SDA_PIN);
    simiic_delay();
}

//主应答(包含ack:SDA=0和no_ack:SDA=0)
//内部使用，用户无需调用
static void simiic_sendack(unsigned char ack_dat)
{
    IfxPort_setPinLow(I2C_SCL_PIN);
    simiic_delay();
    if(ack_dat)
        IfxPort_setPinLow(I2C_SDA_PIN);
    else
        IfxPort_setPinHigh(I2C_SDA_PIN);

    IfxPort_setPinHigh(I2C_SCL_PIN);
    simiic_delay();
    IfxPort_setPinLow(I2C_SCL_PIN);
    simiic_delay();
}


static int sccb_waitack(void)
{
    IfxPort_setPinLow(I2C_SCL_PIN);
    IfxPort_setPinModeInput(I2C_SDA_PIN, IfxPort_InputMode_noPullDevice);
    simiic_delay();

    IfxPort_setPinHigh(I2C_SCL_PIN);
    simiic_delay();

    if(IfxPort_getPinState(I2C_SDA_PIN))           //应答为高电平，异常，通信失败
    {
        IfxPort_setPinModeOutput(I2C_SDA_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
        IfxPort_setPinLow(I2C_SCL_PIN);
        return 0;
    }
    IfxPort_setPinModeOutput(I2C_SDA_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinLow(I2C_SCL_PIN);
    simiic_delay();
    return 1;
}

//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
static void send_ch(uint8 c)
{
    uint8 i = 8;
    while(i--)
    {
        if(c & 0x80)    IfxPort_setPinHigh(I2C_SDA_PIN);//IfxPort_getPinState(I2C_SDA_PIN) 输出数据
        else            IfxPort_setPinLow(I2C_SDA_PIN);
        c <<= 1;
        simiic_delay();
        IfxPort_setPinHigh(I2C_SCL_PIN);                //SCL 拉高，采集信号
        simiic_delay();
        IfxPort_setPinLow(I2C_SCL_PIN);                //SCL 时钟线拉低
    }
    sccb_waitack();
}

//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|使用
//内部使用，用户无需调用
static uint8 read_ch(uint8 ack_x)
{
    uint8 i;
    uint8 c;
    c=0;
    IfxPort_setPinLow(I2C_SCL_PIN);
    simiic_delay();
    IfxPort_setPinHigh(I2C_SDA_PIN);
    IfxPort_setPinModeInput(I2C_SDA_PIN, IfxPort_InputMode_noPullDevice);           //置数据线为输入方式

    for(i=0;i<8;i++)
    {
        simiic_delay();
        IfxPort_setPinLow(I2C_SCL_PIN);         //置时钟线为低，准备接收数据位
        simiic_delay();
        IfxPort_setPinHigh(I2C_SCL_PIN);         //置时钟线为高，使数据线上数据有效
        simiic_delay();
        c<<=1;
        if(IfxPort_getPinState(I2C_SDA_PIN))
        {
            c+=1;   //读数据位，将接收的数据存c
        }
    }
    IfxPort_setPinModeOutput(I2C_SDA_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinLow(I2C_SCL_PIN);
    simiic_delay();
    simiic_sendack(ack_x);

    return c;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC写数据到设备寄存器函数
//  @param      dev_add         设备地址(低七位地址)
//  @param      reg             寄存器地址
//  @param      dat             写入的数据
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
    simiic_start();
    send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
    send_ch( reg );                  //发送从机寄存器地址
    send_ch( dat );                  //发送需要写入的数据
    simiic_stop();
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC从设备寄存器读取数据
//  @param      dev_add         设备地址(低七位地址)
//  @param      reg             寄存器地址
//  @param      type            选择通信方式是IIC  还是 SCCB
//  @return     uint8           返回寄存器的数据
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8 simiic_read_reg(uint8 dev_add, uint8 reg)
{
    uint8 dat;
    simiic_start();
    send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
    send_ch( reg );                 //发送从机寄存器地址

    simiic_start();
    send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
    dat = read_ch(no_ack);                  //读取数据
    simiic_stop();

    return dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC读取多字节数据
//  @param      dev_add         设备地址(低七位地址)
//  @param      reg             寄存器地址
//  @param      dat_add         数据保存的地址指针
//  @param      num             读取字节数量
//  @param      type            选择通信方式是IIC  还是 SCCB
//  @return     uint8           返回寄存器的数据
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint8 num)
{
    simiic_start();
    send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
    send_ch( reg );                 //发送从机寄存器地址

    simiic_start();
    send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
    while(--num)
    {
        *dat_add = read_ch(ack); //读取数据
        dat_add++;
    }
    *dat_add = read_ch(no_ack); //读取数据
    simiic_stop();
}


void simiic_init(void)
{
    IfxPort_setPinModeOutput(I2C_SCL_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(I2C_SDA_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinPadDriver(I2C_SCL_PIN,IfxPort_PadDriver_ttlSpeed1);
    IfxPort_setPinPadDriver(I2C_SDA_PIN,IfxPort_PadDriver_ttlSpeed1);
}





