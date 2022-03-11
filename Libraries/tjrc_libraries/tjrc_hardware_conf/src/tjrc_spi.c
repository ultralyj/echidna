/*
 * thrc_spi.c
 *
 *  Created on: 2022年2月7日
 *      Author: 11657
 */

#include "tjrc_spi.h"

IfxQspi_SpiMaster MasterHandle;
IfxQspi_SpiMaster_Channel MasterChHandle;
IfxQspi_SpiMaster_Pins set_pin;
IfxQspi_SpiMaster_Output set_cs;

/**
 * @brief 初始化QSPI0接口
 * 
 * @return int32_t 初始化状态值
 */
int32_t tjrc_setSpi(void)
{
    const int32_t SPI0_BUAD = 50000000;
    int32_t res = 0;

    rt_kprintf("[spi]s1. set pin\r\n");
    // 设置SPI相关引脚的模式，速度
    set_pin.mrstMode = IfxPort_InputMode_pullDown;
    set_pin.mtsrMode = IfxPort_OutputMode_pushPull;
    set_pin.sclkMode = IfxPort_OutputMode_pushPull;
    set_pin.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    set_cs.driver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    set_cs.mode = IfxPort_OutputMode_pushPull;
    // 设置SPI引脚的复用Port位置
    set_pin.sclk = &IfxQspi0_SCLK_P20_11_OUT;
    set_pin.mtsr = &IfxQspi0_MTSR_P20_14_OUT;
    set_pin.mrst = &IfxQspi0_MRSTA_P20_12_IN;
    set_cs.pin = &IfxQspi0_SLSO2_P20_13_OUT;

    /* 重置SPI主机结构体  */
    rt_kprintf("[spi]s2. init master handle, baudrate = %d\r\n", SPI0_BUAD);
    IfxQspi_SpiMaster_Config MasterConfig;
    IfxQspi_SpiMaster_initModuleConfig(&MasterConfig, &MODULE_QSPI0);
    /* 配置SPI为主机；传输速率和CPU核心 */
    MasterConfig.base.mode = SpiIf_Mode_master;
    MasterConfig.base.maximumBaudrate = (float)SPI0_BUAD;

    MasterConfig.base.rxPriority = 92;
    MasterConfig.base.txPriority = 91;
    MasterConfig.base.erPriority = 90;
    MasterConfig.base.isrProvider = IfxSrc_Tos_cpu0;
    MasterConfig.pins = &set_pin;
    /* 执行初始化 */
    IfxQspi_SpiMaster_initModule(&MasterHandle, &MasterConfig);
    /* 初始化片选通道 */
    rt_kprintf("[spi]s3. init channel for QSPI0, cs: P20.13...");
    res = tjrc_setSpiChannel((float)1000000);
    rt_kprintf("%d\r\n",res);
    return res;
}

int32_t tjrc_setSpiChannel(float baudrate)
{
    int32_t res = 0;
    IfxQspi_SpiMaster_ChannelConfig MasterChConfig;
    IfxQspi_SpiMaster_initChannelConfig(&MasterChConfig, &MasterHandle);
    MasterChConfig.base.baudrate = baudrate;

    /* 设置默认低电平与上升沿跳变 */
    MasterChConfig.base.mode.clockPolarity = SpiIf_ClockPolarity_idleLow;//CPOL
    MasterChConfig.base.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge;//CPHA
    /* 设置MSB和字长 */
    MasterChConfig.base.mode.dataHeading = SpiIf_DataHeading_msbFirst;
    MasterChConfig.base.mode.dataWidth = 8;
    /* 配置片选引脚 */
    MasterChConfig.base.mode.csActiveLevel = Ifx_ActiveState_low;
    MasterChConfig.sls.output = set_cs;
    MasterChConfig.base.mode.csInactiveDelay = 1;
    MasterChConfig.base.mode.csLeadDelay     = 10;
    MasterChConfig.base.mode.csTrailDelay    = 1;
    /* 执行通道初始化 */
    res = IfxQspi_SpiMaster_initChannel(&MasterChHandle, &MasterChConfig);
    return res;
}

uint8_t tjrc_spiTransmitByte(uint8_t txd)
{
    uint8_t rxd = 0xff;
    IfxQspi_SpiMaster_exchange(&MasterChHandle, &txd, &rxd, 1);
    /* 等待传输结束  */
    while (IfxQspi_SpiMaster_getStatus(&MasterChHandle) == SpiIf_Status_busy);
    return rxd;
}

int32_t tjrc_spiTransmit(uint8_t* txd, uint8_t* rxd, int32_t len)
{
    int32_t timeout = 0;
    IfxQspi_SpiMaster_exchange(&MasterChHandle, txd, rxd, (Ifx_SizeT)len);
    /* 等待传输结束  */
    while (IfxQspi_SpiMaster_getStatus(&MasterChHandle) == SpiIf_Status_busy && timeout < 100)
    {
        rt_thread_mdelay(1);
        timeout++;
    }
    if(timeout == 100)
        return -1;
    else
        return 0;
}

