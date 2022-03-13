/**
 * @file tjrc_spi.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief SPI驱动代码，当前版本改为DMA驱动
 * @version 0.2
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_spi.h"

IfxQspi_SpiMaster qspi0_master_handler;
IfxQspi_SpiMaster_Channel qspi0_masterChannel_handler;
IfxQspi_SpiMaster_Pins qspi0_set_pins;
IfxQspi_SpiMaster_Output qspi0_cs_pin;

/**
 * @brief 初始化QSPI0接口
 * 
 * @return int32_t 初始化状态值
 */
int32_t tjrc_setSpi(void)
{
    /* SPI总线最大速率 */
    const int32_t SPI0_BUAD = 50000000;
    int32_t res = 0;

    rt_kprintf("[spi]s1. set pin\r\n");
    // 设置SPI相关引脚的模式，速度
    qspi0_set_pins.mrstMode = IfxPort_InputMode_pullDown;
    qspi0_set_pins.mtsrMode = IfxPort_OutputMode_pushPull;
    qspi0_set_pins.sclkMode = IfxPort_OutputMode_pushPull;
    qspi0_set_pins.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;

    // 设置SPI引脚的复用Port位置
    qspi0_set_pins.sclk = &IfxQspi0_SCLK_P20_11_OUT;
    qspi0_set_pins.mtsr = &IfxQspi0_MTSR_P20_14_OUT;
    qspi0_set_pins.mrst = &IfxQspi0_MRSTA_P20_12_IN;


    /* 重置SPI主机结构体  */
    rt_kprintf("[spi]s2. init master handle, baudrate = %d\r\n", SPI0_BUAD);
    IfxQspi_SpiMaster_Config MasterConfig;
    IfxQspi_SpiMaster_initModuleConfig(&MasterConfig, &MODULE_QSPI0);
    /* 配置SPI为主机；传输速率和CPU核心 */
    MasterConfig.base.mode = SpiIf_Mode_master;
    MasterConfig.base.maximumBaudrate = (float)SPI0_BUAD;

    MasterConfig.base.rxPriority = ISR_PRIORITY_DMA_CH1;
    MasterConfig.base.txPriority = ISR_PRIORITY_DMA_CH2;
    MasterConfig.base.erPriority = ISR_PRIORITY_QSPI0_ER;
    MasterConfig.base.isrProvider = IfxSrc_Tos_cpu0;
    MasterConfig.pins = &qspi0_set_pins;

    /* 配置使用DMA */
    MasterConfig.dma.useDma = TRUE;
    MasterConfig.dma.txDmaChannelId = IfxDma_ChannelId_1;
    MasterConfig.dma.rxDmaChannelId = IfxDma_ChannelId_2;
    /* 执行初始化 */
    IfxQspi_SpiMaster_initModule(&qspi0_master_handler, &MasterConfig);
    /* 初始化片选通道 */
    rt_kprintf("[spi]s3. init channel for QSPI0, cs: P20.13...");
    res = tjrc_setSpiChannel((float)1000000);
    rt_kprintf("%d\r\n",res);
    return res;
}

/**
 * @brief 初始化QSPI0的通道（CS） 
 * 
 * @param baudrate 通道波特率
 * @return int32_t 
 */
int32_t tjrc_setSpiChannel(float baudrate)
{
    int32_t res = 0;

    qspi0_cs_pin.pin = &IfxQspi0_SLSO2_P20_13_OUT;
    qspi0_cs_pin.driver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    qspi0_cs_pin.mode = IfxPort_OutputMode_pushPull;

    IfxQspi_SpiMaster_ChannelConfig MasterChConfig;
    IfxQspi_SpiMaster_initChannelConfig(&MasterChConfig, &qspi0_master_handler);
    MasterChConfig.base.baudrate = baudrate;

    /* 设置默认低电平与上升沿跳变 */
    MasterChConfig.base.mode.clockPolarity = SpiIf_ClockPolarity_idleLow;//CPOL
    MasterChConfig.base.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge;//CPHA
    /* 设置MSB和字长 */
    MasterChConfig.base.mode.dataHeading = SpiIf_DataHeading_msbFirst;
    MasterChConfig.base.mode.dataWidth = 8;
    /* 配置片选引脚 */
    MasterChConfig.base.mode.csActiveLevel = Ifx_ActiveState_low;
    MasterChConfig.sls.output = qspi0_cs_pin;
    MasterChConfig.base.mode.csInactiveDelay = 1;
    MasterChConfig.base.mode.csLeadDelay     = 10;
    MasterChConfig.base.mode.csTrailDelay    = 1;
    /* 执行通道初始化 */
    res = IfxQspi_SpiMaster_initChannel(&qspi0_masterChannel_handler, &MasterChConfig);
    return res;
}

/**
 * @brief QSPI0传输1Byte数据
 * 
 * @param txd 传输数据
 * @return uint8_t 接收数据 
 */
uint8_t tjrc_spiTransmitByte(uint8_t txd)
{
    uint8_t rxd = 0xff;
    IfxQspi_SpiMaster_exchange(&qspi0_masterChannel_handler, &txd, &rxd, 1);
    /* 等待传输结束  */
    while (IfxQspi_SpiMaster_getStatus(&qspi0_masterChannel_handler) == SpiIf_Status_busy);
    return rxd;
}

/**
 * @brief QSPI0交换定长数据
 * 
 * @param txd 发送数据起始地址
 * @param rxd 接收数据起始地址
 * @param len 数据长度
 * @return int32_t 
 */
void tjrc_spiTransmit(uint8_t* txd, uint8_t* rxd, int32_t len)
{
    IfxQspi_SpiMaster_exchange(&qspi0_masterChannel_handler, txd, rxd, (Ifx_SizeT)len);
    /* 等待传输结束  */
    while (IfxQspi_SpiMaster_getStatus(&qspi0_masterChannel_handler) == SpiIf_Status_busy);
}

