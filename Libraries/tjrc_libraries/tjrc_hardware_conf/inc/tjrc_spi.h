/**
 * @file tjrc_spi.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief QSPI硬件配置层头文件
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __TJRC_SPI_H__
#define __TJRC_SPI_H__

#include "rtthread.h"
#include "stdint.h"

#include "IFXQSPI_REGDEF.h"
#include "IfxQspi_SpiMaster.h"
#include "IfxQspi.h"

#include "isr_config.h"

int32_t tjrc_setSpi(void);
int32_t tjrc_setSpiChannel(float baudrate);
uint8_t tjrc_spiTransmitByte(uint8_t txd);
void tjrc_spiTransmit(uint8_t* txd, uint8_t* rxd, int32_t len);

#endif



