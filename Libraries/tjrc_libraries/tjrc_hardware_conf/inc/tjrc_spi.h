/*
 * tjrc_spi.h
 *
 *  Created on: 2022年2月7日
 *      Author: 11657
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
int32_t tjrc_spiTransmit(uint8_t* txd, uint8_t* rxd, int32_t len);

#endif



