/*
 * tjrc_asclin.h
 *
 *  Created on: 2022年2月24日
 *      Author: 11657
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_HARDWARE_CONF_TJRC_ASCLIN_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_HARDWARE_CONF_TJRC_ASCLIN_H_

#include "rtthread.h"
#include "stdint.h"
#include "isr_config.h"
#include "IfxAsclin_Asc.h"



void tjrc_setAsclin0_uart(void);
void tjrc_setAsclin1_uart(void);
void tjrc_setAsclin3_uart(void);

void tjrc_asclin0_transmit(uint8_t *buff, uint32_t len);
void tjrc_asclin1_transmit(uint8_t *buff, uint32_t len);
void tjrc_asclin3_transmit(uint8_t *buff, uint32_t len);
void tjrc_asclin1_sendStr(uint8_t *buff);
uint8_t tjrc_asclin3_receive(uint8_t *resDat);

#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_HARDWARE_CONF_TJRC_ASCLIN_H_ */
