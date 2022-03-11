/*
 * tjrc_eru.h
 *
 *  Created on: 2022年2月24日
 *      Author: 11657
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_ERU_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_ERU_H_

#include "rtthread.h"
#include "stdint.h"

#include "IfxDma_Dma.h"
#include "IfxScuEru.h"
#include "isr_config.h"



typedef struct
{
    Ifx_DMA_CH linked_list[8];  //DMA链表
    IfxDma_Dma_Channel channel; //DMA通道句柄
} DMA_LINK;

void tjrc_initEru(IfxScu_Req_In *reqPin);
uint8_t tjrc_initEmuDma(IfxDma_ChannelId dma_ch, uint8_t *source_addr, uint8_t *destination_addr,
                   IfxScu_Req_In *reqPin, uint16_t dma_count);
#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_ERU_H_ */
