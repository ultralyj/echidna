/**
 * @file tjrc_eru.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief ERU硬件配置层头文件，配置外部中断和DMA相关参数
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
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
