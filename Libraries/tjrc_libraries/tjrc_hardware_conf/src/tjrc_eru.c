/*
 * tjrc_eru.c
 *
 *  Created on: 2022年2月24日
 *      Author: 11657
 */


#include "tjrc_eru.h"



/**
 * @brief 初始化中断引脚
 * 
 * @param reqPin 
 */
void tjrc_initEru(IfxScu_Req_In *reqPin)
{
    IfxScuEru_initReqPin(reqPin, IfxPort_InputMode_pullDown);

    IfxScuEru_InputChannel inputChannel = (IfxScuEru_InputChannel)reqPin->channelId;
    IfxScuEru_InputNodePointer triggerSelect = (IfxScuEru_InputNodePointer)(reqPin->channelId);
    IfxScuEru_OutputChannel outputChannel = (IfxScuEru_OutputChannel)(reqPin->channelId);

    /* 配置为下降沿触发 */
    IfxScuEru_enableFallingEdgeDetection(inputChannel);
    IfxScuEru_disableRisingEdgeDetection(inputChannel);

    IfxScuEru_enableTriggerPulse(inputChannel);
    IfxScuEru_connectTrigger(inputChannel, triggerSelect);

    IfxScuEru_setFlagPatternDetection(outputChannel, inputChannel, TRUE);
    IfxScuEru_enablePatternDetectionTrigger(outputChannel);
    IfxScuEru_setInterruptGatingPattern(outputChannel, IfxScuEru_InterruptGatingPattern_alwaysActive);

    volatile Ifx_SRC_SRCR *src = &MODULE_SRC.SCU.SCU.ERU[(int)outputChannel % 4];
    IfxSrc_Tos eru_service;
    uint8_t eru_priority;

    if (reqPin->channelId == 2 || reqPin->channelId == 6)
        eru_service = IfxSrc_Tos_dma;
    if (reqPin->channelId == 3 || reqPin->channelId == 7)
        eru_service = IfxSrc_Tos_cpu0;
    if (reqPin->channelId == 0 || reqPin->channelId == 4)
        eru_service = IfxSrc_Tos_cpu0;
    if (reqPin->channelId == 1 || reqPin->channelId == 5)
        eru_service = IfxSrc_Tos_cpu0;
    eru_priority = ISR_PRIORITY_ERU((uint8_t)reqPin->channelId);

    IfxSrc_init(src, eru_service, eru_priority);
    IfxSrc_enable(src);
}

IFX_ALIGN(256)
DMA_LINK dma_link_list;

/**
 * @brief 配置中断触发的DMA传输操作
 * 
 * @param dma_ch 
 * @param source_addr 
 * @param destination_addr 
 * @param reqPin 
 * @param dma_count 
 * @return uint8_t 
 */
uint8_t tjrc_initEmuDma(IfxDma_ChannelId dma_ch, uint8_t *source_addr, uint8_t *destination_addr,
                   IfxScu_Req_In *reqPin, uint16_t dma_count)
{
    IfxDma_Dma_Channel dmaChn;
    //eru触发DMA通道号   在eru文件中设置eru的优先级，即为触发的通道
    tjrc_initEru(reqPin);

    /* 拉取默认配置并获得dma句柄 */
    IfxDma_Dma_Config dmaConfig;
    IfxDma_Dma_initModuleConfig(&dmaConfig, &MODULE_DMA);
    IfxDma_Dma dma;
    IfxDma_Dma_initModule(&dma, &dmaConfig);

    IfxDma_Dma_ChannelConfig cfg;
    IfxDma_Dma_initChannelConfig(&cfg, &dma);

    uint8  list_num, i;
    uint16 single_channel_dma_count;

    list_num = 1;
    single_channel_dma_count = dma_count / list_num;
    if(16384 < single_channel_dma_count)
    {
        while(TRUE)
        {
            single_channel_dma_count = dma_count / list_num;
            if((single_channel_dma_count <= 16384) && !(dma_count % list_num))
            {
                break;
            }
            list_num++;
        }
    }


    if(1 == list_num)
    {
        cfg.shadowControl               = IfxDma_ChannelShadow_none;
        cfg.operationMode               = IfxDma_ChannelOperationMode_single;
        cfg.shadowAddress               = 0;
    }
    else
    {
        cfg.shadowControl               = IfxDma_ChannelShadow_linkedList;
        cfg.operationMode               = IfxDma_ChannelOperationMode_continuous;
        cfg.shadowAddress               = IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), (unsigned)&dma_link_list.linked_list[1]);
    }

    cfg.requestMode                     = IfxDma_ChannelRequestMode_oneTransferPerRequest;
    cfg.moveSize                        = IfxDma_ChannelMoveSize_8bit;
    cfg.busPriority                     = IfxDma_ChannelBusPriority_high;

    cfg.sourceAddress                   = IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), source_addr);
    cfg.sourceAddressCircularRange      = IfxDma_ChannelIncrementCircular_none;
    cfg.sourceCircularBufferEnabled     = TRUE;

    cfg.destinationAddressIncrementStep = IfxDma_ChannelIncrementStep_1;

    cfg.channelId                       = (IfxDma_ChannelId)dma_ch;
    cfg.hardwareRequestEnabled          = FALSE;
    cfg.channelInterruptEnabled         = TRUE;
    cfg.channelInterruptPriority        = ISR_PRIORITY_ERU_DMA;
    cfg.channelInterruptTypeOfService   = IfxSrc_Tos_cpu0;

    cfg.destinationAddress              = IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), destination_addr);

    cfg.transferCount                   = single_channel_dma_count;

    IfxDma_Dma_initChannel(&dmaChn, &cfg);

    if(1 < list_num)
    {
        i = 0;
        while(i < list_num)
        {
            cfg.destinationAddress      = IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), destination_addr + single_channel_dma_count * i);
            if(i == (list_num - 1)) cfg.shadowAddress = IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), (unsigned)&dma_link_list.linked_list[0]);
            else                    cfg.shadowAddress = IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), (unsigned)&dma_link_list.linked_list[i+1]);
            cfg.transferCount           = single_channel_dma_count;

            IfxDma_Dma_initLinkedListEntry((void *)&dma_link_list.linked_list[i], &cfg);
            i++;
        }
    }


    IfxDma_Dma_getSrcPointer(&dma_link_list.channel)->B.CLRR = 1;

    return list_num;
}

