/**
 * @file tjrc_sdmmc.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_sdmmc.h"


static int32_t tjrc_sdmmcResponse(uint8_t res);
static uint8_t tjrc_sdmmcSendCmd(uint8_t cmd, uint32_t arg, uint8_t crc);
static uint8_t tjrc_sdmmcReceiveBlock(uint8_t* rxd,uint16_t len);
static uint8_t tjrc_sdmmcTransmitBlock(uint8_t* txd, uint8_t cmd);

/**
 * @brief SDMMC卡的类型（一般为SDHC（V2）卡）
 * 
 */
uint8_t sdmmc_type = SDMMC_TYPE_ERR;

/**
 * @brief 初始化SDMMC，判别SD卡的类型并配置为SPI模式
 * 
 * @return int32_t 
 */
int32_t tjrc_setSdmmc(void)
{
    uint8_t res = 0, sdmmc_buff[4];
    int32_t retry = 0;

    tjrc_setSpi();
    tjrc_setSpiChannel(400000);
    /* 唤醒sdmmc 至少74个时钟（for stm32 @72MHz，当前波特率近似） */
    for(int i = 0; i<100;i++)
    {
        tjrc_spiTransmitByte(0xff);
    }
    rt_kprintf("[sdmmc]send CMD0....");
    /* 复位SD卡，进入IDLE状态 */
    retry = 20;
    while(retry>0 && res!=0x01)
    {
        res = tjrc_sdmmcSendCmd(SDMMC_CMD0, 0, 0x95);
        retry--;
    }
    if(retry > 0)
    {
        rt_kprintf("detect card, try to configure\r\n");
    }
    else
    {
        rt_kprintf("[ERR]cannot find sd card\r\n");
        return -1;
    }

    if(res==0X01)
    {
        if(tjrc_sdmmcSendCmd(SDMMC_CMD8,0x1AA,0x87)==1)
        {
            for(int i = 0; i < 4; i++)
            {
                sdmmc_buff[i]=tjrc_spiTransmitByte(0XFF);
            }
            /* 查询卡是否支持V2类型 */
            if(sdmmc_buff[2]==0X01 && sdmmc_buff[3]==0XAA)
            {
                /* 等待退出IDLE模式 */
                retry=100000;
                while(res && retry>0)
                {
                    tjrc_sdmmcSendCmd(SDMMC_CMD55,0,0X01);
                    res = tjrc_sdmmcSendCmd(SDMMC_CMD41,0x40000000,0X01);
                    retry--;
                }
                if(retry&&tjrc_sdmmcSendCmd(SDMMC_CMD58,0,0X01)==0)
                {
                    /* 获取OCR值 */
                    for(int i = 0; i < 4; i++)
                    {
                        sdmmc_buff[i]=tjrc_spiTransmitByte(0XFF);
                    }
                    if(sdmmc_buff[0]&0x40)
                    {
                        sdmmc_type = SDMMC_TYPE_V2HC;
                    }
                    else
                    {
                        sdmmc_type = SDMMC_TYPE_V2;
                    }
                }
            }
        }
        /* 判断是否为SD-V1卡 */
        else
        {
            tjrc_sdmmcSendCmd(SDMMC_CMD55,0,0X01);       //发送CMD55
            res=tjrc_sdmmcSendCmd(SDMMC_CMD41,0,0X01);    //发送CMD41
            if(res<=1)
            {
                sdmmc_type=SDMMC_TYPE_V1;
                /* 等待退出IDLE模式 */
                retry=100000;
                while(res&&retry>0)
                {
                    tjrc_sdmmcSendCmd(SDMMC_CMD55,0,0X01);   //发送CMD55
                    res=tjrc_sdmmcSendCmd(SDMMC_CMD41,0,0X01);//发送CMD41
                    retry--;
                };
            }
            /* 判断是否为MMC卡 */
            else
            {
                sdmmc_type=SDMMC_TYPE_MMC;//MMC V3
                /* 等待退出IDLE模式 */
                retry=100000;
                while(res&&res>0)
                {
                    res=tjrc_sdmmcSendCmd(SDMMC_CMD1,0,0X01);//发送CMD1
                    retry--;
                }
            }
            if(retry==0||tjrc_sdmmcSendCmd(SDMMC_CMD16,512,0X01)!=0)
                sdmmc_type=SDMMC_TYPE_ERR;//错误的卡
        }
    }
    /* 提高SPI通信速率 */
    tjrc_setSpiChannel(32000000);
    if(sdmmc_type!=SDMMC_TYPE_ERR)
    {
        switch(sdmmc_type)
        {
        case SDMMC_TYPE_MMC:
            rt_kprintf("[sdmmc]find MMC card\r\n");
            break;
        case SDMMC_TYPE_V1:
            rt_kprintf("[sdmmc]find sd-card(V1)\r\n");
            break;
        case SDMMC_TYPE_V2:
            rt_kprintf("[sdmmc]find sd-card(V2)\r\n");
            break;
        case SDMMC_TYPE_V2HC:
            rt_kprintf("[sdmmc]find sd-card(V2HC)\r\n");
            break;
        }
        return 0;
    }
    else
    {
        if(res)
        {
            rt_kprintf("[sdmmc*]fail to configure the card, res=%d\r\n",res);
            return res;
        }
    }
    return -2;
}

/**
 * @brief 从SDMMC设备读取若干扇区的数据
 * 
 * @param rxd 数据起始地址
 * @param sector 扇区地址（512Byte=1扇区）
 * @param cnt 扇区数
 * @return uint8_t 读取状态，0=成功
 */
uint8_t tjrc_sdmmcReadDisk(uint8_t* rxd,uint32_t sector,uint32_t cnt)
{
    uint8_t res;
    if(sdmmc_type!=SDMMC_TYPE_V2HC)
    {
        /* 转换为字节地址 */
        sector <<= 9;
    }
    if(cnt==1)
    {
        /* 起始读命令 */
        res = tjrc_sdmmcSendCmd(SDMMC_CMD17,sector,0X01);//读命令
        if(res==0)
        {
            /* 接收512个字节 */
            res=tjrc_sdmmcReceiveBlock(rxd,512);
        }
    }else
    {
        /* 连续读命令 */
        res=tjrc_sdmmcSendCmd(SDMMC_CMD18,sector,0X01);
        do
        {
            /* 接收512个字节 */
            res=tjrc_sdmmcReceiveBlock(rxd,512);
            rxd+=512;
        }while(--cnt && res==0);
        /* 终止读命令 */
        tjrc_sdmmcSendCmd(SDMMC_CMD12,0,0X01);
    }
    return res;
}

/**
 * @brief 向SDMMC设备写入若干扇区的数据
 * 
 * @param txd 数据起始地址
 * @param sector 扇区地址（512Byte=1扇区）
 * @param cnt 扇区数
 * @return uint8_t 读取状态，0=成功
 */
uint8_t tjrc_sdmmcWriteDisk(uint8_t* txd, uint32_t sector,uint32_t cnt)
{
    uint8_t res;
    if(sdmmc_type!=SDMMC_TYPE_V2HC)
    {
        sector <<= 9;
    }
    if(cnt==1)
    {
        /* 单扇区写命令 */
        res = tjrc_sdmmcSendCmd(SDMMC_CMD24,sector,0X00);
        if(res == 0)
        {
            res = tjrc_sdmmcTransmitBlock(txd,0xFE);
        }
        else
        {
            return 0xFE;
        }
    }
    else
    {
        if(sdmmc_type != SDMMC_TYPE_MMC)
        {
            tjrc_sdmmcSendCmd(SDMMC_CMD55,0,0X01);
            tjrc_sdmmcSendCmd(SDMMC_CMD23,cnt,0X01);
        }
        /* 连续写命令 */
        res = tjrc_sdmmcSendCmd(SDMMC_CMD25,sector,0X01);
        if(res == 0)
        {
            do
            {
                res = tjrc_sdmmcTransmitBlock(txd,0xFC);
                txd+=512;
            }
            while(--cnt && res==0);
            res=tjrc_sdmmcTransmitBlock(0,0xFD);
        }
    }
    return res;
}

/**
 * @brief 读取sdmmc的CSD（卡特性寄存器）寄存器
 * 
 * @param csd_data 数据地址
 * @return uint8_t CSD数据
 */
uint8_t tjrc_sdmmcCsd(uint8_t *csd_data)
{
    uint8_t res;
    /* 发CMD9命令，读CSD */
    res=tjrc_sdmmcSendCmd(SDMMC_CMD9,0,0x01);
    if(res==0)
    {
        /* 接收16个字节的数据  */
        res=tjrc_sdmmcReceiveBlock(csd_data, 16);
    }
    if(res)
        return 1;
    else
        return 0;
}

/**
 * @brief 获取SDMMC的扇区数
 * 
 * @return uint32_t 
 */
uint32_t tjrc_sdmmcSectorCnt(void)
{
    uint8_t csd[16], n;
    uint32_t cap;
    uint16_t csize;
    if(tjrc_sdmmcCsd(csd)!=0)
    {
        rt_kprintf("[*sdmmc]fail to get CSD\r\n");
        return 0;
    }
    if((csd[0]&0xC0)==0x40)  //V2.00的卡
    {
        /* V2的卡 */
        csize = csd[9] + ((uint16_t)csd[8] << 8) + 1;
        cap = (uint32_t)csize << 10;//得到扇区数
    }else
    {
        /* V1的卡 */
        n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
        csize = (csd[8] >> 6) + ((uint16_t)csd[7] << 2) + ((uint16_t)(csd[6] & 3) << 10) + 1;
        cap= (uint32_t)csize << (n - 9);//得到扇区数
    }
    return cap;
}

/**
 * @brief 等待SDMMC进入空闲状态
 * 
 * @return int32_t 
 */
int32_t tjrc_sdmmcWaitReady(void)
{
    uint32_t retry=0;
    tjrc_spiTransmitByte(0xff);
    tjrc_spiTransmitByte(0xff);
    do
    {
        if(tjrc_spiTransmitByte(0XFF)==0XFF)
            return 0;//OK
        retry++;
    }while(retry<0XFFFFFF);//等待
    tjrc_spiTransmitByte(0xff);
    tjrc_spiTransmitByte(0xff);
    return -1;
}

/**
 * @brief 获取SDMMC的响应
 * 
 * @param res 期望响应值
 * @return int32_t 0=正常响应
 */
static int32_t tjrc_sdmmcResponse(uint8_t res)
{
    int32_t retry = 0xfffffff;
    while ((tjrc_spiTransmitByte(0XFF)!=res)&&retry>0)
    {
        retry--;
    }
    if (retry==0)
        return -1;
    else
        return 0;
}

/**
 * @brief 发送标准指令给SDMMC
 * 
 * @param cmd 指令号(1byte)
 * @param arg 指令参数(4byte)
 * @param crc crc校验码(1byte)
 * @return uint8_t 响应
 */
static uint8_t tjrc_sdmmcSendCmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t res;
    int16_t retry=0;
    /* 分别写入命令 */
    tjrc_spiTransmitByte(cmd | 0x40);
    tjrc_spiTransmitByte(arg >> 24);
    tjrc_spiTransmitByte(arg >> 16);
    tjrc_spiTransmitByte(arg >> 8);
    tjrc_spiTransmitByte((uint8_t)arg);
    tjrc_spiTransmitByte(crc);
    if(cmd == SDMMC_CMD12)
        tjrc_spiTransmitByte(0xff);
    /* 等待sdmmc响应 */
    retry = 0X1F;
    while((res & 0X80) && retry>0)
    {
        res = tjrc_spiTransmitByte(0xFF);
        retry--;
    }
    return res;
}

/**
 * @brief 接收任意长度的数据
 * 
 * @param rxd 接收数据缓存的起始地址
 * @param len 数据长度
 * @return uint8_t 响应
 */
static uint8_t tjrc_sdmmcReceiveBlock(uint8_t* rxd,uint16_t len)
{
    /* 等待SD卡发回数据起始令牌0xFE */
    if(tjrc_sdmmcResponse(0xFE))
    {
        return 1;
    }
    /* 开始接收数据 */
//    while(len--)
//    {
//        *rxd=tjrc_spiTransmitByte(0xFF);
//        rxd++;
//    }
    tjrc_spiTransmit(NULL_PTR,rxd,len);
    /* dummy CRC */
    tjrc_spiTransmitByte(0xFF);
    tjrc_spiTransmitByte(0xFF);
    return 0;
}

/**
 * @brief 发送1扇区数据
 * 
 * @param txd 发送数据的起始地址
 * @param cmd 发送指令
 * @return uint8_t 响应
 */
static uint8_t tjrc_sdmmcTransmitBlock(uint8_t* txd, uint8_t cmd)
{
    uint8_t res;
    uint32_t retry;
    /* 等待准备失效 */
    if(tjrc_sdmmcWaitReady())
        return 1;
    tjrc_spiTransmitByte(cmd);
    if(cmd!=0XFD)
    {
        tjrc_spiTransmit(txd,NULL_PTR,512);
        tjrc_spiTransmitByte(0xFF);
        tjrc_spiTransmitByte(0xFF);

        /* 接收响应 */
        res=tjrc_spiTransmitByte(0xFF);
        if((res&0x1F)!=0x05)
        {
            printf("[sd_txblock*]%x",cmd);
            return 2;
        }
        retry = 0;
        while(cmd==0XFE&&!tjrc_spiTransmitByte(0xff))
        {
            retry++;
            if(retry>0xfffffff)
            {
                return 1;
            }
        }
    }
    return 0;
}

