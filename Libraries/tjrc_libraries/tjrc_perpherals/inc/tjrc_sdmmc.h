#ifndef __TJRC_SDMMC_H__
#define __TJRC_SDMMC_H__

#include "tjrc_peripherals.h"

#define SDMMC_CMD0    0       //卡复位
#define SDMMC_CMD1    1
#define SDMMC_CMD8    8       //命令8 ，SEND_IF_COND
#define SDMMC_CMD9    9       //命令9 ，读CSD数据
#define SDMMC_CMD10   10      //命令10，读CID数据
#define SDMMC_CMD12   12      //命令12，停止数据传输
#define SDMMC_CMD16   16      //命令16，设置SectorSize 应返回0x00
#define SDMMC_CMD17   17      //命令17，读sector
#define SDMMC_CMD18   18      //命令18，读Multi sector
#define SDMMC_CMD23   23      //命令23，设置多sector写入前预先擦除N个block
#define SDMMC_CMD24   24      //命令24，写sector
#define SDMMC_CMD25   25      //命令25，写Multi sector
#define SDMMC_CMD41   41      //命令41，应返回0x00
#define SDMMC_CMD55   55      //命令55，应返回0x01
#define SDMMC_CMD58   58      //命令58，读OCR信息
#define SDMMC_CMD59   59      //命令59，使能/禁止CRC，应返回0x00

#define SDMMC_TYPE_ERR     0X00
#define SDMMC_TYPE_MMC     0X01
#define SDMMC_TYPE_V1      0X02
#define SDMMC_TYPE_V2      0X04
#define SDMMC_TYPE_V2HC    0X06

int32_t tjrc_sdmmcWaitReady(void);
uint8_t tjrc_sdmmcCsd(uint8_t *csd_data);
uint32_t tjrc_sdmmcSectorCnt(void);

int32_t tjrc_setSdmmc(void);
uint8_t tjrc_sdmmcReadDisk(uint8_t* rxd,uint32_t sector,uint32_t cnt);
uint8_t tjrc_sdmmcWriteDisk(uint8_t* txd, uint32_t sector,uint32_t cnt);

#endif
