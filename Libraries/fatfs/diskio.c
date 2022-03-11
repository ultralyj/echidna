/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include <tjrc_peripherals.h>
#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */

/* Definitions of physical drive number for each drive */
#define DEV_MMC		0	/* Example: Map MMC/SD card to physical drive 0 */



/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	switch (pdrv)
	{
	case DEV_MMC :
		return RES_OK;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	int32_t res;
	rt_enter_critical();
	switch(pdrv)
	{
	    /* SD卡挂载在0号位上 */
	    case DEV_MMC:
	        res = tjrc_setSdmmc();
            if(!res)
            {
                rt_exit_critical();
                return RES_OK;
            }
            else
            {
                rt_exit_critical();
                return RES_ERROR;
            }
            break;
	}
	rt_exit_critical();
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	uint8_t res;
	rt_enter_critical();
    switch(pdrv)
    {
        /* SD卡挂载在0号位上 */
        case DEV_MMC:
            res = tjrc_sdmmcReadDisk((uint8_t*)buff,sector,count);
            if(!res)
            {
                rt_exit_critical();
                return RES_OK;
            }
            else
            {
                rt_exit_critical();
                return RES_ERROR;
            }
            break;
    }
    rt_exit_critical();
	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
    uint8_t res;

    rt_enter_critical();
    switch(pdrv)
    {
        /* SD卡挂载在0号位上 */
        case DEV_MMC:
            res = tjrc_sdmmcWriteDisk((uint8_t*)buff,sector,count);
            if(!res)
            {
                rt_exit_critical();
                return RES_OK;
            }
            else
            {
                rt_exit_critical();
                rt_kprintf("[ff_diskio]write error:%d\r\n",res);
                return RES_ERROR;
            }
            break;
    }
    rt_exit_critical();
    return RES_PARERR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
    DRESULT res;
    if(pdrv==DEV_MMC)//SD卡
    {
        switch(cmd)
        {
            case CTRL_SYNC:
                if(tjrc_sdmmcWaitReady()==0)
                    res = RES_OK;
                else
                {
                    res = RES_ERROR;
                }
                break;
            case GET_SECTOR_SIZE:
                *(WORD*)buff = 512;
                res = RES_OK;
                break;
            case GET_BLOCK_SIZE:
                *(WORD*)buff = 8;
                res = RES_OK;
                break;
            case GET_SECTOR_COUNT:
                *(DWORD*)buff = tjrc_sdmmcSectorCnt();
                res = RES_OK;
                break;
            default:
                res = RES_PARERR;
                break;
        }
    }
    else
    {
        return RES_ERROR;
    }
    return res;
}

DWORD get_fattime()
{
    return 0;
}
