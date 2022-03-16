/**
 * @file tjrc_fileIO.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 文件读写相关处理算法
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_ALGORITHM_INC_TJRC_FILEIO_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_ALGORITHM_INC_TJRC_FILEIO_H_


#include "ff.h"
#include "string.h"

typedef struct
{
    int32_t version;
    int32_t boot_cnt;
}TJRC_CONFINO;

void tjrc_fileIo_initConfFile(FIL *fp);
void tjrc_fileIo_getConfFile(FIL *fp, TJRC_CONFINO *conf);
void tjrc_fileIo_updateConfFile(FIL *fp, TJRC_CONFINO *conf);

void tjrc_fileIo_camera2bmp(char* path, uint8_t *image);

#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_ALGORITHM_INC_TJRC_FILEIO_H_ */
