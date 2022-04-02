/**
 * @file tjrc_st7735.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief st7735 TFT屏幕驱动程序
 * @version 0.1
 * @date 2022-03-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_ST7735_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_ST7735_H_

#include "stdio.h"
#include "stdint.h"
#include "tjrc_peripherals.h"

/* 引脚定义 */
/* V2主板的引脚定义 */
#define ST7735_SCL &MODULE_P33, 4
#define ST7735_SDA &MODULE_P33, 5
#define ST7735_RES &MODULE_P33, 8
#define ST7735_DC &MODULE_P33, 9
#define ST7735_CS &MODULE_P33, 10
#define ST7735_BL &MODULE_P33, 11
/* 下面是V1主板的引脚定义 */
//#define ST7735_SCL &MODULE_P33, 5
//#define ST7735_SDA &MODULE_P33, 10
//#define ST7735_RES &MODULE_P33, 11
//#define ST7735_DC &MODULE_P33, 4
//#define ST7735_CS &MODULE_P33, 8
//#define ST7735_BL &MODULE_P33, 9

/* 定义屏幕的长宽 */
#define PIXEL_WIDTH 128
#define PIXEL_HEIGHT 160

/* 565颜色简表 */
/*   |-----|------|-----|
       r(5)  g(6)   b(5)      */
#define RGB565_WHITE 0xFFFF
#define RGB565_BLACK 0x0000
#define RGB565_BLUE 0x001F
#define RGB565_BRED 0XF81F
#define RGB565_GRED 0XFFE0
#define RGB565_GBLUE 0X07FF
#define RGB565_RED 0xF800
#define RGB565_MAGENTA 0xF81F
#define RGB565_GREEN 0x07E0
#define RGB565_CYAN 0x7FFF
#define RGB565_YELLOW 0xFFE0
#define RGB565_BROWN 0XBC40 //棕色
#define RGB565_BRRED 0XFC07 //棕红色
#define RGB565_GRAY 0X8430  //灰色
//GUI颜色
#define RGB565_DARKBLUE 0X01CF  //深蓝色
#define RGB565_LIGHTBLUE 0X7D7C //浅蓝色
#define RGB565_GRAYBLUE 0X5458  //灰蓝色

/* 配置屏幕 */
void tjrc_setSt7735(void);
/* 清空屏幕，并设置背景色 */
void tjrc_st7735_clean(uint16_t color);
/* 显示字符串 */
void tjrc_st7735_dispStr612(uint16_t dx, uint16_t dy, uint8_t *str, uint16_t color);
/* 绘图函数 */
void tjrc_st7735_drawPoint(uint16_t x, uint16_t y, uint16_t color);
void tjrc_st7735_drawRectangle(uint16_t x, uint16_t y,uint16_t width, uint16_t height, uint16_t color);
void tjrc_st7735_drawBox(uint16_t x, uint16_t y,uint16_t width, uint16_t height, uint16_t color);
/* 显示点图像 */
void tjrc_st7735_dispImage(uint8_t *image, uint16_t width, uint16_t height, uint16_t x, uint16_t y, uint16_t color);
void tjrc_st7735_dispbin(uint8_t *image, uint16_t width, uint16_t height, uint16_t x, uint16_t y, uint16_t color,uint8_t th);
void tjrc_st7735_dispImage_gray(uint8_t *image, uint16_t width, uint16_t height, uint16_t x, uint16_t y);
/* 获取屏幕当前状态 */
uint8_t tjrc_st7735_getBusy(void);
void tjrc_st7735_setBusy(uint8_t state);


#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_ST7735_H_ */
