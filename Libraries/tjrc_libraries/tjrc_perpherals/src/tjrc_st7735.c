/**
 * @file tjrc_st7735.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief st7735 TFT屏幕驱动
 * @version 0.1
 * @date 2022-03-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "tjrc_st7735.h"
#include "rtthread.h"

/* 屏幕控制参数 */
uint16_t backgroundColor = RGB565_BLACK;
/* 因为只有一个屏，屏幕控制句柄就不对外开放了 */
static uint8_t st7735_busyFlag = 0;
/* 基础：硬件层面的读写函数  */
static void st7735_WriteBit(uint8_t dat);
static void st7735_WR_DATA8(uint8_t dat);
static void st7735_WR_DATA(uint16_t dat);
static void st7735_WR_REG(uint8_t dat);
/* 初始化，包含众多默认配置数据 */
static void tjrc_st7735_init(void);
/* 中间层，可被外部调用  */
static void tjrc_st7735_setRegion(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
static void st7735_displaySingleChar6_12(uint16_t dx, uint16_t dy, uint8_t character, uint16_t color);

/**
 * @brief 配置屏幕的引脚，背景参数并清屏
 * @return NONE
 */
void tjrc_setSt7735(void)
{
    tjrc_st7735_init();
    tjrc_st7735_clean(RGB565_BLACK);
}

/**
 * @brief 向屏幕写入一串字符串
 * @param dx    (uint16_t)字符起始横坐标(L->R)
 * @param dy    (uint16_t)字符起始纵坐标(U->D)
 * @param str   (uint8_t*)字符串
 * @param color (uint16_t)字符串颜色
 * @return NONE
 */
void tjrc_st7735_dispStr612(uint16_t dx, uint16_t dy,
                            uint8_t *str, uint16_t color)
{
    if (tjrc_st7735_getBusy())
        return;
    tjrc_st7735_setBusy(1);
    while (*str)
    {
        if (*str >= ' ')
        {
            st7735_displaySingleChar6_12(dx, dy, *str++, color);
            dx += 6;
        }
        if (*str == '\n' || dx >= PIXEL_WIDTH) //判断换行符
        {
            dx = 0;
            dy += 12;
            str++;
        }
    }
    tjrc_st7735_setBusy(0);
}

/**
 * @brief 向屏幕画一个点
 * @param x    (uint16)点的横坐标
 * @param y    (uint16)点的纵坐标
 * @param color (uint16)颜色
 * @return NONE
 */
void tjrc_st7735_drawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    tjrc_st7735_setRegion(x, y, x, y);
    st7735_WR_DATA(color);
}

/**
 * @brief 向屏幕画一个矩形填充
 * @param x    (uint16)点的横坐标
 * @param y    (uint16)点的纵坐标
 * @param width (uint16)宽度
 * @param height(uint16)高度
 * @param color (uint16)颜色
 * @return NONE
 */
void tjrc_st7735_drawRectangle(uint16_t x, uint16_t y,uint16_t width, uint16_t height, uint16_t color)
{
    tjrc_st7735_setRegion(x, y, x+width-1, y+height-1);
    for(uint32_t i=0;i<width*height;i++)
        st7735_WR_DATA(color);
}

/**
 * @brief 向屏幕画一个矩形边框
 * @param x    (uint16)点的横坐标
 * @param y    (uint16)点的纵坐标
 * @param width (uint16)宽度
 * @param height(uint16)高度
 * @param color (uint16)颜色
 * @return NONE
 */
void tjrc_st7735_drawBox(uint16_t x, uint16_t y,uint16_t width, uint16_t height, uint16_t color)
{
    tjrc_st7735_drawRectangle(x,y,width,1,color);
    tjrc_st7735_drawRectangle(x,y,1,height,color);
    tjrc_st7735_drawRectangle(x+width,y,1,height,color);
    tjrc_st7735_drawRectangle(x,y+height,width,1,color);
}


/**
 * @brief 向屏幕写入一幅点阵位图
 * @param image     (uint8_t*)位图指针
 * @param width     (uint16_t)图像宽度（像素）
 * @param height    (uint16_t)图像高度（像素）
 * @param dx        (uint16_t)字符起始横坐标(L->R)
 * @param dy        (uint16_t)字符起始纵坐标(U->D)
 * @param color     (uint16_t)字符串颜色
 * @return NONE
 */
void tjrc_st7735_dispImage(uint8_t *image, uint16_t width, uint16_t height, uint16_t x, uint16_t y, uint16_t color)
{
    if(!tjrc_st7735_getBusy())
    {
        tjrc_st7735_setBusy(1);
        tjrc_st7735_setRegion(x, y, x + width - 1, y + height - 1);
        for (uint32_t i = 0; i < height; i++)
        {
            for (uint32_t j = 0; j < width / 8; j++)
            {
                uint8_t temp = image[i * width / 8 + j];
                for (uint32_t k = 0; k < 8; k++)
                {
                    if (temp & 0x80)
                        st7735_WR_DATA(color);
                    else
                        st7735_WR_DATA(backgroundColor);
                    temp <<= 1;
                }
            }
        }
        tjrc_st7735_setBusy(0);
    }
}

void tjrc_st7735_dispbin(uint8_t *image, uint16_t width, uint16_t height, uint16_t x, uint16_t y, uint16_t color,uint8_t th)
{
    if(!tjrc_st7735_getBusy())
    {
        tjrc_st7735_setBusy(1);
        tjrc_st7735_setRegion(x, y, x + width - 1, y + height - 1);
        for (uint32_t i = 0; i < height; i++)
        {
            for (uint32_t j = 0; j < width; j++)
            {
                if (image[i * 120 + j]>th?1:0)   //image[i * 120 + j]>th?1:0
                    st7735_WR_DATA(color);
                else
                    st7735_WR_DATA(backgroundColor);
            }
        }
        tjrc_st7735_setBusy(0);
    }
}

/**
 * @brief 向屏幕写入一幅灰度图
 * @param image     (uint8_t*)位图指针
 * @param width     (uint16_t)图像宽度（像素）
 * @param height    (uint16_t)图像高度（像素）
 * @param dx        (uint16_t)字符起始横坐标(L->R)
 * @param dy        (uint16_t)字符起始纵坐标(U->D)
 * @return NONE
 */
void tjrc_st7735_dispImage_gray(uint8_t *image, uint16_t width, uint16_t height, uint16_t x, uint16_t y)
{
    if(!tjrc_st7735_getBusy())
    {
        tjrc_st7735_setBusy(1);
        tjrc_st7735_setRegion(x, y, x + width - 1, y + height - 1);
        for (uint32_t i = 0; i < height; i++)
        {
            uint16_t color_565;
            for (uint32_t j = 0; j < width; j++)
            {
                color_565 = (uint16_t)(image[i * width + j]>>3)<<11;
                color_565 += (uint16_t)(image[i * width + j]>>2)<<5;
                color_565 += (uint16_t)(image[i * width + j]>>3);
                st7735_WR_DATA(color_565);
            }
        }
        tjrc_st7735_setBusy(0);
    }
}

/**
 * @brief 用一种颜色填满屏幕，并把这种颜色设置为背景色
 * @param color (uint16_t)清屏的颜色
 * @return NONE
 */
void tjrc_st7735_clean(uint16_t color)
{
    tjrc_st7735_setBusy(1);
    uint32_t index, totalpoint = PIXEL_WIDTH * PIXEL_HEIGHT; //得到总点数
    tjrc_st7735_setRegion(0, 0, PIXEL_WIDTH - 1, PIXEL_HEIGHT - 1);
    for (index = 0; index < totalpoint; index++)
    {
        st7735_WR_DATA(color);
    }
    backgroundColor = color;
    tjrc_st7735_setBusy(0);
}

/**
 * @brief 向屏幕写入一个字符
 * @param dx    (uint16_t)字符起始横坐标(L->R)
 * @param dy    (uint16_t)字符起始纵坐标(U->D)
 * @param str   (uint8_t)字符，从' '开始
 * @param color (uint16_t)字符串颜色
 * @return NONE
 */
static void st7735_displaySingleChar6_12(uint16_t dx, uint16_t dy,
                                         uint8_t character, uint16_t color)
{
    extern const uint8_t ASCII_6_12[96 * 12];
    character -= ' ';
    tjrc_st7735_setRegion(dx, dy, dx + 5, dy + 11);
    for (uint16_t i = 0; i < 12; i++)
    {
        uint8_t temp = ASCII_6_12[12 * character + i];
        for (uint16_t j = 0; j < 6; j++)
        {
            if (temp & 0x80)
                st7735_WR_DATA(color);
            else
                st7735_WR_DATA(backgroundColor);
            temp <<= 1;
        }
    }
}

/**
 * @brief 设置当前光标位置，需给出两个坐标，第一个是光标起始点(画布左上角)，第二个是要更改部分的终点(画布右下角)
 * @param x1    (uint16_t)起始横坐标
 * @param y1    (uint16_t)起始纵坐标
 * @param x2    (uint16_t)终点横坐标
 * @param y2    (uint16_t)终点纵坐标
 * @return NONE
 */
static void tjrc_st7735_setRegion(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    st7735_WR_REG(0x2a);
    st7735_WR_DATA(x1 + 2);
    st7735_WR_DATA(x2 + 2);
    st7735_WR_REG(0x2b);
    st7735_WR_DATA(y1 + 1);
    st7735_WR_DATA(y2 + 1);
    st7735_WR_REG(0x2c);
}

/**
 * @brief 初始化屏幕，并注入一系列数据
 * @return NONE
 */
static void tjrc_st7735_init(void)
{
    /* 初始化引脚 */
    IfxPort_setPinModeOutput(ST7735_CS, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(ST7735_SDA, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(ST7735_SCL, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(ST7735_RES, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(ST7735_DC, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(ST7735_BL, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    /* 设置IO驱动速度 */
    IfxPort_setPinPadDriver(ST7735_SCL, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxPort_setPinPadDriver(ST7735_SDA, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxPort_setPinPadDriver(ST7735_CS, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxPort_setPinPadDriver(ST7735_RES, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxPort_setPinPadDriver(ST7735_DC, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxPort_setPinPadDriver(ST7735_BL, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    /* 屏幕背光常亮 */
    IfxPort_setPinHigh(ST7735_BL);
    /* 硬件复位信号（可选） */
     IfxPort_setPinLow(ST7735_RES);
     rt_thread_mdelay(100);
     IfxPort_setPinHigh(ST7735_RES);
     rt_thread_mdelay(100);
    /* 配置ST7735的各项寄存器 */
    st7735_WR_REG(0x11); // Sleep out
    /* 配置显示驱动芯片，ST7735S Frame rate */
    st7735_WR_REG(0xB1); // Frame rate 80Hz
    st7735_WR_DATA8(0x02);
    st7735_WR_DATA8(0x35);
    st7735_WR_DATA8(0x36);
    st7735_WR_REG(0xB2); // Frame rate 80Hz
    st7735_WR_DATA8(0x02);
    st7735_WR_DATA8(0x35);
    st7735_WR_DATA8(0x36);
    st7735_WR_REG(0xB3); // Frame rate 80Hz
    st7735_WR_DATA8(0x02);
    st7735_WR_DATA8(0x35);
    st7735_WR_DATA8(0x36);
    st7735_WR_DATA8(0x02);
    st7735_WR_DATA8(0x35);
    st7735_WR_DATA8(0x36);

    st7735_WR_REG(0xB4); // Dot inversion
    st7735_WR_DATA8(0x03);

    /* ST7735S Power Sequence */
    st7735_WR_REG(0xC0);
    st7735_WR_DATA8(0xA2);
    st7735_WR_DATA8(0x02);
    st7735_WR_DATA8(0x84);
    st7735_WR_REG(0xC1);
    st7735_WR_DATA8(0xC5);
    st7735_WR_REG(0xC2);
    st7735_WR_DATA8(0x0D);
    st7735_WR_DATA8(0x00);
    st7735_WR_REG(0xC3);
    st7735_WR_DATA8(0x8D);
    st7735_WR_DATA8(0x2A);
    st7735_WR_REG(0xC4);
    st7735_WR_DATA8(0x8D);
    st7735_WR_DATA8(0xEE);

    st7735_WR_REG(0xC5); // VCOM
    st7735_WR_DATA8(0x0a);

    /* 重要寄存器（36h）：决定转向和色彩模式 */
    st7735_WR_REG(0x36);
    st7735_WR_DATA8(0xC0);

    /* ST7735S Gamma Sequence */
    st7735_WR_REG(0XE0);
    st7735_WR_DATA8(0x12);
    st7735_WR_DATA8(0x1C);
    st7735_WR_DATA8(0x10);
    st7735_WR_DATA8(0x18);
    st7735_WR_DATA8(0x33);
    st7735_WR_DATA8(0x2C);
    st7735_WR_DATA8(0x25);
    st7735_WR_DATA8(0x28);
    st7735_WR_DATA8(0x28);
    st7735_WR_DATA8(0x27);
    st7735_WR_DATA8(0x2F);
    st7735_WR_DATA8(0x3C);
    st7735_WR_DATA8(0x00);
    st7735_WR_DATA8(0x03);
    st7735_WR_DATA8(0x03);
    st7735_WR_DATA8(0x10);
    st7735_WR_REG(0XE1);
    st7735_WR_DATA8(0x12);
    st7735_WR_DATA8(0x1C);
    st7735_WR_DATA8(0x10);
    st7735_WR_DATA8(0x18);
    st7735_WR_DATA8(0x2D);
    st7735_WR_DATA8(0x28);
    st7735_WR_DATA8(0x23);
    st7735_WR_DATA8(0x28);
    st7735_WR_DATA8(0x28);
    st7735_WR_DATA8(0x26);
    st7735_WR_DATA8(0x2F);
    st7735_WR_DATA8(0x3B);
    st7735_WR_DATA8(0x00);
    st7735_WR_DATA8(0x03);
    st7735_WR_DATA8(0x03);
    st7735_WR_DATA8(0x10);

    st7735_WR_REG(0x3A); // 65k mode
    st7735_WR_DATA8(0x05);
    st7735_WR_REG(0x29); // Display on
}

/**
 * @brief 写入1字节数据到屏幕
 * @param dat   (uint8_t)数据
 * @return NONE
 */
static void st7735_WR_DATA8(uint8_t dat)
{
    st7735_WriteBit(dat);
}

/**
 * @brief 写入2字节数据到屏幕（通常配置好后直接在光标点处显示对应的565色彩）
 * @param dat   (uint16_t)数据
 * @return NONE
 */
static void st7735_WR_DATA(uint16_t dat)
{
    st7735_WriteBit(dat >> 8);
    st7735_WriteBit((uint8_t)dat);
}

/**
 * @brief 写入1字节命令到屏幕
 * @param dat   (uint8_t)命令
 * @return NONE
 */
static void st7735_WR_REG(uint8_t dat)
{
    IfxPort_setPinLow(ST7735_DC);
    st7735_WriteBit(dat);
    IfxPort_setPinHigh(ST7735_DC);
}
void delay_us(uint32_t us)
{
    for (uint32_t i = 0; i < us; i++)
    {
        for (uint32_t j = 0; j < 1000; j++)
            ;
    }
}
/**
 * @brief 模拟SPI时序，写入1字节数据到屏幕
 * @param dat   (uint8_t)写入的数据
 * @return NONE
 */
static void st7735_WriteBit(uint8_t dat)
{
    IfxPort_setPinLow(ST7735_CS);
    for (uint8_t i = 0; i < 8; i++)
    {
        IfxPort_setPinLow(ST7735_SCL);
        if (dat & 0x80)
            IfxPort_setPinHigh(ST7735_SDA);
        else
            IfxPort_setPinLow(ST7735_SDA);
        IfxPort_setPinHigh(ST7735_SCL);
        dat <<= 1;
    }
    IfxPort_setPinHigh(ST7735_CS);
}

/**
 * @brief 设置当前屏幕是否忙
 * 
 * @param state 
 */
void tjrc_st7735_setBusy(uint8_t state)
{
    st7735_busyFlag = state;
}

/**
 * @brief 判断当前屏幕是否忙
 * 
 * @return uint8_t 
 */
uint8_t tjrc_st7735_getBusy(void)
{
    return st7735_busyFlag;
}
