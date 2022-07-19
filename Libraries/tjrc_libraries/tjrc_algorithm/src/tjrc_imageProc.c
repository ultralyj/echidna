/**
 * @file tjrc_imageProc.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 包含图像处理主要功能函数
 * @version 0.1
 * @date 2022-04-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "tjrc_imageProc.h"

IFX_ALIGN(4) uint8_t image_bin[IMAGE_HEIGHT][IMAGE_WIDTH];
line_info edge_line;
inflection_info inflections;

IFX_ALIGN(4) uint8_t midline[IMAGE_HEIGHT];
float camera_slope = 0, camera_pixelError = 0;

static void patchLine(uint8_t y0, uint8_t y1, uint8_t* line_x_out);
static float fit_slope(const uint8_t* line,line_info* line_info_in);
static float weighted_line(const uint8_t* line,line_info* line_info_in);

uint8_t* tjrc_imageProc(const uint8_t* image_p)
{
    uint8_t* image_bin_p = image_bin[0];
    /* 第一步：二值化 */
    uint8_t threshold = tjrc_binarization_otsu(image_p, IMAGE_WIDTH, IMAGE_HEIGHT);
    uint8_t threshold_bias = 20;
    threshold += threshold_bias;
    //printf("[imp]threshold=%d(OTSU)\r\n", threshold);
    tjrc_binarization_getBinImage(threshold, image_p, image_bin_p, IMAGE_WIDTH, IMAGE_HEIGHT);

    /* 第二步：搜线，求拐点，补线 */
    tjrc_imageProc_searchEdge_x(image_bin_p,&edge_line);
    tjrc_imageProc_fineInflection(&edge_line,&inflections);
    tjrc_imageProc_patchLine(&edge_line, &inflections);

    tjrc_imageProc_updateImage(image_bin_p,&edge_line);

    camera_pixelError = weighted_line(midline,&edge_line);
    camera_slope = fit_slope(midline,&edge_line);
    return image_bin_p;
}
/**
 * @brief 从中间向两边扩展的朴素搜线方法
 *
 * @param image [in](uint8*)图像指针(IMAGE_HEIGHT*IMAGE_WIDTH)
 * @param line_info_out [out]边线信息结构体，存储边线横坐标，是否丢线等信息
 */
void tjrc_imageProc_searchEdge_x(const uint8_t* image, line_info* line_info_out)
{
    /* 二维数组重构，方便调用 */
    uint8_t* image_p[IMAGE_HEIGHT];
    for (uint8_t i = 0; i < IMAGE_HEIGHT; i++)
    {
        image_p[i] = (uint8_t*) & image[i * IMAGE_WIDTH];
    }
    line_info_out->x_left_edgeCnt = 0;
    line_info_out->x_right_edgeCnt = 0;

    /* 从下往上进行水平搜线，最底层以中间为起点 */
    uint8_t p_start = IMAGE_WIDTH / 2;
    uint8_t P_left_findEdgeCnt = 0;
    uint8_t P_right_findEdgeCnt = 0;
    for (uint16_t i = IMAGE_HEIGHT - 1; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
    {
        /* 定义左右边界x坐标和中线坐标 */
        uint8_t p_left = p_start;
        uint8_t p_right = p_start + 1;
        uint8_t p_mid = p_start;

        /* 左右边界正常搜线标志位 */
        uint8_t p_left_findEdge = 0;
        uint8_t p_right_findEdge = 0;

        /* 左侧搜线，退出判据：触及图像边界或者左侧两个像素点为黑 */
        while (p_left - 1 > IMAGE_COL_KEEPOUT_PIXEL)
        {
            if (image_p[i][p_left] == IMAGE_COLOR_BLACK &&
                image_p[i][p_left - 1] == IMAGE_COLOR_BLACK)
            {
                P_left_findEdgeCnt ++;
                p_left_findEdge = 1;
                break;
            }
            p_left--;
        }
        /* 右侧搜线，退出判据：触及图像边界或者左侧两个像素点为黑 */
        while (p_right + 1 < IMAGE_WIDTH - IMAGE_COL_KEEPOUT_PIXEL)
        {
            if (image_p[i][p_right] == IMAGE_COLOR_BLACK &&
                image_p[i][p_right + 1] == IMAGE_COLOR_BLACK)
            {
                P_right_findEdgeCnt ++;
                p_right_findEdge = 1;
                break;
            }
            p_right++;
        }

        p_mid = (p_right + p_left) / 2;
        /* 更新下一次搜线的起点的横坐标为本次搜线的中点 */
        p_start = p_mid;

        /* 断线退出，判据：当本行中线为black且下行也为black退出循环 */
        if (image_p[i][p_start] == IMAGE_COLOR_BLACK &&
            image_p[i - 1][p_start] == IMAGE_COLOR_BLACK &&
            i< IMAGE_HEIGHT - 20)
        {
            //printf("[edge_x]line break row:%d\n",i);
            break;
        }
        /* 存储边线坐标点到line_info */
        line_info_out->pixel_cnt = IMAGE_HEIGHT - i;
        line_info_out->x_left[i] = p_left;
        line_info_out->x_right[i] = p_right;
        line_info_out->x_left_findEdge[i] = p_left_findEdge;
        line_info_out->x_right_findEdge[i] = p_right_findEdge;
        line_info_out->x_left_edgeCnt = P_left_findEdgeCnt;
        line_info_out->x_right_edgeCnt = P_right_findEdgeCnt;
        //printf("[edge_x]left:%d,right:%d,mid:%d\n", p_left, p_right, (p_left + p_right) / 2);
        //image_p[i][(p_left + p_right) / 2] = IMAGE_COLOR_BLACK;
    }
}

/**
 * @brief 通过边线信息，四方向寻找拐点的朴素搜索方法（求拐点的方法：求边线的二阶导）
 *
 * @param line_info_in [in]边线信息结构体，存储边线横坐标，是否丢线等信息
 * @param inflection_info_out [out]拐点结构体，存储4角拐点的坐标和是否存在的标志位
 */
void tjrc_imageProc_fineInflection(const line_info* line_info_in, inflection_info* inflection_info_out)
{
    /* 清空拐点搜寻标志位：复位到未搜到拐点状态 */
    inflection_info_out->findFlag = 0;
    /* 搜索的起止纵坐标 */
    uint8_t search_begin = 0, search_end = 0;
    /* 单独提取边线坐标信息 */
    const uint8_t* line_left = line_info_in->x_left;
    const uint8_t* line_right = line_info_in->x_right;
    /* 用于记录相邻边线横坐标的差值 */
    int8_t rate_slope[LINE_INFO_MAX_PIXEL];
    int16_t rate_Inflection[LINE_INFO_MAX_PIXEL];

    /*______LEFT_______ */
    /* 计算左边线的横坐标差值(起始点规定为0) */
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i < IMAGE_HEIGHT; i++)
    {
        rate_slope[i] = (i == IMAGE_HEIGHT - IMAGE_INTEREST_REGION) ?
                                0 : line_left[i] - line_left[i - 1];
        //printf("%d ", rate_slope[i]);
    }
    /* 对差值再次求差值（二阶导） */
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i < IMAGE_HEIGHT; i++)
    {
        rate_Inflection[i] = (i == IMAGE_HEIGHT - IMAGE_INTEREST_REGION) ?
            0 : rate_slope[i] - rate_slope[i - 1];
        //printf("%d ", rate_Inflection[i]);
    }
    /* 方向一：从上往下寻找左侧拐点 */
    search_begin = IMAGE_HEIGHT - IMAGE_INTEREST_REGION + 1;
    search_end = IMAGE_HEIGHT;
    for (uint8_t i = search_begin; i < search_end; i++)
    {
        /* 判据：存在两个连续拐点率绝对值之和超过阈值，且两个点的拐点率异号 */
        if (abs(rate_Inflection[i]) + abs(rate_Inflection[i - 1]) > INFLECTION_THRESHOLD &&
            rate_Inflection[i]* rate_Inflection[i - 1] < 0)
        {
            /* 微调i寻找到不丢线的一个点 */
            while (!line_info_in->x_left_findEdge[i] && i - INFLECTION_ADJUST_PIXEL > search_begin)
            {
                i--;
            }
            /* 对坐标进行偏移，消除离散二阶导的影响  */
            i -= INFLECTION_ADJUST_PIXEL;
            /* 标志已找到拐点 */
            inflection_info_out->findFlag |= inflection_upper_left;
            inflection_info_out->x_upper_left = line_left[i];
            inflection_info_out->y_upper_left = i;
            //printf("find inflection[LU] (%d,%d)\n", line_left[i],i);
            break;
        }
    }
    /* 方向二：从下往上寻找左侧拐点 */
    search_begin = IMAGE_HEIGHT - 1;
    search_end = (inflection_info_out->findFlag & inflection_upper_left)?
        inflection_info_out->y_upper_left + 10 : IMAGE_HEIGHT - IMAGE_INTEREST_REGION;
    for (uint8_t i = search_begin; i >= search_end; i--)
    {
        /* 判据：存在两个连续拐点率绝对值之和超过阈值，且两个点的拐点率异号 */
        if (abs(rate_Inflection[i]) + abs(rate_Inflection[i - 1]) > INFLECTION_THRESHOLD &&
            rate_Inflection[i] * rate_Inflection[i - 1] < 0)
        {
            /* 微调i寻找到不丢线的一个点 */
            while (!line_info_in->x_left_findEdge[i-1] && i + INFLECTION_ADJUST_PIXEL < search_begin)
            {
                i++;
            }
            /* 对坐标进行偏移，消除离散二阶导的影响  */
            i += INFLECTION_ADJUST_PIXEL;
            /* 标志已找到拐点 */
            inflection_info_out->findFlag |= inflection_lower_left;
            inflection_info_out->x_lower_left = line_left[i];
            inflection_info_out->y_lower_left = i;
            //printf("find inflection[LL] (%d,%d)\n", line_left[i], i);
            break;
        }
    }

    /*______RIGHT_______ */
    /* 计算左边线的横坐标差值(起始点规定为0) */
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i < IMAGE_HEIGHT; i++)
    {
        rate_slope[i] = (i == IMAGE_HEIGHT - IMAGE_INTEREST_REGION) ?
            0 : line_right[i] - line_right[i - 1];
    }
    /* 对差值再次求差值（二阶导） */
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i < IMAGE_HEIGHT; i++)
    {
        rate_Inflection[i] = (i == IMAGE_HEIGHT - IMAGE_INTEREST_REGION) ?
            0 : rate_slope[i] - rate_slope[i - 1];
        //printf("%d ", rate_Inflection[i]);
    }
    /* 方向三：从上往下寻找右侧拐点 */
    search_begin = IMAGE_HEIGHT - IMAGE_INTEREST_REGION + 1;
    search_end = IMAGE_HEIGHT;
    for (uint8_t i = search_begin; i < search_end; i++)
    {
        /* 判据：存在两个连续拐点率绝对值之和超过阈值，且两个点的拐点率异号 */
        if (abs(rate_Inflection[i]) + abs(rate_Inflection[i - 1]) > INFLECTION_THRESHOLD &&
            rate_Inflection[i] * rate_Inflection[i - 1] < 0)
        {
            /* 微调i寻找到不丢线的一个点 */
            while (!line_info_in->x_left_findEdge[i] && i- INFLECTION_ADJUST_PIXEL > search_begin)
            {
                i--;
            }
            /* 对坐标进行偏移，消除离散二阶导的影响  */
            i -= INFLECTION_ADJUST_PIXEL;
            /* 标志已找到拐点 */
            inflection_info_out->findFlag |= inflection_upper_right;
            inflection_info_out->x_upper_right = line_right[i];
            inflection_info_out->y_upper_right = i;
            //printf("find inflection[RU] (%d,%d)\n", line_right[i], i);
            break;
        }
    }
    /* 方向四：从下往上寻找右侧拐点 */
    search_begin = IMAGE_HEIGHT - 1;
    search_end = (inflection_info_out->findFlag & inflection_upper_right) ?
        inflection_info_out->y_upper_right + 10 : IMAGE_HEIGHT - IMAGE_INTEREST_REGION;
    for (uint8_t i = search_begin; i >= search_end; i--)
    {
        /* 判据：存在两个连续拐点率绝对值之和超过阈值，且两个点的拐点率异号 */
        if (abs(rate_Inflection[i]) + abs(rate_Inflection[i - 1]) > INFLECTION_THRESHOLD &&
            rate_Inflection[i] * rate_Inflection[i - 1] < 0)
        {
            /* 微调i寻找到不丢线的一个点 */
            while (!line_info_in->x_left_findEdge[i] && i + INFLECTION_ADJUST_PIXEL < search_begin)
            {
                i++;
            }
            /* 对坐标进行偏移，消除离散二阶导的影响  */
            i += INFLECTION_ADJUST_PIXEL;
            /* 标志已找到拐点 */
            inflection_info_out->findFlag |= inflection_lower_right;
            inflection_info_out->x_lower_right = line_left[i];
            inflection_info_out->y_lower_right = i;
            //printf("find inflection[RL] (%d,%d)\n", line_right[i], i);
            break;
        }
    }
}

/**
 * @brief 根据拐点信息，对当前情况进行分类进行补线
 *
 * @param line_info_out [out]边线信息结构体，存储边线横坐标，是否丢线等信息
 * @param inflection_info_in [in]拐点结构体，存储4角拐点的坐标和是否存在的标志位
 */
void tjrc_imageProc_patchLine(line_info* line_info_out, inflection_info* inflection_info_in)
{
    uint8_t x_left_lotCnt = IMAGE_INTEREST_REGION - line_info_out->x_left_edgeCnt;
    uint8_t x_right_lotCnt = IMAGE_INTEREST_REGION - line_info_out->x_right_edgeCnt;
    /* 情况1：四拐点，两侧存在丢线：左右补线 */
    if (inflection_info_in->findFlag == inflection_all &&
        x_left_lotCnt > 5 && x_right_lotCnt > 5)
    {
        patchLine(inflection_info_in->y_upper_left, inflection_info_in->y_lower_left, line_info_out->x_left);
        patchLine(inflection_info_in->y_upper_right, inflection_info_in->y_lower_right, line_info_out->x_right);
    }
    /* 情况2：上方2拐点，两侧底部存在丢线：从底部左右补线 */
    if (inflection_info_in->findFlag == inflection_upper &&
        line_info_out->x_left_findEdge[IMAGE_HEIGHT - 1] == 0 &&
        line_info_out->x_right_findEdge[IMAGE_HEIGHT - 1] == 0)
    {
        patchLine(inflection_info_in->y_upper_left, IMAGE_HEIGHT - 1, line_info_out->x_left);
        patchLine(inflection_info_in->y_upper_right, IMAGE_HEIGHT - 1, line_info_out->x_right);
    }
}

/**
 * @brief 将边线信息和中线更新叠加到图像
 *
 * @param image [out]图像
 * @param line_info_in [in]边线信息
 */
void tjrc_imageProc_updateImage(uint8_t* image, line_info* line_info_in)
{
    for (uint8_t i = IMAGE_HEIGHT - 1; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
    {
        if(IMAGE_HEIGHT - line_info_in->pixel_cnt > i)
            break;
        /* 绘制中线 */
        uint8_t x_mid = (line_info_in->x_left[i] + line_info_in->x_right[i]) / 2;
        image[i * IMAGE_WIDTH + x_mid] = 0x00;
        midline[i] = x_mid;
        /* 绘制边线 */
        image[i * IMAGE_WIDTH + line_info_in->x_left[i]] = 0xAA;
        image[i * IMAGE_WIDTH + line_info_in->x_right[i]] = 0xAA;

    }
}

/**
 * @brief （内部函数）根据两个点绘制连线
 *
 * @param y0 起始点的纵坐标
 * @param y1 终止点的纵坐标
 * @param line_x_out [io]线结构体
 */
static void patchLine(uint8_t y0, uint8_t y1, uint8_t* line_x_out)
{
    if (y0 > IMAGE_HEIGHT || y0 < IMAGE_HEIGHT - IMAGE_INTEREST_REGION ||
        y1> IMAGE_HEIGHT || y1 < IMAGE_HEIGHT - IMAGE_INTEREST_REGION)
    {
        printf("[error]patchLine invaild param %d,%d\n",y0,y1);
        return;
    }
    if (y0 > y1)
    {
        uint8_t temp;
        temp = y0;
        y0 = y1;
        y1 = temp;
    }
    float k = ((float)(line_x_out[y1] - line_x_out[y0])) / ((float)(y1 - y0));
    for (uint8_t i = y0; i < y1; i++)
    {
        line_x_out[i] = (uint8_t)((int8_t)line_x_out[y0] + (int8_t)(k * (float)(i - y0)));
    }
}

static float fit_slope(const uint8_t* line,line_info* line_info_in)
{
    float sum_xy = 0.0f, sum_xx = 0.0f;
    float sum_x = 0.0f, sum_y = 0.0f;
    uint8_t n = 0;
    for (uint8_t i = IMAGE_HEIGHT - 1; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
    {
        if(IMAGE_HEIGHT - line_info_in->pixel_cnt > i)
            break;
        sum_xy += i * line[i];
        sum_xx += line[i] * line[i];
        sum_x += line[i];
        sum_y += i;
        n += 1;
    }
    float k = -(n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    k = atan(k) / 3.1415926 * 180;
    k = (k >= 0) ? k : 180 + k;
    k -= 90;
    return k;
}

static float weighted_line(const uint8_t* line,line_info* line_info_in)
{
    static uint8_t weighted_table[IMAGE_HEIGHT];
    static uint8_t first = 1;
    float table_sum = 0;
    if (first)
    {
        for (uint8_t i = IMAGE_HEIGHT - 1; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
        {
            float w = -0.02 * (i - 40) * (i - 40) + 20;
            weighted_table[i] = (w > 0) ? (uint8_t)w : 0;
        }
        first = 0;
    }
    float lsum = 0.0f;
    for (uint8_t i = IMAGE_HEIGHT - 1; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
    {
        if(IMAGE_HEIGHT - line_info_in->pixel_cnt > i)
            break;
        lsum += line[i] * weighted_table[i];
        table_sum+=weighted_table[i];
    }
    lsum /= table_sum;
    return lsum - IMAGE_WIDTH / 2;
}
