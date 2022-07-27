/**
 * @file tjrc_imageProc.c
 * @author YYYDS team. (1951578@tongji.edu.cn)
 * @brief 赛道巡线的核心包含图像处理主要功能函数
 * @version 2.6.1
 * @date 2022-07-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_imageProc.h"


static void patchLine(uint8_t y0, uint8_t y1, uint8_t* line_x_out);
static float fit_slope(const uint8_t* line, line_info* line_info_in);
static float weighted_line(const uint8_t* line, line_info* line_info_in);

/**
 * @brief 图像处理结果全局变量，用于传回cpu0进行运动学控制
 * 
 */
float camera_slope = 0, camera_pixelError = 0;
/**
 * @brief 二值化图像矩阵（二值化矩阵也将用于绘制边线，中线等信息）
 * 尺寸为120*80=9600Byte=9.375KB(x2)
 * 
 */
uint8_t image_bin[IMAGE_HEIGHT][IMAGE_WIDTH];

/**
 * @brief 
 * 
 * @param image_p 
 * @return uint8_t* 
 */
uint8_t* tjrc_imageProc(const uint8_t* image_p)
{
    uint8_t* image_bin_p = image_bin[0];
    static uint8_t midline[IMAGE_HEIGHT];
    static line_info edge_line;
    static inflection_info inflections;

    /* 第一步：二值化 */
    static uint8_t threshold_smooth = 30;
    uint8_t threshold = tjrc_binarization_otsu(image_p, IMAGE_WIDTH, IMAGE_HEIGHT);
    uint8_t threshold_bias = 10;
    threshold += threshold_bias;
    threshold_smooth = (uint8_t)(0.8*threshold_smooth+0.2*threshold);
    //tjrc_log("[imp]threshold=%d(OTSU)\r\n", threshold);
    tjrc_binarization_getBinImage(threshold, image_p, image_bin_p, IMAGE_WIDTH, IMAGE_HEIGHT);

    /* 第二步：搜线，求拐点，补线 */
    if (check_gridLine(image_bin_p))
    {
        tjrc_log("gird line\r\n");
    }
    tjrc_imageProc_searchEdge_x(image_bin_p, &edge_line);
    tjrc_imageProc_fineInflection(&edge_line, &inflections);
    tjrc_imageProc_patchLine(&edge_line, &inflections);

    /* 第三步：更新图像 */
    tjrc_imageProc_updateImage(image_bin_p, &edge_line, &inflections);

    /* 第四步：将中线信息存储到全局变量在内核0调用 */
    for (uint8_t i = IMAGE_HEIGHT - 1; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
    {
        if (IMAGE_HEIGHT - edge_line.pixel_cnt > i)
            continue;
        /* 绘制中线 */
        midline[i] = (edge_line.x_left[i] + edge_line.x_right[i]) / 2;
    }
    /* 获得中线拟合斜率（正负90度）和中线二次加权平均偏移 */
    camera_pixelError = weighted_line(midline, &edge_line);
    camera_slope = fit_slope(midline, &edge_line);
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
        image_p[i] = (uint8_t*)&image[i * IMAGE_WIDTH];
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
                P_left_findEdgeCnt++;
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
                P_right_findEdgeCnt++;
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
            i < IMAGE_HEIGHT - 20)
        {
            tjrc_log("[edge_x]line break row:%d\n", i);
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
        tjrc_log("[edge_x]left:%d,right:%d,mid:%d\n", p_left, p_right, (p_left + p_right) / 2);
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
    const int16_t slope_threshold = 3;
    const uint8_t search_tolerance = 5;
    /* 清空拐点搜寻标志位：复位到未搜到拐点状态 */
    inflection_info_out->findFlag = 0;
    /* 搜索的起止纵坐标 */
    uint8_t search_begin = 0, search_end = 0;
    /* 单独提取边线坐标信息 */
    const uint8_t* line_left = line_info_in->x_left;
    const uint8_t* line_right = line_info_in->x_right;
    /* 用于记录相邻边线横坐标的差值 */
    int8_t rate_slope[LINE_INFO_MAX_PIXEL];

    /*______LEFT_______ */
    /* 计算左边线的横坐标差值(起始点规定为0) */
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i < IMAGE_HEIGHT; i++)
    {
        rate_slope[i] = (i == IMAGE_HEIGHT - IMAGE_INTEREST_REGION) ?
            0 : line_left[i] - line_left[i - 1];
        tjrc_log("%d, ", rate_slope[i]);
    }
    tjrc_log("\n");

    /* 方向一：从上往下寻找左侧拐点 */
    /* 首先确定拐点的搜索范围（顶部（若中间截断，则从截断处开始）--->底部-边缘忽略行） */
    if (IMAGE_HEIGHT - line_info_in->pixel_cnt > IMAGE_HEIGHT - IMAGE_INTEREST_REGION + 1)
    {
        search_begin = IMAGE_HEIGHT - line_info_in->pixel_cnt;
    }
    else
    {
        search_begin = IMAGE_HEIGHT - IMAGE_INTEREST_REGION + 1;
    }
    search_end = IMAGE_HEIGHT - search_tolerance;
    for (uint8_t i = search_begin; i < search_end - 2; i++)
    {
        /* 判据：连续3行的斜率满足如下条件（需要将摄像头尽可能架高，俯视） */
        if (rate_slope[i] >= -slope_threshold &&
            rate_slope[i + 1] >= -slope_threshold &&
            rate_slope[i + 2] < -slope_threshold)
        {
            /* 标志已找到拐点 */
            inflection_info_out->findFlag |= inflection_upper_left;
            inflection_info_out->x_upper_left = line_left[i + 1];
            inflection_info_out->y_upper_left = i + 1;
            tjrc_log("find inflection[LU] (%d,%d)\n", line_left[i + 1], i + 1);
            break;
        }
    }
    /* 方向二：从下往上寻找左侧拐点 */
    search_begin = IMAGE_HEIGHT - search_tolerance;
    search_end = (inflection_info_out->findFlag & inflection_upper_left) ?
        inflection_info_out->y_upper_left + 10 : IMAGE_HEIGHT - IMAGE_INTEREST_REGION;
    for (uint8_t i = search_begin; i >= search_end + 2; i--)
    {

        /* 判据：连续3行的斜率满足如下条件（需要将摄像头尽可能架高，俯视） */
        if (rate_slope[i] <= slope_threshold &&
            rate_slope[i - 1] <= slope_threshold &&
            rate_slope[i - 2] > slope_threshold)
        {
            /* 标志已找到拐点 */
            inflection_info_out->findFlag |= inflection_lower_left;
            inflection_info_out->x_lower_left = line_left[i - 2];
            inflection_info_out->y_lower_left = i - 2;
            tjrc_log("find inflection[LL] (%d,%d)\n", line_left[i - 2], i - 2);
            break;
        }
    }

    /*______RIGHT_______ */
    /* 计算左边线的横坐标差值(起始点规定为0) */
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i < IMAGE_HEIGHT; i++)
    {
        rate_slope[i] = (i == IMAGE_HEIGHT - IMAGE_INTEREST_REGION) ?
            0 : line_right[i] - line_right[i - 1];
        tjrc_log("%d, ", rate_slope[i]);
    }
    tjrc_log("\n");

    /* 方向三：从上往下寻找右侧拐点 */
    if (IMAGE_HEIGHT - line_info_in->pixel_cnt > IMAGE_HEIGHT - IMAGE_INTEREST_REGION + 1)
    {
        search_begin = IMAGE_HEIGHT - line_info_in->pixel_cnt;
    }
    else
    {
        search_begin = IMAGE_HEIGHT - IMAGE_INTEREST_REGION + 1;
    }
    search_end = IMAGE_HEIGHT - search_tolerance;
    for (uint8_t i = search_begin; i < search_end - 2; i++)
    {
        /* 判据：连续3行的斜率满足如下条件（需要将摄像头尽可能架高，俯视） */
        if (rate_slope[i] <= slope_threshold &&
            rate_slope[i + 1] <= slope_threshold &&
            rate_slope[i + 2] > slope_threshold)
        {
            /* 标志已找到拐点 */
            inflection_info_out->findFlag |= inflection_upper_right;
            inflection_info_out->x_upper_right = line_right[i + 1];
            inflection_info_out->y_upper_right = i + 1;
            tjrc_log("find inflection[RU] (%d,%d)\n", line_right[i + 1], i + 1);
            break;
        }
    }
    /* 方向四：从下往上寻找右侧拐点 */
    search_begin = IMAGE_HEIGHT - search_tolerance;
    search_end = (inflection_info_out->findFlag & inflection_upper_right) ?
        inflection_info_out->y_upper_right + 10 : IMAGE_HEIGHT - IMAGE_INTEREST_REGION;
    for (uint8_t i = search_begin; i >= search_end + 2; i--)
    {
        /* 判据：连续3行的斜率满足如下条件（需要将摄像头尽可能架高，俯视） */
        if (rate_slope[i] >= -slope_threshold &&
            rate_slope[i - 1] >= -slope_threshold &&
            rate_slope[i - 2] < -slope_threshold)
        {
            /* 标志已找到拐点 */
            inflection_info_out->findFlag |= inflection_lower_right;
            inflection_info_out->x_lower_right = line_right[i - 2];
            inflection_info_out->y_lower_right = i - 2;
            tjrc_log("find inflection[RL] (%d,%d)\n", line_right[i - 2], i - 2);
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
 * @brief 将边线信息，拐点信息和中线更新叠加到图像
 * 
 * @param image [out]图像
 * @param line_info_in [in]边线信息
 * @param inflection_info_in [in]拐点信息
 */
void tjrc_imageProc_updateImage(uint8_t* image, line_info* line_info_in, inflection_info* inflection_info_in)
{
    for (uint8_t i = IMAGE_HEIGHT - 1; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
    {
        if (IMAGE_HEIGHT - line_info_in->pixel_cnt > i)
            continue;
        /* 绘制中线 */
        uint8_t x_mid = (line_info_in->x_left[i] + line_info_in->x_right[i]) / 2;
        image[i * IMAGE_WIDTH + x_mid] = 0x00;
        /* 绘制边线 */
        image[i * IMAGE_WIDTH + line_info_in->x_left[i]] = 0xAA;
        image[i * IMAGE_WIDTH + line_info_in->x_right[i]] = 0xAA;

    }
    /* 添加拐点（0x33特殊映射到红色） */
    if (inflection_info_in->findFlag & inflection_upper_left)
        image[inflection_info_in->y_upper_left * IMAGE_WIDTH + inflection_info_in->x_upper_left] = 0x33;
    if (inflection_info_in->findFlag & inflection_upper_right)
        image[inflection_info_in->y_upper_right * IMAGE_WIDTH + inflection_info_in->x_upper_right] = 0x33;
    if (inflection_info_in->findFlag & inflection_lower_left)
        image[inflection_info_in->y_lower_left * IMAGE_WIDTH + inflection_info_in->x_lower_left] = 0x33;
    if (inflection_info_in->findFlag & inflection_lower_right)
        image[inflection_info_in->y_lower_right * IMAGE_WIDTH + inflection_info_in->x_lower_right] = 0x33;
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
        tjrc_log("[error]patchLine invaild param %d,%d\n", y0, y1);
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

/**
 * @brief 判断当前区域有无斑马线
 * 
 * @param image 
 * @return uint8_t 
 */
uint8_t check_gridLine(const uint8_t* image)
{
    /* 定义扫描起止行（上0下80） */
    const uint8_t row_begin = 45, row_end = 70;
    /* 定义扫描斑马线黑线宽度上下阈值 */
    const uint8_t grid_min = 5, grid_max = 10;
    /* 定义斑马线黑块个数判断阈值(正常有7块) */
    const uint8_t blocks_min = 5, blocks_max = 10;
    /* 定义最少判定为斑马线的正确判断次数 */
    const uint8_t times_min = 3;
    /* 记录正确判断次数 */
    uint8_t times = 0;
    for (uint8_t y = row_begin; y <= row_end; y++)
    {
        uint8_t blocks = 0;
        uint8_t cursor = 0;    //指向栈顶的游标
        const uint8_t* row_ptr = &(image[IMAGE_WIDTH * y]);
        //for (uint8_t x = IMAGE_COL_KEEPOUT_PIXEL; x < IMAGE_WIDTH - IMAGE_COL_KEEPOUT_PIXEL; x++)
        //  if (row_ptr[x] == IMAGE_COLOR_BLACK)
        //      tjrc_log("*");
        //  else
        //      tjrc_log("_");
        //tjrc_log("\n");
        /* 横向遍历当前行 */
        for (uint8_t x = IMAGE_COL_KEEPOUT_PIXEL; x < IMAGE_WIDTH - IMAGE_COL_KEEPOUT_PIXEL; x++)
        {
            /* 若当前为黑色像素，则累加黑色像素宽度；若为白色，则结束宽度累加，
                计算当前黑块宽度，如果宽度合适，则算作一次合格黑色斑马线 */
            if (row_ptr[x] == IMAGE_COLOR_BLACK)
            {
                cursor++;
            }
            else
            {
                if (cursor >= grid_min && cursor <= grid_max)
                    blocks++;
                cursor = 0;
            }
        }
        /* 黑块个数必须满足要求 */
        if (blocks >= blocks_min && blocks <= blocks_max)
        {
            times++;
        }
    }
    /* 正确判断次数必须满足要求 */
    if (times >= times_min)
        return 1;
    else
        return 0;
}


/**
 * @brief 求当前直线斜率
 *
 * @param line 中线横坐标数组mid[height]的指针
 * @param line_info_in 边线信息
 * @return float 斜率，-90deg ~ 90deg
 */
static float fit_slope(const uint8_t* line, line_info* line_info_in)
{
    float sum_xy = 0.0f, sum_xx = 0.0f;
    float sum_x = 0.0f, sum_y = 0.0f;
    uint8_t n = 0;
    for (uint8_t i = IMAGE_HEIGHT - 1; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
    {
        if (IMAGE_HEIGHT - line_info_in->pixel_cnt > i)
            break;
        sum_xy += i * line[i];
        sum_xx += line[i] * line[i];
        sum_x += line[i];
        sum_y += i;
        n += 1;
    }
    /* 若无法获取斜率信息，则返回0 */
    if (n * sum_xx - sum_x * sum_x == 0)
    {
        return 0;
    }
    else
    {
        /* 最小二乘拟合，返回角度值 */
        float k = -(n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        k = atan(k) / 3.1415926 * 180;
        k = (k >= 0) ? k : 180 + k;
        k -= 90;
        return k;
    }
}

/**
 * @brief 加权[权值曲线：-0.02(x-40)^2+20]平均中线横坐标
 *
 * @param line 中线横坐标数组mid[height]的指针
 * @param line_info_in 边线信息
 * @return float -60~60
 */
static float weighted_line(const uint8_t* line, line_info* line_info_in)
{
    static uint8_t weighted_table[IMAGE_HEIGHT];
    static uint8_t first = 1;
    float table_sum = 0;

    /* 首次调用，生成权值表 */
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
        if (IMAGE_HEIGHT - line_info_in->pixel_cnt > i)
            break;
        lsum += line[i] * weighted_table[i];
        table_sum += weighted_table[i];
    }
    lsum /= table_sum;
    if (table_sum != 0)
    {
        return lsum - IMAGE_WIDTH / 2;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 图像处理调试信息输出（可以通过注释影藏调试信息）
 * 
 * @param format 
 * @param ... 
 * @return int 
 */
int tjrc_log(char* format, ...)
{
    /*static char buff[128];
    va_list ap;
    va_start(ap, format);
    vsnprintf(buff, sizeof(buff), format, ap);
    va_end(ap);
    printf("%s", buff);*/
    return 0;
}

