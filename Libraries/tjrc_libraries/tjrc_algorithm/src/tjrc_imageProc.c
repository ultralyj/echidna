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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "tjrc_imageProc.h"

//宏定义控制变量
#define tjrc_gui       0     //与上位机的交互控制
#define tjrc_debug     0     //与串口调试的控制
#define tjrc_display   1     //与tft屏幕显示的控制

IFX_ALIGN(4) uint8_t image_bin[IMAGE_HEIGHT][IMAGE_WIDTH];
line_info edge_line;
inflection_info inflections;

static void patchLine(uint8_t y0, uint8_t y1, uint8_t* line_x_out);

/* 图像全局数组 */
uint8_t imageout[MT9V03X_H][MT9V03X_W];
uint8_t image[MT9V03X_H][MT9V03X_W];            //使用二维指针image代替信息图像，便于后期二值化图像和深度图像的转换

/* 二值化控制变量 */
uint8_t threshold=90;                           //二值化阈值
uint8_t binimage_flag = 0;                      //二值化标志，默认没有二值化

/* 边界全局结构体数组 */
struct LEFT_EDGE L_edge[150];                   //左边界结构体  #全局 使用
struct LEFT_EDGE R_edge[150];                   //右边界结构体  #全局 使用
struct LEFT_EDGE M_Line[150];                   //中线结构体，备用选择2
struct LEFT_EDGE Last_M_Line[150];              //上次中线结构体
struct LEFT_EDGE MID_LINE[150];                 //中线结构体，备用选择1

/* 最终的边线数组 */
struct LEFT_EDGE* Mid_Line = M_Line;            //中线结构体【实际使用】
struct LEFT_EDGE L_edge_use[80];                //左边界结构体 最后使用  #全局 使用
struct LEFT_EDGE R_edge_use[80];                //右边界结构体 最后使用  #全局 使用

/* 控制变量 */
uint8_t effective_points = 65;                  //用于防止单段边线长度过长导致拐点误判 #全局 控制 使用
uint8_t line_amount = 100;                      //当下边线大于该数，强制退出搜线
uint8_t edge_count_amount = 3;                  //连续搜到边界次数，中间搜线开始条件
float dist_control = 100;                       //判断是否为直线的控制参数  #全局 控制 使用
uint8_t L_edge_end_row = 7;                     //左边界行结束点
uint8_t R_edge_end_row = 7;                     //右边界行结束点
uint8_t M_edge_end_row = 5;                     //中边界行结束点
uint8_t min_col = 3, max_col = 177;             //搜索结束列值
uint8_t L_edge_start_col = 3;                   //左边界列搜索起始点
uint8_t R_edge_start_col = 117;                 //右边界列搜索起始点
uint8_t L_basic_row_start = MT9V03X_H - 1;      //左边线搜线开始点
uint8_t R_basic_row_start = MT9V03X_H - 2;      //右边线搜线开始点
uint8_t L_lost_ = 10;                           //左丢线限制次数
uint8_t R_lost_ = 10;                           //右丢线限制次数
float line_blur_kernel = 7;                     //边线三角滤波核的大小
uint8_t L_edge_lost_start_col = 4;              //中间左丢线列搜索起始点
uint8_t R_edge_lost_start_col = 115;            //中间右丢线列搜索起始点
uint8_t dist = 4;                               //求拐点角度的点数间隔
uint8_t Image_H = MT9V03X_H;
uint8_t Image_W = MT9V03X_W;

/* 中线处理 未使用 */
uint8_t searchpoint = 1;                        //正常求出来的中线点标志，用于结构体.flag，便于后期处理
uint8_t nihepoint = 2;                          //拟合求出来的中线点标志，用于结构体.flag，便于后期处理
extern uint8_t Road_Width[MT9V03X_H];           //质量矩，用于丢线
uint8_t l_display_cnt = 0;
uint8_t r_display_cnt = 0;
uint8_t value3[30];
uint8_t run_flag = 0;                           //  发车/停车标志  #全局 未使用
uint8_t stop_flag = 0;
uint8_t center_turn_flag;                       //0 左趋势， 1 右趋势
uint8_t center_biaoxiang_arry[5];

/* 直线斜率数据存放数组 */
float line_left_bottom[2];
float line_left_top[2];
float line_right_top[2];
float line_right_bottom[2];
float k_l=0,k_r=0,b_l=0,b_r=0;

/* 拐点标志 */
uint8_t dot_left_bottom_l = 0;                  //左下拐点，趋势左
uint8_t dot_left_bottom_r = 0;                  //左下拐点，趋势右
uint8_t dot_left_top_l = 0;                     //左上拐点，趋势左
uint8_t dot_left_top_r = 0;                     //左上拐点，趋势右
uint8_t dot_right_bottom_l = 0;                 //右下拐点，趋势左
uint8_t dot_right_bottom_r = 0;                 //右下拐点，趋势右
uint8_t dot_right_top_l = 0;                    //右上拐点，趋势左
uint8_t dot_right_top_r = 0;                    //右上拐点，趋势右
uint8_t L_corner_flag = 0;                      //左拐点存在标志
uint8_t R_corner_flag = 0;                      //右拐点存在标志

/* 拐点存放变量 */
uint8_t L_corner_row = 0;                       //左拐点所在行
uint8_t L_corner_col = 0;                       //左拐点所在列
int32_t L_corner_angle = 0;                     //左拐点角度
uint8_t R_corner_row = 0;                       //右拐点所在行
uint8_t R_corner_col = 0;                       //右拐点所在列
int32_t R_corner_angle = 0;                     //右拐点角度
uint8_t L_top_corner_start = 0;                 //左上拐点开始序号
uint8_t R_top_corner_start = 0;                 //右上拐点开始序号
uint8_t L_top_i = 0;                            //左上拐角对应i
uint8_t L_beh_i = 0;                            //左下拐角对应i
uint8_t R_top_i = 0;                            //右上拐角对应i
uint8_t R_beh_i = 0;                            //右下拐角对应i
uint8_t center_lost_corner_row_l = 0;           //中间左丢线开始拐点行坐标
uint8_t center_lost_corner_col_l = 0;           //中间左丢线开始拐点列坐标
int32_t L_center_lost_corner_angle = 0;         //中间左丢线开始左拐点角度
uint8_t center_lost_corner_row_r = 0;           //中间右丢线开始拐点行坐标
uint8_t center_lost_corner_col_r = 0;           //中间右丢线开始拐点列坐标
int32_t R_center_lost_corner_angle = 0;         //中间右丢线开始左拐点角度

/* 第二段边线存放变量 */
uint8_t center_lost_row_l = 0;                  //中间左丢线开始行坐标
uint8_t center_lost_col_l = 0;                  //中间左丢线开始列坐标
uint8_t center_lost_row_r = 0;                  //中间右丢线开始行坐标
uint8_t center_lost_col_r = 0;                  //中间右丢线开始列坐标

/* 直线总体偏差 */
float dist_sum_left_bottom = 0;
float dist_sum_left_top = 0;
float dist_sum_right_bottom = 0;
float dist_sum_right_top = 0;

/* 使能控制变量 */
uint8_t enable_balinyu = 1;                     //八邻域爬线使能标志，默认使能
uint8_t enable_midline = 1;                     //使能中线拟合，默认不开启
uint8_t enable_midpross = 1;                    //使能中线处理，默认开启
uint8_t enable_check_l_r_edge_same = 0;         //使能左、右边线是否重合，默认不开启
uint8_t edge_process_flag = 0;                  //使能边线处理
uint8_t enable_L_corner = 1;                    //左拐点搜索使能标志 默认使能
uint8_t enable_R_corner = 1;                    //右拐点搜索是能标志 默认使能

/* 全局标志位 */
uint8_t L_start_lost = 0;                       //左边线起始丢线标志
uint8_t R_start_lost = 0;                       //右边线起始丢线标志
uint8_t left_findflag, right_findflag;          //边线存在标志位
uint8_t center_lost_flag_l = 0;                 //中间左丢线标志位;
uint8_t center_lost_flag_r = 0;                 //中间右丢线标志位
uint8_t is_line_left_bottom = 0;                //是否为直线标志
uint8_t is_line_left_top = 0;
uint8_t is_line_right_bottom = 0;
uint8_t is_line_right_top = 0;

/* 计数控制变量 */
uint8_t L_search_edge_count = 0, R_search_edge_count = 0;       //搜索到边界
uint8_t line_point_count_left, line_point_count_right;          //左右线有效点计数
uint8_t L_edge_count = 0, R_edge_count = 0;                     //左右边点的个数
uint8_t Mid_count = 0;                                          //中线点的个数
uint8_t center_arry_count;
uint8_t line_lose_center_left;
uint8_t line_lose_center_right;
uint8_t dire_left;                                              //记录上一个点的相对位置
uint8_t dire_right;                                             //记录上一个点的相对位置
uint8_t L_lost_count = 0;                                       //左丢线计数
uint8_t R_lost_count = 0;                                       //右丢线计数
uint8_t pre_L_edge_count = 0;
uint8_t pre_R_edge_count = 0;
uint8_t num_cnt = 0;                                            //记录连续水平点的个数
uint8_t L_count = 0;
uint8_t R_count = 0;

/* 方差 */
float statistics1 = 0;

/*用于显示屏显示*/
char buf[32];

/* 八邻域搜线方向数组 */
/* 前进方向定义：
*   0
* 3   1
*   2   */
int32_t dir_front[4][2] = { {-1,  0},{0,  1},{1,  0},{0, -1} };

/* 前进方向定义：
* 0   1
*
* 3   2  */
int32_t dir_frontleft[4][2] = { {-1, -1},{-1,  1},{1,  1},{1, -1} };

/* 前进方向定义：
* 3   0
*
* 2   1  */
int32_t dir_frontright[4][2] = { {-1,  1},{1,  1},{1, -1},{-1, -1} };

uint8_t Road_Width[LCDH] = { 113,112,111,110,109,108,107,106,105,104,103,102,101,100,99 ,98 ,97 ,96 ,95 ,94 ,
                        93 ,92 ,91 ,90 , 89, 88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74,
                        73 ,72 ,71 ,70 ,69 , 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54,
                        53 ,52 , 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34 };

/* 控制参数列表 */
tjrc_image_info tjrc_imageProc_ctr = {
   //###控制参数列表
   77,    //左边线行搜线开始点
   77,    //右边线行搜线开始点
   5,     //左边界列搜索起始点
   115,   //右边界列搜索起始点
   70,   //左右边界搜点时最多允许的点
   70,   //左右边界搜点时最多允许的点
   3,     //最小列坐标，即边界值
   116,   //最大列坐标，即边界值
   10,    //左丢线限制次数
   10,    //右丢线限制次数
   10,     //中间左丢线列搜索起始点
   110,   //中间右丢线列搜索起始点
   4,     //求拐点角度的点数间隔
   65,    //用于防止单段边线长度过长导致拐点误判 #全局 控制 使用
   100,   //判断是否为直线的控制参数  #全局 控制 使用
   100,   //当下边线大于该数，强制退出搜线
   3,     //连续搜到边界后限制退出次数，中间搜线开始条件

   //###标志位参数列表
   1,     //使能二值化标志位
   0,     //使能二值化（0：直接用灰度判断，1：用二值化后的图像判断，2：单点sobel判断）
   1,     //使能八邻域搜线标志位
   1,     //使能拐点搜索标准位
   0      //使能识别元素标志位
};

/*!
 * @brief      图像处理主函数
 * @param      imagein  (uint8_t*)输入图像数组
 * @param      info     (tjrc_image_info)输入控制参数结构体
 * @relative   XLW_otsuThreshold()
               GetOSTU()
               SobelThreshold()
               SobelAutoThreshold()
               Get_01_Value()
               get_binImage()
               tjrc_findstartpoint_row()
               tjrc_findstartpoint_col()
               tjrc_findline_lefthand()
               tjrc_findline_righthand()
               tjrc_findturnpoint_leftbottom()
               tjrc_findturnpoint_lefttop()
               tjrc_findturnpoint_righttop()
               tjrc_findturnpoint_rightbottom()
               clear_point()
               black_()
               white_()
               FitStraightLine()
 * @return     none
 * @author     YYY
 * @version    V1.2
 * @date       2022/4/12
 */
void tjrc_imageProcess(uint8_t* image, tjrc_image_info* info)
{
   /* 内部控制变量 */
   uint8_t max_col = Image_W - 5, min_col = 5;         //最大/小列坐标，即边界值  #内部 控制 使用
   uint8_t L_search_amount = 70, R_search_amount = 70;  //左右边界搜点时最多允许的点  #内部 控制 使用
   uint8_t jilu_row_l = 0, jilu_col_l = 0, jilu_row_r = 0, jilu_col_r = 0;  //记录搜索到的基础边界点行列值   #内部 使用

   /* 获取参数控制列表 */
   L_basic_row_start = tjrc_imageProc_ctr.L_basic_row_start;
   R_basic_row_start = tjrc_imageProc_ctr.R_basic_row_start;
   L_edge_start_col = tjrc_imageProc_ctr.L_edge_start_col;
   R_edge_start_col = tjrc_imageProc_ctr.R_edge_start_col;
   L_search_amount = tjrc_imageProc_ctr.L_search_amount;
   R_search_amount = tjrc_imageProc_ctr.R_search_amount;
   min_col = tjrc_imageProc_ctr.min_col;
   max_col = tjrc_imageProc_ctr.max_col;
   L_lost_ = tjrc_imageProc_ctr.L_lost_;
   R_lost_ = tjrc_imageProc_ctr.R_lost_;
   L_edge_lost_start_col = tjrc_imageProc_ctr.L_edge_lost_start_col;
   R_edge_lost_start_col = tjrc_imageProc_ctr.R_edge_lost_start_col;
   dist = tjrc_imageProc_ctr.dist;
   effective_points=tjrc_imageProc_ctr.effective_points;               //用于防止单段边线长度过长导致拐点误判 #全局 控制 使用
   dist_control=tjrc_imageProc_ctr.dist_control;
   line_amount=tjrc_imageProc_ctr.line_amount;
   edge_count_amount=tjrc_imageProc_ctr.edge_count_amount;
   L_edge_end_row = 7;                                                 //左边界行结束点
   R_edge_end_row = 7;

   /* 边界初始化 */
   clear_point();

   /* 二值化 */
   if (tjrc_imageProc_ctr.enable_bin)
   {
       threshold = tjrc_binarization_otsu(image, IMAGE_WIDTH, IMAGE_HEIGHT);
       uint8_t threshold_bias = 5;
       threshold += threshold_bias;

       /* 图片压缩,并传输至上位机 */
#if tjrc_gui
       tjrc_wireless_sendImage(image,threshold);
#endif

#if tjrc_debug
       printf("[imp]threshold=%d(OTSU)\n", threshold);
#endif

       /* 显示在屏幕中 */
#if tjrc_display
       tjrc_st7735_dispbin(image, 120, 80, 4, 24, 0xFFFF,threshold);
#endif
   }

   /* 八邻域搜线 */
   if (tjrc_imageProc_ctr.enable_balinyu)//如果使能八邻域爬线即一般处理
   {
       /* 变量初始化 */
       L_edge_count = 0;//左边点个数清0
       R_edge_count = 0;//右边点个数清0
       int32_t exist_edge_size = 0;   //判断是否存在左/右边界

       /* 寻找搜线起始点 */
       exist_edge_size = tjrc_findstartpoint_row(image,L_basic_row_start, 0);              //寻找左线开始点，并判断是否存在当前边
       if (exist_edge_size >= 0) { jilu_row_l = L_basic_row_start; jilu_col_l = exist_edge_size; left_findflag = 1; }

       exist_edge_size = tjrc_findstartpoint_row(image,R_basic_row_start, (uint8_t)(1));   //寻找右线开始点，并判断是否存在当前边
       if (exist_edge_size >= 0) { jilu_row_r = R_basic_row_start; jilu_col_r = exist_edge_size; right_findflag = 1; }

       #if tjrc_debug
           rt_kprintf("左边线起始点：(x=%d,y=%d)\r\n", jilu_row_l, jilu_col_l);
           rt_kprintf("右边线起始点：(x=%d,y=%d)\r\n", jilu_row_r, jilu_col_r);
       #endif

       /* 进行八邻域搜线  */
       if (left_findflag)//如果左边界点存在并找到,则开始爬线
       {
           /* 变量初始化 */
           L_edge[0].row = jilu_row_l;
           L_edge[0].col = jilu_col_l;
           L_edge[0].flag = 1;
           uint8_t curr_row = jilu_row_l;           //初始化行坐标
           uint8_t curr_col = jilu_col_l;           //初始化列坐标
           dire_left = 0;                           //初始化上个边界点的来向
           center_turn_flag = 1;                    //初始化在未标定状态
           center_lost_flag_l = 0;                  //中间左丢线标志位

           /* 开始搜第一段边线，最多取70个点，不会往下搜，共7个方位 */
           tjrc_findline_lefthand(image,jilu_row_l, jilu_col_l, L_search_amount, 1);

           /* 寻找第一段边线的拐点 */
           if (!L_start_lost)                       //只有左下边线情况
           {
               tjrc_findturnpoint_leftbottom(0, L_edge_count - 1, L_search_amount);      //寻找拐点
           }
           else if (L_start_lost)                   //只有左上边线情况
           {
               tjrc_findturnpoint_lefttop(1, L_edge_count - 1, L_search_amount);         //寻找拐点
           }

           /* 第二段边线起始点 */
           L_top_corner_start = L_edge_count + 1;

           /* 寻找是否有第二段边线起始点 */
           tjrc_findstartpoint_col(image,L_corner_row, L_corner_col, 0);

           /* 搜第二段边线，寻找拐点 */
           if (center_lost_flag_l)
           {
               tjrc_findline_lefthand(image,center_lost_row_l, center_lost_col_l, 70, L_edge_count + 1);
               tjrc_findturnpoint_lefttop(L_top_corner_start, L_edge_count - 1, 70);
           }

           /* 显示在屏幕中 */
           #if tjrc_display
               for(int32_t i=0;i<L_edge_count;i++)
               {
                   tjrc_st7735_drawPoint(4+L_edge[i].col,28+L_edge[i].row,0xF800);
               }
           #endif
       }
       if (right_findflag)//如果右边界存在并搜到
       {
           /* 变量初始化 */
           R_edge[0].row = jilu_row_r;
           R_edge[0].col = jilu_col_r;
           R_edge[0].flag = 1;
           uint8_t curr_row = jilu_row_r;           //初始化行坐标
           uint8_t curr_col = jilu_col_r;           //初始化列坐标
           dire_right = 0;                          //初始化上个边界点的来向
           center_lost_flag_r = 0;                  //中间右丢线标志位

           /* 开始搜第一段边线，最多取70个点，不会往下搜，共7个方位 */
           tjrc_findline_righthand(image,jilu_row_r, jilu_col_r, R_search_amount, 1);      //寻找拐点

           /* 寻找第一段边线的拐点 */
           if (!R_start_lost)                       //只有左下边线情况
           {
               tjrc_findturnpoint_rightbottom(0, R_edge_count - 1, R_search_amount);       //寻找拐点
           }
           else if (R_start_lost)                   //只有左上边线情况
           {
               tjrc_findturnpoint_righttop(1, R_edge_count - 1, R_search_amount);
           }

           /* 第二段边线起始点 */
           R_top_corner_start = R_edge_count + 1;

           /* 寻找是否有第二段边线起始点 */
           tjrc_findstartpoint_col(image,R_corner_row, R_corner_col, 1);

           /* 搜第二段边线，寻找拐点 */
           if (center_lost_flag_r)
           {
               tjrc_findline_righthand(image,center_lost_row_r, center_lost_col_r, 70, R_edge_count + 1);
               tjrc_findturnpoint_righttop(R_top_corner_start, R_edge_count - 1, 70);
           }

           /* 显示在屏幕中 */
           #if tjrc_display
               for(int32_t i=0;i<R_edge_count;i++)
               {
                   tjrc_st7735_drawPoint(4+R_edge[i].col,28+R_edge[i].row,0x07E0);
               }
           #endif
       }
   }

    #if tjrc_debug
       printf("dot_left_bottom_l:%d\n", dot_left_bottom_l);
       printf("dot_left_bottom_r:%d\n", dot_left_bottom_r);
       printf("dot_left_top_l:%d\n", dot_left_top_l);
       printf("dot_left_top_r:%d\n", dot_left_top_r);
       printf("dot_right_bottom_l:%d\n", dot_right_bottom_l);
       printf("dot_right_bottom_r:%d\n", dot_right_bottom_r);
       printf("dot_right_top_l:%d\n", dot_right_top_l);
       printf("dot_right_top_r:%d\n", dot_right_top_r);

       printf("distance bottom left:%f\n", dist_sum_left_bottom);
       printf("distance top left:%f\n", dist_sum_left_top);
       printf("distance bottom right:%f\n", dist_sum_right_bottom);
       printf("distance top right:%f\n", dist_sum_right_top);

       printf("is_line_left_bottom:%d\n", is_line_left_bottom);
       printf("is_line_left_top:%d\n", is_line_left_top);
       printf("is_line_right_bottom:%d\n", is_line_right_bottom);
       printf("is_line_right_top:%d\n", is_line_right_top);
    #endif

    /* 显示在屏幕中 */
    #if tjrc_display
        sprintf(buf,"lblr:%d %d",dot_left_bottom_l,dot_left_bottom_r);
        tjrc_st7735_dispStr612(2,150,(uint8_t*)buf,RGB565_GREEN);
        sprintf(buf,"ltlr:%d %d",dot_left_top_l,dot_left_top_r);
        tjrc_st7735_dispStr612(2,138,(uint8_t*)buf,RGB565_GREEN);
        sprintf(buf,"rblr:%d %d",dot_right_bottom_l,dot_right_bottom_r);
        tjrc_st7735_dispStr612(54,150,(uint8_t*)buf,RGB565_GREEN);
        sprintf(buf,"rtlr:%d %d",dot_right_top_l,dot_right_top_r);
        tjrc_st7735_dispStr612(54,138,(uint8_t*)buf,RGB565_GREEN);
        sprintf(buf,"%d %d",is_line_left_top,is_line_right_top);
        tjrc_st7735_dispStr612(108,138,(uint8_t*)buf,RGB565_WHITE);
        sprintf(buf,"%d %d",is_line_left_bottom,is_line_right_bottom);
        tjrc_st7735_dispStr612(108,150,(uint8_t*)buf,RGB565_WHITE);
    #endif

    /* 判断元素，生成最终的边线数组  */
   if (tjrc_imageProc_ctr.enable_element)
   {
       /* 判断正十字 */
       if (dot_left_bottom_l && dot_right_bottom_r && dot_right_top_r && dot_left_top_l)  //正十字
       {
//           printf("this is zheng shi zi\n");
           tjrc_st7735_dispStr612(5,126,"zheng shi zi",0xFFFF);
           tjrc_greekcross_patchLine(L_corner_row, L_corner_col, center_lost_corner_col_l, center_lost_corner_row_l, R_corner_row, R_corner_col, center_lost_corner_col_r, center_lost_corner_row_r);
       }

       /* 判断中十字 */
       if (!dot_left_bottom_l && !dot_right_bottom_r && dot_right_top_r && dot_left_top_l)//中十字
       {
//           printf("this is zhong shi zi\n");
           tjrc_st7735_dispStr612(5,126,"zhong shi zi",0xFFFF);
           tjrc_centercross_patchLine(L_top_i, R_top_i);
       }

       /* 判断左斜入十字 */
       if (!dot_left_bottom_l && dot_right_bottom_r && dot_right_top_r && dot_left_top_l)//左斜入十字
       {
//           printf("this is zuo xieru shi zi\n");
           tjrc_st7735_dispStr612(5,126,"zuo xieru shi zi",0xFFFF);
           tjrc_oblique_leftcross_patchLine(L_top_i, R_top_i);
       }

       /* 判断右斜入十字 */
       if (dot_left_bottom_l && !dot_right_bottom_r && dot_right_top_r && dot_left_top_l)//右斜入十字
       {
//           printf("this is you xieru shi zi\n");
           tjrc_st7735_dispStr612(5,126,"you xieru shi zi",0xFFFF);
           tjrc_oblique_rightcross_patchLine(L_top_i, R_top_i);
       }

       /* 判断左圆环 */
       if (dot_left_bottom_l && dot_left_top_l && is_line_right_top || is_line_right_bottom)//左圆环
       {
//           printf("this is zuo yuan huan\n");
           tjrc_st7735_dispStr612(5,126,"zuo yuan huan",0xFFFF);
           tjrc_leftring_patchLine(L_corner_row, L_corner_col, center_lost_corner_col_l, center_lost_corner_row_l);
       }

       /* 判断右圆环 */
       if (dot_right_bottom_r && dot_right_top_r && is_line_left_top || is_line_left_bottom)//右圆环
       {
//           printf("this is you yuan huan\n");
           tjrc_st7735_dispStr612(5,126,"you yuan huan",0xFFFF);
           tjrc_rightring_patchLine(R_corner_row, R_corner_col, center_lost_corner_col_r, center_lost_corner_row_r);
       }

       /* 判断左弯 */
       if (dot_left_bottom_l && dot_right_bottom_l) //左弯
       {
//           printf("this is zuo wan\n");
           tjrc_st7735_dispStr612(5,126,"zuo wan",0xFFFF);
           tjrc_leftturn_patchLine(L_top_corner_start);
       }

       /* 判断右弯 */
       if (dot_left_bottom_r && dot_right_bottom_r) //右弯
       {
//           printf("this is you wan\n");
           tjrc_st7735_dispStr612(5,126,"you wan",0xFFFF);
           tjrc_rightturn_patchLine(R_top_corner_start);
       }

       //if (dot_left_top_l && is_line_right)//左入圆环 #状态机中使用
       //{
       //  printf("this is zuo ru yuan huan\n");
       //  //printf("\n直线顶点为：(%d,%d),(%d,%d)",L_corner_col, L_corner_row, center_lost_corner_col_l, center_lost_corner_row_l);
       //  line_left_top[0] = (float)(center_lost_corner_col_l - 119) / (float)(center_lost_corner_row_l - 79);   //k
       //  line_left_top[1] = (float)(119 - line_left_top[0] * 79);  //b
       //  //printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
       //  //显示该直线
       //  uint8_t m1, m2;
       //  if (79 > center_lost_corner_row_l)
       //  {
       //      m1 = center_lost_corner_row_l;
       //      m2 = 79;
       //  }
       //  else
       //  {
       //      m2 = center_lost_corner_row_l;
       //      m1 = 79;
       //  }
       //  //printf("  m1=%d,m2=%d\n", m1, m2);
       //  for (int32_t i = m1; i < m2; i++)
       //  {
       //      L_edge_use[i].row = i;
       //      L_edge_use[i].col = (int32_t)(i * line_left_top[0] + line_left_top[1]);
       //      //printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
       //      if (L_edge_use[i].col < 0)
       //      {
       //          L_edge_use[i].col = 0;
       //      }
       //      if (L_edge_use[i].col > 119)
       //      {
       //          L_edge_use[i].col = 119;
       //      }
       //  }
       //}

       //if (dot_right_top_l && is_line_left)//右入圆环  #状态机中使用
       //{
       //  printf("this is you ru yuan huan\n");
       //  line_right_top[0] = (float)(center_lost_corner_col_r - 0) / (float)(center_lost_corner_row_r - 79);   //k
       //  line_right_top[1] = (float)(0 - line_right_top[0] * 79);  //b
       //  //printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
       //  //显示该直线
       //  uint8_t m1, m2;
       //  if (79 > center_lost_corner_row_r)
       //  {
       //      m1 = center_lost_corner_row_r;
       //      m2 = 79;
       //  }
       //  else
       //  {
       //      m2 = center_lost_corner_row_r;
       //      m1 = 79;
       //  }
       //  //printf("  m1=%d,m2=%d\n", m1, m2);
       //  for (int32_t i = m1; i < m2; i++)
       //  {
       //      R_edge_use[i].row = i;
       //      R_edge_use[i].col = (int32_t)(i * line_right_top[0] + line_right_top[1]);
       //      //printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
       //      if (R_edge_use[i].col < 0)
       //      {
       //          R_edge_use[i].col = 0;
       //      }
       //      if (R_edge_use[i].col > 119)
       //      {
       //          R_edge_use[i].col = 119;
       //      }
       //  }
       //}
   }

   /* 计算中线 */
   for (int32_t i = 0; i < 80; i++)
   {
       Mid_Line[i].col = (L_edge_use[i].col + R_edge_use[i].col) / 2;
       Mid_Line[i].row = i;
   }

   /* 显示在屏幕中 */
   #if tjrc_display
       for(int32_t i=0;i<80;i++)
       {
           tjrc_st7735_drawPoint(4+L_edge_use[i].col,28+L_edge_use[i].row,0xF81F);
       }
       for(int32_t i=0;i<80;i++)
       {
           tjrc_st7735_drawPoint(4+R_edge_use[i].col,28+R_edge_use[i].row,0X5458);
       }
       for(int32_t i=0;i<80;i++)
       {
           tjrc_st7735_drawPoint(4+(L_edge_use[i].col+R_edge_use[i].col)/2,28+(L_edge_use[i].row+R_edge_use[i].row)/2,0x001F);
       }
   #endif
}


/**
 * @brief 横向搜索起始点
 * @param image     (uint8_t)输入图像数组
 * @param row       (uint8_t)起始横坐标
 * @param side      (uint8_t)左 or 右
 * @return col
 */
int tjrc_findstartpoint_row(uint8_t image[MT9V03X_H][MT9V03X_W],uint8_t row, uint8_t side)  //5和155需要调整
{
   //左边线
   if (side == 0)
   {
       uint8_t find_edge = 0;
       L_lost_count = 0;
       //从开始搜线行向上生长
       for (uint8_t rowi = row; rowi > L_edge_end_row; rowi--)
       {
           //如果图像最左侧为赛道外（未丢线），则开始横向生长
           if (L_lost_count <= L_lost_)
           {
               //横向生长
               for (int col = Image_W / 2; col > L_edge_start_col; col--)
               {
                   //printf("image[%d][%d],is black:%d\n", rowi, col, black_(image[rowi][col]));
                   if (black_(image[rowi][col]))
                   {
                       //printf("col:%d\n ", col);
                       //如果出现黑黑白白，则判断为边界线，退出循环
                       if (black_(image[rowi][col]) && black_(image[rowi][col - 1]) && white_(image[rowi][col + 2]) && white_(image[rowi][col + 1]))
                       {
                           L_basic_row_start = rowi;   //赋值开始搜线行（左）
                           #if tjrc_debug
                               printf("left start：(x=%d,y=%d)\r\n", rowi, col);
                           #endif
                           if (rowi<Image_H / 2) return -1;
                           return col ;               //返回列值         //!!!   1->2
                       }
                   }
                   if (col == L_edge_start_col + 1)
                   {
                       L_lost_count++;
                       //printf("L_lost_count:%d\n", L_lost_count);
                   }
               }
           }
           //printf("lost %d ", L_lost_count);
           if (L_lost_count > L_lost_)//丢线大于次数限制开启行搜索
           {
               break;
           }
       }
       if (L_lost_count > L_lost_)//丢线大于次数限制开启行搜索
       {
           for (uint8_t rowi = row; rowi > L_edge_end_row; rowi--)
           {
               //如果出现黑黑白白，则判断为边界线，退出循环
               if (black_(image[rowi - 3][L_edge_start_col]) && black_(image[rowi - 2][L_edge_start_col]) && white_(image[rowi - 1][L_edge_start_col]) && white_(image[rowi][L_edge_start_col]))
               {
                   L_basic_row_start = rowi - 2;   //赋值开始搜线行（左）
                   #if tjrc_debug
                       printf("left lost start：(x=%d,y=%d)\r\n", L_basic_row_start, L_edge_start_col);
                   #endif
                   L_start_lost = 1;
                   if (rowi<Image_H / 2) return -1;
                   return L_edge_start_col;               //返回列值         //!!!   1->2
               }
           }
       }
   }
   //右边线
   if (side == 1)
   {
       uint8_t find_edge = 0;
       R_lost_count = 0;
       for (uint8_t rowi = row; rowi > R_edge_end_row; rowi--)
       {
           //如果图像最右侧为赛道外（未丢线），则开始横向生长
           if (R_lost_count <= R_lost_)
           {
               //横向生长
               for (int col = Image_W / 2; col < R_edge_start_col; col++)
               {
                   if (black_(image[rowi][col]))
                   {
                       //如果出现黑黑白白，则判断为边界线，退出循环
                       if (white_(image[rowi][col - 2]) && white_(image[rowi][col - 1]) && black_(image[rowi][col + 1]) && black_(image[rowi][col]))
                       {
                           R_basic_row_start = rowi;
                           #if tjrc_debug
                               printf("right start：(x=%d,y=%d)\r\n", rowi, col);
                           #endif
                           if (rowi < Image_H / 2) return -1;
                           return col;
                       }
                   }
                   if (col == R_edge_start_col - 1)
                   {
                       R_lost_count++;
                   }
               }
           }

           if (R_lost_count > R_lost_)//丢线大于次数限制开启行搜索
           {
               break;
           }
           if (find_edge == 2) return -1;
       }
       if (R_lost_count > R_lost_)//丢线大于次数限制开启行搜索
       {
           for (uint8_t rowi = row; rowi > R_edge_end_row; rowi--)
           {
               if (black_(image[rowi - 3][R_edge_start_col]) && black_(image[rowi - 2][R_edge_start_col]) && white_(image[rowi - 1][R_edge_start_col]) && white_(image[rowi][R_edge_start_col]))
               {
                   R_basic_row_start = rowi - 2;   //赋值开始搜线行（左）
                   #if tjrc_debug
                       printf("right lost start：(x=%d,y=%d)\r\n", R_basic_row_start, R_edge_start_col);
                   #endif
                   R_start_lost = 1;
                   if (rowi < Image_H / 2) return -1;
                   return R_edge_start_col;               //返回列值         //!!!   1->2
               }
           }
       }
   }
   return -1;
}

/**
 * @brief 纵向搜索起始点
 * @param image     (uint8_t)输入图像数组
 * @param row       (uint8_t)起始横坐标
 * @param col       (uint8_t)起始纵坐标
 * @param side      (uint8_t)左 or 右
 * @return NONE
 */
void tjrc_findstartpoint_col(uint8_t image[MT9V03X_H][MT9V03X_W],uint8_t row, uint8_t col, uint8_t side)
{
   if (side == 0)
   {
       for (uint8_t rowi = row; rowi > L_edge_end_row; rowi--)
       {
           //printf("row=%d", rowi);
           if (black_(image[rowi - 3][col]) && black_(image[rowi - 2][col]) && white_(image[rowi - 1][col]) && white_(image[rowi][col]))
           {
               #if tjrc_debug
                   printf("center lost left start：(x=%d,y=%d)\r\n", rowi - 2, L_edge_lost_start_col);
               #endif
               center_lost_flag_l = 1;
               center_lost_row_l = rowi - 2;     //中间左丢线开始行坐标
               center_lost_col_l = col; //中间左丢线开始列坐标
               break;
           }
       }
   }
   if (side == 1)
   {
       for (uint8_t rowi = row; rowi > R_edge_end_row; rowi--)
       {
           //printf("row=%d", rowi);
           if (black_(image[rowi - 3][col]) && black_(image[rowi - 2][col]) && white_(image[rowi - 1][col]) && white_(image[rowi][col]))
           {
               #if tjrc_debug
                   printf("center lost left start：(x=%d,y=%d)\r\n", rowi - 2, R_edge_lost_start_col);
               #endif
               center_lost_flag_r = 1;
               center_lost_row_r = rowi - 2;     //中间左丢线开始行坐标
               center_lost_col_r = col; //中间左丢线开始列坐标
               break;
           }
       }
   }
}

/**
 * @brief 左八领域搜线
 * @param image          (uint8_t)输入图像数组
 * @param jilu_row       (uint8_t)起始横坐标
 * @param jilu_col       (uint8_t)起始纵坐标
 * @param search_amount  (uint8_t)搜索点数
 * @param start          (uint8_t)搜线数组起始下标
 * @return NONE
 */
void tjrc_findline_lefthand(uint8_t image[MT9V03X_H][MT9V03X_W],int jilu_row, int jilu_col, int search_amount,int start)
{
   L_edge[0].row = jilu_row;
   L_edge[0].col = jilu_col;
   L_edge[0].flag = 1;
   uint8_t curr_row = jilu_row;//初始化行坐标
   uint8_t curr_col = jilu_col;//初始化列坐标
   dire_left = 0; //初始化上个边界点的来向
   center_turn_flag = 1;//初始化在未标定状态
   center_lost_flag_l = 0;//中间左丢线标志位
   uint8_t turn = 0;
   //开始搜线，最多取150个点，不会往下搜，共7个方位
   for (int i = start; i < start + search_amount - 1; i++)   //最多搜索70个点
   {
       ////越界退出 行越界和列越界（向上向下向左向右）
       if (curr_row < L_edge_end_row || curr_row > Image_H - 1 || curr_row + 1 < L_edge_end_row)  break;
       if (curr_col > max_col || curr_col < min_col && !center_lost_flag_l)   //max=160-5 min=5
       {
           break;
       }
       if (turn == 1)break;
       //搜线过程
       if (black_(image[curr_row + dir_front[dire_left][0]][curr_col + dir_front[dire_left][1]]))
       {
           dire_left = (dire_left + 1) % 4;
           i = i - 1;
           if (black_(image[curr_row - 1][curr_col + 0]) && black_(image[curr_row + 0][curr_col + 1]) && black_(image[curr_row + 1][curr_col + 0]) && black_(image[curr_row + 0][curr_col - 1]))break;
       }
       else if (black_(image[curr_row + dir_frontleft[dire_left][0]][curr_col + dir_frontleft[dire_left][1]]))
       {
           curr_row += dir_front[dire_left][0];
           curr_col += dir_front[dire_left][1];
           L_edge[i].row = curr_row;
           L_edge[i].col = curr_col;
           L_edge[i].flag = 1;
           L_edge_count = L_edge_count + 1;
       }
       else
       {
           curr_row += dir_frontleft[dire_left][0];
           curr_col += dir_frontleft[dire_left][1];
           dire_left = (dire_left + 3) % 4;
           L_edge[i].row = curr_row;
           L_edge[i].col = curr_col;
           L_edge[i].flag = 1;
           L_edge_count = L_edge_count + 1;
       }
//       rt_kprintf("left:(x1=%d,y1=%d,i=%d,start=%d,start+search_amount-1=%d)\r\n", L_edge[i].row, L_edge[i].col,i,start,start+search_amount-1);
   }
   //printf("count%d ", L_edge_count);
}

/**
 * @brief 右八领域搜线
 * @param image          (uint8_t)输入图像数组
 * @param jilu_row       (uint8_t)起始横坐标
 * @param jilu_col       (uint8_t)起始纵坐标
 * @param search_amount  (uint8_t)搜索点数
 * @param start          (uint8_t)搜线数组起始下标
 * @return NONE
 */
void tjrc_findline_righthand(uint8_t image[MT9V03X_H][MT9V03X_W],int jilu_row, int jilu_col, int search_amount, int start)
{
   R_edge[0].row = jilu_row;
   R_edge[0].col = jilu_col;
   R_edge[0].flag = 1;
   uint8_t curr_row = jilu_row;//初始化行坐标
   uint8_t curr_col = jilu_col;//初始化列坐标
   dire_right = 0; //初始化上个边界点的来向
   center_turn_flag = 1;//初始化在未标定状态
   center_lost_flag_r = 0;//中间左丢线标志位
   uint8_t turn = 0;
   //开始搜线，最多取150个点，不会往下搜，共7个方位
   //printf("start=%d,search_amount=%d\n",start, search_amount);
   for (int i = start; i < start + search_amount - 1; i++)  //最多搜索70个点
   {
       ////越界退出 行越界和列越界（向上向下向左向右）
       if (curr_row < R_edge_end_row || curr_row>Image_H - 1 || curr_row + 1 < R_edge_end_row)  break;
       //printf("(curr_col=%d,max_col=%d)\n", curr_col, max_col);
       if (curr_col > max_col || curr_col < min_col && !center_lost_flag_r)   //max=160-5 min=5
       {
           break;
       }
       if (turn == 1)break;
       //搜线过程
       if (black_(image[curr_row + dir_front[dire_right][0]][curr_col + dir_front[dire_right][1]]))
       {
           dire_right = (dire_right + 3) % 4;
           i = i - 1;
           if (black_(image[curr_row - 1][curr_col + 0]) && black_(image[curr_row + 0][curr_col + 1]) && black_(image[curr_row + 1][curr_col + 0]) && black_(image[curr_row + 0][curr_col - 1]))break;
       }
       else if (black_(image[curr_row + dir_frontright[dire_right][0]][curr_col + dir_frontright[dire_right][1]]))
       {
           curr_row += dir_front[dire_right][0];
           curr_col += dir_front[dire_right][1];
           R_edge[i].row = curr_row;
           R_edge[i].col = curr_col;
           R_edge[i].flag = 1;
           R_edge_count = R_edge_count + 1;
       }
       else
       {
           curr_row += dir_frontright[dire_right][0];
           curr_col += dir_frontright[dire_right][1];
           dire_right = (dire_right + 1) % 4;
           R_edge[i].row = curr_row;
           R_edge[i].col = curr_col;
           R_edge[i].flag = 1;
           R_edge_count = R_edge_count + 1;
       }
//       rt_kprintf("right:(x1=%d,y1=%d,i=%d,start=%d,start+search_amount-1=%d)\r\n", R_edge[i].row, R_edge[i].col,i,start,start+search_amount-1);
       //printf("(x1=%d,y1=%d,i=%d, start + search_amount - 1=%d)\n", R_edge[i].row, R_edge[i].col,i, start + search_amount - 1);
   }
   //printf("count%d ", L_edge_count);
}

/**
 * @brief 左下拐点搜索
 * @param start          (uint8_t)搜线数组起始下标
 * @param end            (uint8_t)搜线数组结束下标
 * @param search_amount  (uint8_t)搜索点数
 * @return NONE
 */
void tjrc_findturnpoint_leftbottom(int start,int end,int search_amount)
{
   uint8_t m3 = 0;
   if (end - start > search_amount)end = start + search_amount;
   //直线顶点坐标(L_edge[0].col,L_edge[0].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
//   printf("left line bottom point:(%d,%d),(%d,%d)\r\n", L_edge[start].row, L_edge[start].col, L_edge[end].row, L_edge[end].col);
   if((L_edge[end].row - L_edge[start].row)!=0)
   {
       line_left_bottom[0] = (float)(L_edge[end].col - L_edge[start].col) / (float)(L_edge[end].row - L_edge[start].row);   //k
       line_left_bottom[1] = (float)(L_edge[start].col - line_left_bottom[0] * L_edge[start].row);  //b
   }

//   printf("line:k=%f,b=%f\r\n", line_left_bottom[0], line_left_bottom[1]);                                                                                          //显示该直线
   uint8_t m1, m2;
   if (L_edge[start].row > L_edge[end].row)
   {
       m1 = L_edge[end].row;
       m2 = L_edge[start].row;
   }
   else
   {
       m2 = L_edge[end].row;
       m1 = L_edge[start].row;
   }
   for (int i = m1; i < m2; i++)
   {
       L_edge_use[i].row = i;
       L_edge_use[i].col = (int)(i * line_left_bottom[0] + line_left_bottom[1]);
       //printf("hh(%d,%d)\n", L_edge_use[i].col, L_edge_use[i].row);
       if (L_edge_use[i].col < 0)
       {
           L_edge_use[i].col = 0;
       }
       if (L_edge_use[i].col > 119)
       {
           L_edge_use[i].col = 119;
       }
   }
   float max = 0;
   float distance = 0;
   dist_sum_left_bottom = 0;
   dot_left_bottom_l = 0;
   dot_left_bottom_r = 0;
   is_line_left_bottom = 0;
   for (int i = start; i < end; i++)
   {
       distance = (float)(line_left_bottom[0] * L_edge[i].row - L_edge[i].col + line_left_bottom[1]) / sqrt(line_left_bottom[0] * line_left_bottom[0] + 1);
       //printf("%f\n", distance);
       if (distance < max)
       {
           max = distance;
           L_corner_row = L_edge[i].row;
           L_corner_col = L_edge[i].col;
           L_beh_i = i;
       }
       //printf("%f\n", distance);
       dist_sum_left_bottom += distance;//求总体差值判断是否为直线
   }
   dist_sum_left_bottom = dist_sum_left_bottom * search_amount / (end - start);
   L_corner_angle = max;
   if (dist_sum_left_bottom < 0)dot_left_bottom_l = 1;//左拐线
   else dot_left_bottom_r = 1;//右拐线
   if ((dist_sum_left_bottom >= 0 ? dist_sum_left_bottom : -dist_sum_left_bottom) < dist_control && (dist_sum_left_bottom >= 0 ? dist_sum_left_bottom : -dist_sum_left_bottom)!=0)
   {
       is_line_left_bottom = 1;
       dot_left_bottom_l = 0;
       dot_left_bottom_r = 0;
   }

   #if tjrc_debug
       printf("distance bottom left:%f\r\n", dist_sum_left_bottom);
   #endif
}

/**
 * @brief 左上拐点搜索
 * @param start          (uint8_t)搜线数组起始下标
 * @param end            (uint8_t)搜线数组结束下标
 * @param search_amount  (uint8_t)搜索点数
 * @return NONE
 */
void tjrc_findturnpoint_lefttop(int start, int end,int search_amount)
{
   uint8_t m3 = 0;
   if (end - start > search_amount)end = start + search_amount;
   //直线顶点坐标(L_edge[1].col,L_edge[1].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
//   printf("left line top point:(%d,%d),(%d,%d)\r\n", L_edge[start].row, L_edge[start].col, L_edge[end].row, L_edge[end].col);
   if((L_edge[end].row - L_edge[start].row)!=0)
   {
       line_left_top[0] = (float)(L_edge[end].col - L_edge[start].col) / (float)(L_edge[end].row - L_edge[start].row);   //k
       line_left_top[1] = (float)(L_edge[start].col - line_left_top[0] * L_edge[start].row);  //b
   }

//   printf("line:k=%f,b=%f\r\n", line_left_top[0], line_left_top[1]);
   //显示该直线
   uint8_t m1, m2;
   if (L_edge[start].row > L_edge[end].row)
   {
       m1 = L_edge[end].row;
       m2 = L_edge[start].row;
   }
   else
   {
       m2 = L_edge[end].row;
       m1 = L_edge[start].row;
   }
   //printf("  m1=%d,m2=%d\n", m1, m2);
   for (int i = m1; i < m2; i++)
   {
       L_edge_use[i].row = i;
       L_edge_use[i].col = (int)(i * line_left_top[0] + line_left_top[1]);
       //printf("hh(%d,%d)\n", L_edge_use[i].col, L_edge_use[i].row);
       if (L_edge_use[i].col < 0)
       {
           L_edge_use[i].col = 0;
       }
       if (L_edge_use[i].col > 119)
       {
           L_edge_use[i].col = 119;
       }
   }
   float max = 0;
   float distance = 0;
   dist_sum_left_top = 0;
   dot_left_top_l = 0;
   dot_left_top_r = 0;
   is_line_left_top = 0;
   for (int i = start; i < end; i++)
   {
       distance = (float)(line_left_top[0] * L_edge[i].row - L_edge[i].col + line_left_top[1]) / sqrt(line_left_top[0] * line_left_top[0] + 1);
       if (distance < max)
       {
           max = distance;
           center_lost_corner_row_l = L_edge[i].row;
           center_lost_corner_col_l = L_edge[i].col;
           L_top_i = i;
       }
       dist_sum_left_top += distance;//求总体差值判断是否为直线
   }
   dist_sum_left_top = dist_sum_left_top * search_amount / (end - start);
   L_center_lost_corner_angle = max;
   if (dist_sum_left_top < 0)dot_left_top_l = 1;//左拐线
   else dot_left_top_r = 1;//右拐线
   if ((dist_sum_left_top >= 0 ? dist_sum_left_top : -dist_sum_left_top) < dist_control && (dist_sum_left_top >= 0 ? dist_sum_left_top : -dist_sum_left_top)!=0)
   {
       is_line_left_top = 1;
       dot_left_top_l = 0;
       dot_left_top_r = 0;
   }
   #if tjrc_debug
       printf("distance top left:%f\n", dist_sum_left_top);
   #endif
}

/**
 * @brief 右下拐点搜索
 * @param start          (uint8_t)搜线数组起始下标
 * @param end            (uint8_t)搜线数组结束下标
 * @param search_amount  (uint8_t)搜索点数
 * @return NONE
 */
void tjrc_findturnpoint_rightbottom(int start, int end,int search_amount)
{
   uint8_t m3 = 0;
   if (end - start > search_amount)end = start + search_amount;
   //直线顶点坐标(L_edge[0].col,L_edge[0].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)

   if((R_edge[end].row - R_edge[start].row)!=0)
   {
       line_right_bottom[0] = (float)(R_edge[end].col - R_edge[start].col) / (float)(R_edge[end].row - R_edge[start].row);   //k
       line_right_bottom[1] = (float)(R_edge[start].col - line_right_bottom[0] * R_edge[start].row);  //b
   }
   #if tjrc_debug
       printf("right line bottom point:(%d,%d),(%d,%d)\r\n", R_edge[start].row, R_edge[start].col, R_edge[end].row, R_edge[end].col);
       printf("line:k=%f,b=%f\r\n", line_right_bottom[0], line_right_bottom[1]);                                                                                            //显示该直线
   #endif
   uint8_t m1, m2;
   if (R_edge[start].row > R_edge[end].row)
   {
       m1 = R_edge[end].row;
       m2 = R_edge[start].row;
   }
   else
   {
       m2 = R_edge[end].row;
       m1 = R_edge[start].row;
   }
   for (int i = m1; i < m2; i++)
   {
       R_edge_use[i].row = i;
       R_edge_use[i].col = (int)(i * line_right_bottom[0] + line_right_bottom[1]);
       //printf("hh(%d,%d)\n", L_edge_use[i].col, L_edge_use[i].row);
       if (R_edge_use[i].col < 0)
       {
           R_edge_use[i].col = 0;
       }
       if (R_edge_use[i].col > 119)
       {
           R_edge_use[i].col = 119;
       }
   }
   float max = 0;
   float distance = 0;
   dist_sum_right_bottom = 0;
   dot_right_bottom_l = 0;
   dot_right_bottom_r = 0;
   is_line_right_bottom = 0;
   for (int i = start; i < end; i++)
   {
       distance = (float)(line_right_bottom[0] * R_edge[i].row - R_edge[i].col + line_right_bottom[1]) / sqrt(line_right_bottom[0] * line_right_bottom[0] + 1);
       //printf("%f\n", distance);
       if (distance > max)
       {
           max = distance;
           R_corner_row = R_edge[i].row;
           R_corner_col = R_edge[i].col;
           R_beh_i = i;
       }
       //printf("%f\n", distance);
       dist_sum_right_bottom += distance;//求总体差值判断是否为直线
   }
   dist_sum_right_bottom = dist_sum_right_bottom * search_amount / (end - start);
   R_corner_angle = max;
   if (dist_sum_right_bottom < 0)dot_right_bottom_l = 1;//左拐线
   else dot_right_bottom_r = 1;//右拐线
   if ((dist_sum_right_bottom >= 0 ? dist_sum_right_bottom : -dist_sum_right_bottom) < dist_control && (dist_sum_right_bottom >= 0 ? dist_sum_right_bottom : -dist_sum_right_bottom)!=0)
   {
       is_line_right_bottom = 1;
       dot_right_bottom_l = 0;
       dot_right_bottom_r = 0;
   }

   #if tjrc_debug
       printf("distance bottom left:%f\r\n", dist_sum_right_bottom);
   #endif
}

/**
 * @brief 右上拐点搜索
 * @param start          (uint8_t)搜线数组起始下标
 * @param end            (uint8_t)搜线数组结束下标
 * @param search_amount  (uint8_t)搜索点数
 * @return NONE
 */
void tjrc_findturnpoint_righttop(int start, int end,int search_amount)
{
   uint8_t m3 = 0;
   if (end - start > search_amount)end = start + search_amount;
   //直线顶点坐标(L_edge[1].col,L_edge[1].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)

   if((R_edge[end].row - R_edge[start].row)!=0)
   {
       line_right_top[0] = (float)(R_edge[end].col - R_edge[start].col) / (float)(R_edge[end].row - R_edge[start].row);   //k
       line_right_top[1] = (float)(R_edge[start].col - line_right_top[0] * R_edge[start].row);  //b
   }
   #if tjrc_debug
       printf("right line top point:(%d,%d),(%d,%d)\r\n", R_edge[start].row, R_edge[start].col, R_edge[end].row, R_edge[end].col);
       printf("line:k=%f,b=%f\r\n", line_right_top[0], line_right_top[1]);
   #endif
   //显示该直线
   uint8_t m1, m2;
   if (R_edge[start].row > R_edge[end].row)
   {
       m1 = R_edge[end].row;
       m2 = R_edge[start].row;
   }
   else
   {
       m2 = R_edge[end].row;
       m1 = R_edge[start].row;
   }
   //printf("  m1=%d,m2=%d\n", m1, m2);
   for (int i = m1; i < m2; i++)
   {
       R_edge_use[i].row = i;
       R_edge_use[i].col = (int)(i * line_right_top[0] + line_right_top[1]);
       //printf("hh(%d,%d)\n", L_edge_use[i].col, L_edge_use[i].row);
       if (R_edge_use[i].col < 0)
       {
           R_edge_use[i].col = 0;
       }
       if (R_edge_use[i].col > 119)
       {
           R_edge_use[i].col = 119;
       }
   }
   float max = 0;
   float distance = 0;
   dist_sum_right_top = 0;
   dot_right_top_l = 0;
   dot_right_top_r = 0;
   is_line_right_top = 0;
   for (int i = start; i < end; i++)
   {
       distance = (float)(line_right_top[0] * R_edge[i].row - R_edge[i].col + line_right_top[1]) / sqrt(line_right_top[0] * line_right_top[0] + 1);
       if (distance > max)
       {
           max = distance;
           center_lost_corner_row_r = R_edge[i].row;
           center_lost_corner_col_r = R_edge[i].col;
           R_top_i = i;
       }
       dist_sum_right_top += distance;//求总体差值判断是否为直线
   }
   dist_sum_right_top = dist_sum_right_top * search_amount / (end - start);
   R_center_lost_corner_angle = max;
   if (dist_sum_right_top < 0)dot_right_top_l = 1;//左拐线
   else dot_right_top_r = 1;//右拐线

   if ((dist_sum_right_top >= 0 ? dist_sum_right_top : -dist_sum_right_top) < dist_control && (dist_sum_right_top >= 0 ? dist_sum_right_top : -dist_sum_right_top)!=0)
   {
       is_line_right_top = 1;
       dot_right_top_l = 0;
       dot_right_top_r = 0;
   }

   #if tjrc_debug
       printf("distance top left:%f\n", dist_sum_right_top);
   #endif
}

/**
 * @brief 正十字补线
 * @param L_corner_row              (uint8_t)左下拐点行坐标
 * @param L_corner_col              (uint8_t)左下拐点列坐标
 * @param center_lost_corner_row_l  (uint8_t)左上拐点行坐标
 * @param center_lost_corner_col_l  (uint8_t)左上拐点列坐标
 * @param R_corner_row              (uint8_t)右下拐点行坐标
 * @param R_corner_col              (uint8_t)右下拐点列坐标
 * @param center_lost_corner_row_r  (uint8_t)右上拐点行坐标
 * @param center_lost_corner_col_r  (uint8_t)右上拐点列坐标
 * @return NONE
 */
void tjrc_greekcross_patchLine(uint8_t L_corner_row, uint8_t L_corner_col, uint8_t center_lost_corner_col_l, uint8_t center_lost_corner_row_l, uint8_t R_corner_row, uint8_t R_corner_col, uint8_t center_lost_corner_col_r, uint8_t center_lost_corner_row_r)
{
    if ((center_lost_corner_row_l - L_corner_row) != 0)
    {
        line_left_top[0] = (float)(center_lost_corner_col_l - L_corner_col) / (float)(center_lost_corner_row_l - L_corner_row);   //k
        line_left_top[1] = (float)(L_corner_col - line_left_top[0] * L_corner_row);  //b
    }

    //显示该直线
    uint8_t m1, m2;
    if (L_corner_row > center_lost_corner_row_l)
    {
        m1 = center_lost_corner_row_l;
        m2 = L_corner_row;
    }
    else
    {
        m2 = center_lost_corner_row_l;
        m1 = L_corner_row;
    }
    //printf("  m1=%d,m2=%d\n", m1, m2);
    for (int i = m1; i < m2; i++)
    {
        L_edge_use[i].row = i;
        L_edge_use[i].col = (int)(i * line_left_top[0] + line_left_top[1]);
        //printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
        if (L_edge_use[i].col < 0)
        {
            L_edge_use[i].col = 0;
        }
        if (L_edge_use[i].col > 119)
        {
            L_edge_use[i].col = 119;
        }
    }
    for (int i = L_beh_i; i > 0; i--)
    {
        L_edge_use[m2].row = L_edge[i].row;
        L_edge_use[m2].col = L_edge[i].col;
        m2++;
    }
    for (int i = L_top_i; i < L_edge_count; i++)
    {
        L_edge_use[m1].row = L_edge[i].row;
        L_edge_use[m1].col = L_edge[i].col;
        m1--;
    }

    line_right_top[0] = (float)(center_lost_corner_col_r - R_corner_col) / (float)(center_lost_corner_row_r - R_corner_row);   //k
    line_right_top[1] = (float)(R_corner_col - line_right_top[0] * R_corner_row);  //b
    //printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
    //显示该直线
    if (R_corner_row > center_lost_corner_row_r)
    {
        m1 = center_lost_corner_row_r;
        m2 = R_corner_row;
    }
    else
    {
        m2 = center_lost_corner_row_r;
        m1 = R_corner_row;
    }
    //printf("  m1=%d,m2=%d\n", m1, m2);
    for (int i = m1; i < m2; i++)
    {
        R_edge_use[i].row = i;
        R_edge_use[i].col = (int)(i * line_right_top[0] + line_right_top[1]);
        //printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
        if (R_edge_use[i].col < 0)
        {
            R_edge_use[i].col = 0;
        }
        if (R_edge_use[i].col > 119)
        {
            R_edge_use[i].col = 119;
        }
    }
    for (int i = R_beh_i; i > 0; i--)
    {
        R_edge_use[m2].row = R_edge[i].row;
        R_edge_use[m2].col = R_edge[i].col;
        m2++;
    }
    for (int i = R_top_i; i < R_edge_count; i++)
    {
        R_edge_use[m1].row = R_edge[i].row;
        R_edge_use[m1].col = R_edge[i].col;
        m1--;
    }
}

/**
 * @brief 中十字补线
 * @param L_top_i       (uint8_t)左上拐点开始下标
 * @param R_top_i       (uint8_t)右上拐点开始下标
 * @return NONE
 */
void tjrc_centercross_patchLine(uint8_t L_top_i,uint8_t R_top_i)
{
    FitStraightLine(L_top_i, L_top_i + 10, 0);
    //printf("k_l:%f,b_l:%f\n", k_l, b_l);
    for (int i = L_edge[L_top_i].row; i < 80; i++)
    {
        L_edge_use[i].row = i;
        L_edge_use[i].col = (int)(i * k_l + b_l);
        //printf("hh(%d,%d)\n", L_edge_use[i].row, L_edge_use[i].col);
        if (L_edge_use[i].col < 0)
        {
            L_edge_use[i].col = 0;
        }
        if (L_edge_use[i].col > 119)
        {
            L_edge_use[i].col = 119;
        }
    }
    uint8_t m1 = L_edge[L_top_i].row;
    for (int i = L_top_i; i < L_edge_count; i++)
    {
        L_edge_use[m1].row = L_edge[i].row;
        L_edge_use[m1].col = L_edge[i].col;
        m1--;
    }
    FitStraightLine(R_top_i, R_top_i + 10, 1);
    //printf("k_r:%f,b_r:%f\n", k_r, b_r);
    for (int i = R_edge[R_top_i].row; i < 80; i++)
    {
        R_edge_use[i].row = i;
        R_edge_use[i].col = (int)(i * k_r + b_r);
        if (R_edge_use[i].col > 119)
        {
            R_edge_use[i].col = 119;
        }
        if (R_edge_use[i].col < 0)
        {
            R_edge_use[i].col = 0;
        }
    }
    uint8_t m2 = R_edge[R_top_i].row;
    for (int i = R_top_i; i < R_edge_count; i++)
    {
        R_edge_use[m2].row = R_edge[i].row;
        R_edge_use[m2].col = R_edge[i].col;
        m2--;
    }
}

/**
 * @brief 左圆环补线
 * @param L_corner_row              (uint8_t)左下拐点行坐标
 * @param L_corner_col              (uint8_t)左下拐点列坐标
 * @param center_lost_corner_row_l  (uint8_t)左上拐点行坐标
 * @param center_lost_corner_col_l  (uint8_t)左上拐点列坐标
 * @return NONE
 */
void tjrc_leftring_patchLine(uint8_t L_corner_row, uint8_t L_corner_col, uint8_t center_lost_corner_col_l, uint8_t center_lost_corner_row_l)
{
    line_left_top[0] = (float)(center_lost_corner_col_l - L_corner_col) / (float)(center_lost_corner_row_l - L_corner_row);   //k
    line_left_top[1] = (float)(L_corner_col - line_left_top[0] * L_corner_row);  //b
    //printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
    //显示该直线
    uint8 m1, m2;
    if (L_corner_row > center_lost_corner_row_l)
    {
        m1 = center_lost_corner_row_l;
        m2 = L_corner_row;
    }
    else
    {
        m2 = center_lost_corner_row_l;
        m1 = L_corner_row;
    }
    //printf("  m1=%d,m2=%d\n", m1, m2);
    for (int i = m1; i < m2; i++)
    {
        L_edge_use[i].row = i;
        L_edge_use[i].col = (int)(i * line_left_top[0] + line_left_top[1]);
        //printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
        if (L_edge_use[i].col < 0)
        {
            L_edge_use[i].col = 0;
        }
        if (L_edge_use[i].col > 119)
        {
            L_edge_use[i].col = 119;
        }
    }
    for (int i = L_beh_i; i > 0; i--)
    {
        L_edge_use[m2].row = L_edge[i].row;
        L_edge_use[m2].col = L_edge[i].col;
        m2++;
    }
    for (int i = L_top_i; i < L_edge_count; i++)
    {
        L_edge_use[m1].row = L_edge[i].row;
        L_edge_use[m1].col = L_edge[i].col;
        m1--;
    }
    m1 = 79;
    for (int i = 0; i < 75; i++)
    {
        R_edge_use[m1].row = R_edge[i].row;
        R_edge_use[m1].col = R_edge[i].col;
        m1--;
    }
}

/**
 * @brief 右圆环补线
 * @param R_corner_row              (uint8_t)右下拐点行坐标
 * @param R_corner_col              (uint8_t)右下拐点列坐标
 * @param center_lost_corner_row_r  (uint8_t)右上拐点行坐标
 * @param center_lost_corner_col_r  (uint8_t)右上拐点列坐标
 * @return NONE
 */
void tjrc_rightring_patchLine(uint8_t R_corner_row, uint8_t R_corner_col, uint8_t center_lost_corner_col_r, uint8_t center_lost_corner_row_r)
{
    line_right_top[0] = (float)(center_lost_corner_col_r - R_corner_col) / (float)(center_lost_corner_row_r - R_corner_row);   //k
    line_right_top[1] = (float)(R_corner_col - line_right_top[0] * R_corner_row);  //b
    //printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
    //显示该直线
    uint8 m1, m2;
    if (R_corner_row > center_lost_corner_row_r)
    {
        m1 = center_lost_corner_row_r;
        m2 = R_corner_row;
    }
    else
    {
        m2 = center_lost_corner_row_r;
        m1 = R_corner_row;
    }
    //printf("  m1=%d,m2=%d\n", m1, m2);
    for (int i = m1; i < m2; i++)
    {
        R_edge_use[i].row = i;
        R_edge_use[i].col = (int)(i * line_right_top[0] + line_right_top[1]);
        //printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
        if (R_edge_use[i].col < 0)
        {
            R_edge_use[i].col = 0;
        }
        if (R_edge_use[i].col > 119)
        {
            R_edge_use[i].col = 119;
        }
    }
    for (int i = R_beh_i; i > 0; i--)
    {
        R_edge_use[m2].row = R_edge[i].row;
        R_edge_use[m2].col = R_edge[i].col;
        m2++;
    }
    for (int i = R_top_i; i < R_edge_count; i++)
    {
        R_edge_use[m1].row = R_edge[i].row;
        R_edge_use[m1].col = R_edge[i].col;
        m1--;
    }
    m1 = 79;
    for (int i = 0; i < 75; i++)
    {
        L_edge_use[m1].row = L_edge[i].row;
        L_edge_use[m1].col = L_edge[i].col;
        m1--;
    }
}

/**
 * @brief 左弯补线
 * @param L_top_corner_start        (uint8_t)左上拐点开始下标
 * @return NONE
 */
void tjrc_leftturn_patchLine(uint8_t L_top_corner_start)
{
    if (!L_start_lost)
    {
        int m1 = L_edge[1].row;
        for (int i = 1; i < L_top_corner_start - 1; i++)
        {
            L_edge_use[m1].row = L_edge[i].row;
            L_edge_use[m1].col = L_edge[i].col;
            //printf("(x=%d,y=%d,i=%d)\n", L_edge_use[m1].row, L_edge_use[m1].col, i);
            m1--;
        }
        //printf("\n");
        for (int i = m1; i > 3; i--)
        {
            L_edge_use[i].row = i;
            L_edge_use[i].col = 0;
            //printf("(x=%d,y=%d,i=%d)\n", L_edge_use[i].row, L_edge_use[i].col, i);
        }
        //printf("\n");
        int m2 = R_edge[1].row;
        //printf("( R_edge[1].row=%d)\n", R_edge[1].row);
        for (int i = 1; i < 70; i++)
        {
            R_edge_use[m2].row = R_edge[i].row;
            R_edge_use[m2].col = R_edge[i].col;
            //printf("(x=%d,y=%d,i=%d)\n", R_edge_use[m2].row, R_edge_use[m2].col, m2);
            m2--;
        }
    }
    else
    {
        for (int i = 79; i > 7; i--)
        {
            L_edge_use[i].row = i;
            L_edge_use[i].col = 0;
            //printf("(x=%d,y=%d,i=%d)\n", L_edge_use[i].row, L_edge_use[i].col, i);
        }
        //printf("\n");
        int m2 = R_edge[1].row;
        m2 = 0;

        for (int i = R_basic_row_start; i > R_edge[R_edge_count - 1].row; i--)
        {
            //printf("hh(L1=%d,L2=%d,m2=%d,i=%d,x=%d,y=%d)\n", R_edge[m2].row, R_edge[m2 + 1].row, m2, i, R_edge_use[i].row, R_edge_use[i].col);
            if (R_edge[m2].row == R_edge[m2 + 1].row)
            {
                m2++;
                i++;
            }
            else
            {
                R_edge_use[i].row = R_edge[m2].row;
                R_edge_use[i].col = R_edge[m2].col;
                m2++;
            }
            //m2--;
        }
        uint8_t temp = R_edge_use[R_edge[R_edge_count - 1].row].col;
        for (int i = R_edge[R_edge_count - 1].row; i > 0; i--)
        {
            R_edge_use[i].row = i;
            R_edge_use[i].col = temp;
        }
    }
}

/**
 * @brief 右弯补线
 * @param R_top_corner_start        (uint8_t)右上拐点开始下标
 * @return NONE
 */
void tjrc_rightturn_patchLine(uint8_t R_top_corner_start)
{
    if (!R_start_lost)
    {
        int m1 = R_edge[1].row;
        for (int i = 1; i < R_top_corner_start - 1; i++)
        {
            R_edge_use[m1].row = R_edge[i].row;
            R_edge_use[m1].col = R_edge[i].col;
            //printf("(x=%d,y=%d,i=%d)\n", L_edge_use[m1].row, L_edge_use[m1].col, i);
            m1--;
        }
        //printf("\n");
        for (int i = m1; i > 3; i--)
        {
            R_edge_use[i].row = i;
            R_edge_use[i].col = 119;
            //printf("(x=%d,y=%d,i=%d)\n", L_edge_use[i].row, L_edge_use[i].col, i);
        }
        //printf("\n");
        int m2 = L_edge[1].row;
        //printf("( R_edge[1].row=%d)\n", R_edge[1].row);
        for (int i = 1; i < 70; i++)
        {
            L_edge_use[m2].row = L_edge[i].row;
            L_edge_use[m2].col = L_edge[i].col;
            //printf("(x=%d,y=%d,i=%d)\n", R_edge_use[m2].row, R_edge_use[m2].col, m2);
            m2--;
        }
    }
    else
    {
        for (int i = 79; i > 7; i--)
        {
            R_edge_use[i].row = i;
            R_edge_use[i].col = 119;
            //printf("(x=%d,y=%d,i=%d)\n", L_edge_use[i].row, L_edge_use[i].col, i);
        }
        //printf("\n");
        int m2 = L_edge[1].row;
        uint8_t a = 0;
        m2 = 0;
        //边线截断
        for (int i = L_basic_row_start; i > L_edge[L_edge_count-1].row ; i--)
        {
            //printf("hh(L1=%d,L2=%d,m2=%d,i=%d,x=%d,y=%d)\n", L_edge[m2].row, L_edge[m2 + 1].row, m2,i, L_edge_use[i].row, L_edge_use[i].col);
            if (L_edge[m2].row == L_edge[m2 + 1].row)
            {
                m2++;
                i++;
            }
            else
            {
                L_edge_use[i].row = L_edge[m2].row;
                L_edge_use[i].col = L_edge[m2].col;
                m2++;
            }
            //m2--;
        }
        uint8_t temp = L_edge_use[L_edge[L_edge_count - 1].row].col;
        for (int i = L_edge[L_edge_count-1].row; i >0; i--)
        {
            L_edge_use[i].row = i;
            L_edge_use[i].col = temp;
        }
    }
}

/**
 * @brief 左斜入十字补线
 * @param L_top_i       (uint8_t)左上拐点开始下标
 * @param R_top_i       (uint8_t)右上拐点开始下标
 * @return NONE
 */
void tjrc_oblique_leftcross_patchLine(uint8_t L_top_i, uint8_t R_top_i)
{
    uint8_t m1, m2;
    FitStraightLine(L_top_i, L_top_i + 5, 0);
    //printf("k_l:%f,b_l:%f\n", k_l, b_l);
    for (int i = L_edge[L_top_i].row; i < 80; i++)
    {
        L_edge_use[i].row = i;
        L_edge_use[i].col = (int)(i * k_l + b_l);
        //printf("hh(%d,%d)\n", L_edge_use[i].row, L_edge_use[i].col);
        if (L_edge_use[i].col < 0)
        {
            L_edge_use[i].col = 0;
        }
        if (L_edge_use[i].col > 119)
        {
            L_edge_use[i].col = 119;
        }
    }

    m1 = L_edge[L_top_i].row;
    for (int i = L_top_i; i < L_edge_count; i++)
    {
        L_edge_use[m1].row = L_edge[i].row;
        L_edge_use[m1].col = L_edge[i].col;
        m1--;
    }
    //显示该直线

    if ((center_lost_corner_row_r - R_corner_row) != 0)
    {
        line_right_top[0] = (float)(center_lost_corner_col_r - R_corner_col) / (float)(center_lost_corner_row_r - R_corner_row);   //k
        line_right_top[1] = (float)(R_corner_col - line_right_top[0] * R_corner_row);  //b
    }
    //printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
    //显示该直线
    if (R_corner_row > center_lost_corner_row_r)
    {
        m1 = center_lost_corner_row_r;
        m2 = R_corner_row;
    }
    else
    {
        m2 = center_lost_corner_row_r;
        m1 = R_corner_row;
    }
    //printf("  m1=%d,m2=%d\n", m1, m2);
    for (int i = m1; i < m2; i++)
    {
        R_edge_use[i].row = i;
        R_edge_use[i].col = (int)(i * line_right_top[0] + line_right_top[1]);
        //printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
        if (R_edge_use[i].col < 0)
        {
            R_edge_use[i].col = 0;
        }
        if (R_edge_use[i].col > 119)
        {
            R_edge_use[i].col = 119;
        }
    }
    for (int i = R_beh_i; i > 0; i--)
    {
        R_edge_use[m2].row = R_edge[i].row;
        R_edge_use[m2].col = R_edge[i].col;
        m2++;
    }
    for (int i = R_top_i; i < R_edge_count; i++)
    {
        R_edge_use[m1].row = R_edge[i].row;
        R_edge_use[m1].col = R_edge[i].col;
        m1--;
    }
}

/**
 * @brief 右斜入十字补线
 * @param L_top_i       (uint8_t)左上拐点开始下标
 * @param R_top_i       (uint8_t)右上拐点开始下标
 * @return NONE
 */
void tjrc_oblique_rightcross_patchLine(uint8_t L_top_i, uint8_t R_top_i)
{
    uint8_t m1, m2;
    FitStraightLine(R_top_i, R_top_i + 5, 0);
    //printf("k_l:%f,b_l:%f\n", k_l, b_l);
    for (int i = R_edge[R_top_i].row; i < 80; i++)
    {
        R_edge_use[i].row = i;
        R_edge_use[i].col = (int)(i * k_r + b_r);
        //printf("hh(%d,%d)\n", L_edge_use[i].row, L_edge_use[i].col);
        if (R_edge_use[i].col < 0)
        {
            R_edge_use[i].col = 0;
        }
        if (R_edge_use[i].col > 119)
        {
            R_edge_use[i].col = 119;
        }
    }

    m1 = R_edge[R_top_i].row;
    for (int i = R_top_i; i < R_edge_count; i++)
    {
        R_edge_use[m1].row = R_edge[i].row;
        R_edge_use[m1].col = R_edge[i].col;
        m1--;
    }
    //显示该直线

    if ((center_lost_corner_row_l - L_corner_row) != 0)
    {
        line_left_top[0] = (float)(center_lost_corner_col_l - L_corner_col) / (float)(center_lost_corner_row_l - L_corner_row);   //k
        line_left_top[1] = (float)(L_corner_col - line_left_top[0] * L_corner_row);  //b
    }
    //printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
    //显示该直线
    if (L_corner_row > center_lost_corner_row_l)
    {
        m1 = center_lost_corner_row_l;
        m2 = L_corner_row;
    }
    else
    {
        m2 = center_lost_corner_row_l;
        m1 = L_corner_row;
    }
    //printf("  m1=%d,m2=%d\n", m1, m2);
    for (int i = m1; i < m2; i++)
    {
        L_edge_use[i].row = i;
        L_edge_use[i].col = (int)(i * line_left_top[0] + line_left_top[1]);
        //printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
        if (L_edge_use[i].col < 0)
        {
            L_edge_use[i].col = 0;
        }
        if (L_edge_use[i].col > 119)
        {
            L_edge_use[i].col = 119;
        }
    }
    for (int i = L_beh_i; i > 0; i--)
    {
        L_edge_use[m2].row = L_edge[i].row;
        L_edge_use[m2].col = L_edge[i].col;
        m2++;
    }
    for (int i = L_top_i; i < L_edge_count; i++)
    {
        L_edge_use[m1].row = L_edge[i].row;
        L_edge_use[m1].col = L_edge[i].col;
        m1--;
    }
}

/**
 * @brief 获取中线（未使用）
 * @return NONE
 */
int32_t size1 = 0;
void get_mid()
{
   size1 = 1;
   //相关变量
   uint8_t mid_cnt = 0;      //用于向下拟合，如果开启拟合，则用于记录新的中线点个数
   uint8_t last_mid_count = 0; //用于记录上一场中线的中线个数，用于中线平缓
   uint8_t up_cnt = 15;        //向上生长的限制个数
   //保存上一场中线数据
   for (int32_t i = 0; i < Mid_count; i++)
   {
       Last_M_Line[i].row = Mid_Line[i].row;
       Last_M_Line[i].col = Mid_Line[i].col;
   }
   last_mid_count = Mid_count;
   //情况1：左右均找到
   if (left_findflag && right_findflag)
   {
       size1 = 2;
       //初始化中线个数
       Mid_count = 0;
       //防止中线出错
       L_edge[0].row = L_edge[1].row;
       L_edge[0].col = L_edge[1].col;
       R_edge[0].row = R_edge[1].row;
       R_edge[0].col = R_edge[1].col;
       //左边线比右边线长
       if (L_edge_count >= R_edge_count)
       {
           float k = 1.0 * L_edge_count / R_edge_count;
           for (int32_t i = 0; i < R_edge_count; i++)
           {
               Mid_count = Mid_count + 1;
               M_Line[i].row = (uint8_t)((L_edge[(uint8_t)(k * i)].row + R_edge[i].row) / 2);
               M_Line[i].col = (uint8_t)((L_edge[(uint8_t)(k * i)].col + R_edge[i].col) / 2);
               M_Line[i].flag = searchpoint;
           }
       }
       //右边线比左边线长
       else if (L_edge_count < R_edge_count)
       {
           float k = 1.0 * R_edge_count / L_edge_count;
           for (int32_t i = 0; i < L_edge_count; i++)
           {
               Mid_count = Mid_count + 1;
               M_Line[i].row = (uint8_t)((L_edge[i].row + R_edge[(uint8_t)(k * i)].row) / 2);
               M_Line[i].col = (uint8_t)((L_edge[i].col + R_edge[(uint8_t)(k * i)].col) / 2);
               M_Line[i].flag = searchpoint;
           }
       }
   }
   //情况2：单边全丢线
   //      右边界丢线，左边界不丢线||边线数量差距大
   else if ((left_findflag == 1 && right_findflag == 0) || (left_findflag && L_edge_count - R_edge_count > 70))
   {
       //防止中线出错
       L_edge[0].row = L_edge[1].row;
       L_edge[0].col = L_edge[1].col;
       //初始化中线个数
       Mid_count = 0;
       for (int32_t i = 0; i < L_edge_count; i++)
       {
           int16_t col = ((2 * (int16_t)L_edge[i].col + (int16_t)Road_Width[Image_H - L_edge[i].row]) / 2);
           if (col > Image_W - 1)    col = Image_W - 1;
           else if (col < 0)  col = 0;
           Mid_count = Mid_count + 1;
           M_Line[i].row = L_edge[i].row;
           M_Line[i].col = (uint8_t)col;
           M_Line[i].flag = searchpoint;
       }
   }
   //      左边线丢线，右边线不丢线
   else if ((left_findflag == 0 && right_findflag == 1) || (right_findflag && R_edge_count - L_edge_count > 70))
   {
       //防止中线出错
       R_edge[0].row = R_edge[1].row;
       R_edge[0].col = R_edge[1].col;
       //初始化中线个数
       Mid_count = 0;
       for (int32_t i = 0; i < R_edge_count; i++)
       {
           int16_t col = ((2 * (int16_t)R_edge[i].col - (int16_t)Road_Width[Image_H - R_edge[i].row]) / 2);
           if (col > Image_W - 1)    col = Image_W - 1;
           else if (col < 0)  col = 0;
           Mid_count = Mid_count + 1;
           M_Line[i].row = R_edge[i].row;
           M_Line[i].col = (uint8_t)col;
           M_Line[i].flag = searchpoint;
       }
   }
   //情况3：两边全丢线
   //情况4：左右边均存在，但中线太少
   if (Mid_count > 0)
   {
       uint8_t down_cnt = 15;
       //车头附近赛道没找到，则向下拟合
       if (M_Line[0].row < Image_H - 2 && M_Line[0].row > Image_H / 4 && Mid_count > 2)
       {
           int32_t num = Mid_count / 4;
           int32_t sumX = 0, sumY = 0;
           float sumUP = 0, sumDown = 0, avrX = 0, avrY = 0, K = 0, B = 0;
           for (int32_t i = 0; i < num; i++)
           {
               sumX = sumX + M_Line[i].row;
               sumY = sumY + M_Line[i].col;
           }
           avrX = 1.0 * sumX / num;
           avrY = 1.0 * sumY / num;
           for (int32_t i = 0; i < num; i++)
           {
               sumUP = sumUP + (M_Line[i].col - avrY) * (M_Line[i].row - avrX);
               sumDown = sumDown + (M_Line[i].row - avrX) * (M_Line[i].row - avrX);
           }
           if (sumDown == 0)
           {
               K = 0;
           }
           else
           {
               K = 1.0 * sumUP / sumDown;
               B = 1.0 * (sumY - K * sumX) / num;
           }
           for (int32_t i = M_Line[0].row; i < Image_H; i++)  //开始向下拟合
           {
               int32_t col = K * i + B;
               if (col > Image_W - 1) col = Image_W - 1;
               else if (col < 0)    col = 0;
               MID_LINE[mid_cnt].row = (uint8_t)i;
               MID_LINE[mid_cnt].col = (uint8_t)col;
               mid_cnt = mid_cnt + 1;
               if (--down_cnt == 0) break;
           }
       }
       //中线点个数太少，开启向上拟合
       if (Mid_count + mid_cnt < 60 && Mid_count >2)
       {
           int32_t num = Mid_count / 4;
           int32_t sumX = 0, sumY = 0;
           float sumUP = 0, sumDown = 0, avrX = 0, avrY = 0, K, B;
           for (int32_t i = Mid_count - num - 1; i < Mid_count; i++)
           {
               sumX = sumX + M_Line[i].row;
               sumY = sumY + M_Line[i].col;
           }
           avrX = 1.0 * sumX / num;
           avrY = 1.0 * sumY / num;
           for (int32_t i = Mid_count - num - 1; i < Mid_count; i++)
           {
               sumUP = sumUP + (M_Line[i].col - avrY) * (M_Line[i].row - avrX);
               sumDown = sumDown + (M_Line[i].row - avrX) * (M_Line[i].row - avrX);
           }
           if (sumDown == 0)
           {
               K = 0;
           }
           else
           {
               K = 1.0 * sumUP / sumDown;
               B = 1.0 * (sumY - K * sumX) / num;
           }
           for (int32_t i = M_Line[Mid_count - 1].row; i > 0; i--)  //开始向上拟合
           {
               int32_t col = K * i + B;
               if (col > Image_W - 1) col = Image_W - 1;
               else if (col < 0)    col = 0;
               M_Line[Mid_count].row = (uint8_t)i;
               M_Line[Mid_count].col = (uint8_t)col;
               Mid_count = Mid_count + 1;
               if (--up_cnt == 0) break;
           }
       }
       //如果向下拟合了，则搬运中线代码至
       if (mid_cnt > 0)
       {
           for (int32_t i = 0; i < Mid_count; i++)
           {
               MID_LINE[mid_cnt].row = M_Line[i].row;
               MID_LINE[mid_cnt].col = M_Line[i].col;
               mid_cnt = mid_cnt + 1;
           }
       }
   }
   if (mid_cnt > 0)
   {
       Mid_count = mid_cnt;
       Mid_Line = MID_LINE;
   }
   else
       Mid_Line = M_Line;
   //中线平缓
   float k = 0.8;
   float k2;
   for (int32_t i = 0; i < Mid_count - 1; i++)
   {
       Mid_Line[i].row = (Mid_Line[i].row + Mid_Line[i + 1].row) / 2;
       Mid_Line[i].col = (Mid_Line[i].col + Mid_Line[i + 1].col) / 2;
   }
   for (int32_t i = Mid_count - 1; i > 1; i--)
   {
       Mid_Line[i].row = (Mid_Line[i].row + Mid_Line[i - 1].row) / 2;
       Mid_Line[i].col = (Mid_Line[i].col + Mid_Line[i - 1].col) / 2;
   }
   if (last_mid_count >= Mid_count)
   {
       k2 = 1.0 * last_mid_count / Mid_count;
       for (int32_t i = 0; i < Mid_count; i++)
       {
           Mid_Line[i].row = (1 - k) * Last_M_Line[(uint8_t)(k2 * i)].row + k * Mid_Line[i].row;
           Mid_Line[i].col = (1 - k) * Last_M_Line[(uint8_t)(k2 * i)].col + k * Mid_Line[i].col;
       }
   }
   else if (last_mid_count < Mid_count)
   {
       k2 = 1.0 * Mid_count / last_mid_count;
       for (int32_t i = 0; i < last_mid_count; i++)
       {
           Mid_Line[i].row = (1 - k) * Last_M_Line[i].row + k * Mid_Line[(uint8_t)(k2 * i)].row;
           Mid_Line[i].col = (1 - k) * Last_M_Line[i].col + k * Mid_Line[(uint8_t)(k2 * i)].col;
       }
   }
}

/**
 * @brief 初始化所有变量
 * @return NONE
 */
void clear_point()
{
   for (int32_t i = 0; i < 140; i++)
   {
       L_edge[i].row = 0;
       L_edge[i].col = 0;
       L_edge[i].flag = 0;
   }
   for (int32_t i = 0; i < 140; i++)
   {
       R_edge[i].row = 0;
       R_edge[i].col = 0;
       R_edge[i].flag = 0;
   }
   //变量初始化
   for (int i = 0; i < 80; i++)
   {
       L_edge_use[i].row = 0;
       L_edge_use[i].col = 0;
       L_edge_use[i].flag = 0;
       R_edge_use[i].row = 0;
       R_edge_use[i].col = 119;
       R_edge_use[i].flag = 0;
       Mid_Line[i].row=0;
       Mid_Line[i].col=60;
   }

   //变量初始化
   dot_left_bottom_l = 0;   //左下拐点，趋势左
   dot_left_bottom_r = 0;   //左下拐点，趋势右
   dot_left_top_l = 0;      //左上拐点，趋势左
   dot_left_top_r = 0;      //左上拐点，趋势右
   dot_right_bottom_l = 0;  //右下拐点，趋势左
   dot_right_bottom_r = 0;  //右下拐点，趋势右
   dot_right_top_l = 0;     //右上拐点，趋势左
   dot_right_top_r = 0;     //右上拐点，趋势右

   //直线斜率数据存放数组 //变量初始化
   line_left_bottom[0] = 0;
   line_left_bottom[1] = 0;
   line_left_top[0] = 0;
   line_left_top[1] = 0;
   line_right_top[0] = 0;
   line_right_top[1] = 0;
   line_right_bottom[0] = 0;
   line_right_bottom[1] = 0;

   //变量初始化
   is_line_left_bottom = 0;
   is_line_left_top = 0;
   is_line_right_bottom = 0;
   is_line_right_top = 0;

   //直线总体偏差 //变量初始化
   dist_sum_left_bottom = 0;
   dist_sum_left_top = 0;
   dist_sum_right_bottom = 0;
   dist_sum_right_top = 0;

   //变量初始化
   center_lost_row_l = 0;     //中间左丢线开始行坐标
   center_lost_col_l = 0; //中间左丢线开始列坐标
   center_lost_row_r = 0;     //中间左丢线开始行坐标
   center_lost_col_r = 0; //中间左丢线开始列坐标

   L_start_lost = 0;
   center_lost_flag_l = 0;
   R_start_lost = 0;
   center_lost_flag_r = 0;

   L_corner_flag = 0;// 初始化变量
   L_corner_row = 0;
   L_corner_col = 0;
   L_corner_angle = 0;
   R_corner_flag = 0;// 初始化变量
   R_corner_row = 0;
   R_corner_col = 0;
   R_corner_angle = 0;
}
/*--------------------------------------------------------------------------
* 【函数功能】：求拐点的角度值
* 【参    数】：三个点的行列坐标
* 【返 回 值】：角度值
* 【备    注】：无
*--------------------------------------------------------------------------*/
//int32_t Get_angle(uint8_t ax, uint8_t ay, uint8_t bx, uint8_t by, uint8_t cx, uint8_t cy)
//{
//  int8 abx = ax - bx;
//  int8 aby = ay - by;
//  int8 cbx = cx - bx;
//  int8 cby = cy - by;
//  int8 ab_muti_cb = abx * cbx + aby * cby;
//  int8 dist_ab = sqrt(abx * abx + aby * aby);
//  int8 dist_cb = sqrt(cbx * cbx + cby * cby);
//  int8 cosvalue = ab_muti_cb / (dist_ab * dist_cb);
//  return (int32_t)(acos(cosvalue) * 180 / 3.14159);
//}

/**
 * @brief 计算拐点角度值（未使用）
 * @param ax      (uint8_t)ax
 * @param ay      (uint8_t)ay
 * @param bx      (uint8_t)bx=ax+4
 * @param by      (uint8_t)by=ay+4
 * @param cx      (uint8_t)cx=ax+8
 * @param cy      (uint8_t)cy=ay+8
 * @return 角度
 */
int32_t Get_angle(uint8_t ax, uint8_t ay, uint8_t bx, uint8_t by, uint8_t cx, uint8_t cy)
{
   float abx = ax - bx;
   float aby = ay - by;
   float cbx = cx - bx;
   float cby = cy - by;
   float ab_muti_cb = abx * cbx + aby * cby;
   float dist_ab = sqrt(abx * abx + aby * aby);
   float dist_cb = sqrt(cbx * cbx + cby * cby);
   float cosvalue = ab_muti_cb / (dist_ab * dist_cb);
   return (int32_t)(acos(cosvalue) * 180 / 3.14159);
}
/*--------------------------------------------------------------------------
* 【函数功能】：判断是否存在左/右边界点（未使用）
* 【参    数】：搜线基本行数值      左/右边界选择标志(0:左边，1:右边)
* 【返 回 值】：-1 或 边界点列值，-1表示没有找到边界点
* 【备    注】：无
*--------------------------------------------------------------------------*/
int32_t edge_point_ornot(uint8_t image[MT9V03X_H][MT9V03X_W],uint8_t row, uint8_t side)  //5和155需要调整
{
   //左边线
   if (side == 0)
       {
           uint8_t find_edge = 0;
           L_lost_count = 0;
           //从开始搜线行向上生长
           for (uint8_t rowi = row; rowi > 0; rowi--)
           {
               //如果图像最左侧为赛道外（未丢线），则开始横向生长
               if (L_lost_count <= L_lost_)
               {
                   //横向生长
                   for (int32_t col = Image_W / 2; col > L_edge_start_col; col--)
                   {
                       //printf("image[%d][%d],is black:%d\n", rowi, col, black_(image[rowi][col]));
//                        if (black_(image[rowi][col]))
//                        if(black_2((uint8_t*)imageout[0],rowi,col))
                       if(black_2((uint8_t*)image[0],rowi,col))
                       {
                           //printf("col:%d\n ", col);
                           //如果出现黑黑白白，则判断为边界线，退出循环
//                            if (black_(image[rowi][col]) && black_(image[rowi][col - 1]) && white_(image[rowi][col + 2]) && white_(image[rowi][col + 1]))
//                            if (black_2((uint8_t*)imageout[0],rowi,col) && black_2((uint8_t*)imageout[0],rowi,col - 1) && white_2((uint8_t*)imageout[0],rowi,col + 2) && white_2((uint8_t*)imageout[0],rowi,col + 1))
                           if (black_2((uint8_t*)image[0],rowi,col) && black_2((uint8_t*)image[0],rowi,col - 1) && white_2((uint8_t*)image[0],rowi,col + 2) && white_2((uint8_t*)image[0],rowi,col + 1))
                           {
                               L_basic_row_start = rowi;   //赋值开始搜线行（左）
//                                printf("left start：(x=%d,y=%d)\n", rowi, col);
                               //if (rowi<Image_H / 2 && col>Image_W / 2) return -1;
                               return col ;               //返回列值         //!!!   1->2
                           }
                       }
                       if (col == L_edge_start_col + 1)
                       {
                           L_lost_count++;
                           //printf("L_lost_count:%d\n", L_lost_count);
                       }
                   }
               }
               //printf("lost %d ", L_lost_count);
               if (L_lost_count > L_lost_)//丢线大于次数限制开启行搜索
               {
                   //如果出现黑黑白白，则判断为边界线，退出循环
                   break;
               }
               if (find_edge == 2) return -1;
           }
           if (L_lost_count > L_lost_)//丢线大于次数限制开启行搜索
           {
               for (uint8_t rowi = row; rowi > 0; rowi--)
               {
                   //如果出现黑黑白白，则判断为边界线，退出循环
//                    if (black_(image[rowi - 3][L_edge_start_col]) && black_(image[rowi - 2][L_edge_start_col]) && white_(image[rowi - 1][L_edge_start_col]) && white_(image[rowi][L_edge_start_col]))
//                    if (black_2((uint8_t*)imageout[0],rowi - 3,L_edge_start_col) && black_2((uint8_t*)imageout[0],rowi - 2,L_edge_start_col) && white_2((uint8_t*)imageout[0],rowi - 1,L_edge_start_col) && white_2((uint8_t*)imageout[0],rowi,L_edge_start_col))
                   if (black_2((uint8_t*)image[0],rowi - 3,L_edge_start_col) && black_2((uint8_t*)image[0],rowi - 2,L_edge_start_col) && white_2((uint8_t*)image[0],rowi - 1,L_edge_start_col) && white_2((uint8_t*)image[0],rowi,L_edge_start_col))
                   {
                       L_basic_row_start = rowi - 2;   //赋值开始搜线行（左）
//                        printf("left lost start：(x=%d,y=%d)\n", L_basic_row_start, L_edge_start_col);
                       L_start_lost = 1;
                       //if (rowi<Image_H / 2 && col>Image_W / 2) return -1;
                       return L_edge_start_col;               //返回列值         //!!!   1->2
                   }
               }
           }
       }
       //右边线
       if (side == 1)
       {
           uint8_t find_edge = 0;
           R_lost_count = 0;
           for (uint8_t rowi = row; rowi > 0; rowi--)
           {
               //如果图像最右侧为赛道外（未丢线），则开始横向生长
               if (R_lost_count <= R_lost_)
               {
                   //横向生长
                   for (int32_t col = Image_W / 2; col < R_edge_start_col; col++)
                   {
//                        if (black_(image[rowi][col]))
//                        if (black_2((uint8_t*)imageout[0],rowi,col))
                       if (black_2((uint8_t*)image[0],rowi,col))
                       {
                           //如果出现黑黑白白，则判断为边界线，退出循环
//                            if (white_(image[rowi][col - 2]) && white_(image[rowi][col - 1]) && black_(image[rowi][col + 1]) && black_(image[rowi][col]))
//                            if (white_2((uint8_t*)imageout[0],rowi,col - 2) && white_2((uint8_t*)imageout[0],rowi,col - 1) && black_2((uint8_t*)imageout[0],rowi,col + 1) && black_2((uint8_t*)imageout[0],rowi,col))
                           if (white_2((uint8_t*)image[0],rowi,col - 2) && white_2((uint8_t*)image[0],rowi,col - 1) && black_2((uint8_t*)image[0],rowi,col + 1) && black_2((uint8_t*)image[0],rowi,col))
                           {
                               R_basic_row_start = rowi;
//                                printf("right start：(x=%d,y=%d)\n", rowi, col);
                               if (rowi < Image_H / 2 && col < Image_W / 2) return -1;
                               return col;
                           }
                       }
                       if (col == R_edge_start_col - 1)
                       {
                           R_lost_count++;
                       }
                   }
               }

               if (R_lost_count > R_lost_)//丢线大于次数限制开启行搜索
               {
                   break;
               }
               if (find_edge == 2) return -1;
           }
           if (R_lost_count > R_lost_)//丢线大于次数限制开启行搜索
           {
               for (uint8_t rowi = row; rowi > 0; rowi--)
               {
//                    if (black_(image[rowi - 3][R_edge_start_col]) && black_(image[rowi - 2][R_edge_start_col]) && white_(image[rowi - 1][R_edge_start_col]) && white_(image[rowi][R_edge_start_col]))
//                    if (black_2((uint8_t*)imageout[0],rowi - 3,R_edge_start_col) && black_2((uint8_t*)imageout[0],rowi - 2,R_edge_start_col) && white_2((uint8_t*)imageout[0],rowi - 1,R_edge_start_col) && white_2((uint8_t*)imageout[0],rowi,R_edge_start_col))
                   if (black_2((uint8_t*)image[0],rowi - 3,R_edge_start_col) && black_2((uint8_t*)image[0],rowi - 2,R_edge_start_col) && white_2((uint8_t*)image[0],rowi - 1,R_edge_start_col) && white_2((uint8_t*)image[0],rowi,R_edge_start_col))
                   {
                       R_basic_row_start = rowi - 2;   //赋值开始搜线行（左）
//                        printf("right lost start：(x=%d,y=%d)\n", R_basic_row_start, R_edge_start_col);
                       R_start_lost = 1;
                       //if (rowi<Image_H / 2 && col>Image_W / 2) return -1;
                       return R_edge_start_col;               //返回列值         //!!!   1->2
                   }
               }
           }
       }
       return -1;
}

/**
 * @brief 判断黑点（阈值判断 or 二值判断）
 * @param x        (uint8_t)输入图像灰度值
 * @return 0 or 1
 */
uint8_t black_(uint8_t x)    //判断黑
{
   if (binimage_flag == 0)  //如果没有二值化，则通过阈值判断黑白
   {
       if (x < threshold)
           return 1;
       else if (x >= threshold)
           return 0;
   }
   else if (binimage_flag == 1)
   {
       if (x == 0)
           return 1;
       else if (x == 1)
           return 0;
   }
   return 0;
}

/**
 * @brief 判断白点（阈值判断 or 二值判断）
 * @param x        (uint8_t)输入图像灰度值
 * @return 0 or 1
 */
uint8_t white_(uint8_t x)    //判断白
{
   if (binimage_flag == 0)
   {
       if (x < threshold)
           return 0;
       else if (x >= threshold)
           return 1;
   }
   else if (binimage_flag == 1)
   {
       if (x == 1)
           return 1;
       else if (x == 0)
           return 0;
   }
   return 0;
}

/**
 * @brief 判断黑点（阈值判断 or 二值判断 or 单点sobel判断）
 * @param image    (uint8_t*)输入图像数组指针
 * @param row      (uint8_t)所在行
 * @param col      (uint8_t)所在列
 * @return 0 or 1
 */
uint8_t black_2(uint8_t* image,uint8_t row,uint8_t col)    //判断黑
{
   if (binimage_flag == 0)  //如果没有二值化，则通过阈值判断黑白
   {
       if (image[row*MT9V03X_W+col] < threshold)
           return 1;
       else if (image[row*MT9V03X_W+col] >= threshold)
           return 0;
   }
   else if (binimage_flag == 1)
   {
       if (image[row*MT9V03X_W+col] == 0)
           return 1;
       else if (image[row*MT9V03X_W+col] == 1)
           return 0;
   }
   else if(binimage_flag == 2)
   {
       /** 卷积核大小 */
       int32_t KERNEL_SIZE = 3;
       int32_t temp[4];
       /* 计算不同方向梯度幅值  */
       /* 90 deg  */
       temp[0] = -(int32_t)image[(row - 1)*MT9V03X_W + (col - 1)] + (int32_t)image[(row - 1)*MT9V03X_W + (col + 1)]     // -1,  0,  1
           - (int32_t)image[(row)*MT9V03X_W + (col - 1)] + (int32_t)image[(row)*MT9V03X_W + (col + 1)]                  // -1,  0,  1
           - (int32_t)image[(row + 1)*MT9V03X_W + (col - 1)] + (int32_t)image[(row + 1)*MT9V03X_W + (col + 1)];         // -1,  0,  1
       /* 0 deg  */
       temp[1] = -(int32_t)image[(row - 1)*MT9V03X_W + col - 1] + (int32_t)image[(row + 1)*MT9V03X_W + col - 1]     // -1, -1, -1
           - (int32_t)image[(row - 1)*MT9V03X_W + col] + (int32_t)image[(row + 1)*MT9V03X_W + col]                  //  0,  0,  0
           - (int32_t)image[(row - 1)*MT9V03X_W + col + 1] + (int32_t)image[(row + 1)*MT9V03X_W + col + 1];         //  1,  1,  1

       /* 45 deg  */
       temp[2] = -(int32_t)image[(row - 1)*MT9V03X_W + col] + (int32_t)image[(row)*MT9V03X_W + col - 1]             //  0, -1, -1
           - (int32_t)image[(row)*MT9V03X_W + col + 1] + (int32_t)image[(row + 1)*MT9V03X_W + col]                  //  1,  0, -1
           - (int32_t)image[(row - 1)*MT9V03X_W + col + 1] + (int32_t)image[(row + 1)*MT9V03X_W + col - 1];         //  1,  1,  0
       /* 135 deg  */
       temp[3] = -(int32_t)image[(row - 1)*MT9V03X_W + col] + (int32_t)image[(row)*MT9V03X_W + col + 1]             // -1, -1,  0
           - (int32_t)image[(row)*MT9V03X_W + col - 1] + (int32_t)image[(row + 1)*MT9V03X_W + col]                  // -1,  0,  1
           - (int32_t)image[(row - 1)*MT9V03X_W + col - 1] + (int32_t)image[(row + 1)*MT9V03X_W + col + 1];         //  0,  1,  1

       temp[0] = abs(temp[0]);
       temp[1] = abs(temp[1]);
       temp[2] = abs(temp[2]);
       temp[3] = abs(temp[3]);

       /* 找出梯度幅值最大值  */
       for (int16_t k = 1; k < 4; k++)
       {
           if (temp[0] < temp[k])
           {
               temp[0] = temp[k];
           }
       }

       /* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
       temp[3] = (int32_t)image[(row - 1)*MT9V03X_W + col - 1] + (int32_t)image[(row - 1)*MT9V03X_W + col] + (int32_t)image[(row - 1)*MT9V03X_W + col + 1]
           + (int32_t)image[(row)*MT9V03X_W + col - 1] + (int32_t)image[(row)*MT9V03X_W + col] + (int32_t)image[(row)*MT9V03X_W + col + 1]
           + (int32_t)image[(row + 1)*MT9V03X_W + col - 1] + (int32_t)image[(row + 1)*MT9V03X_W + col] + (int32_t)image[(row + 1)*MT9V03X_W + col + 1];

       if (temp[0] >= temp[3] / 10.0f)
       {
           return 0;
       }
       else
       {
           return 1;
       }
   }
   return 0;
}

/**
 * @brief 判断白点（阈值判断 or 二值判断 or 单点sobel判断）
 * @param image    (uint8_t*)输入图像数组指针
 * @param row      (uint8_t)所在行
 * @param col      (uint8_t)所在列
 * @return 0 or 1
 */
uint8_t white_2(uint8_t* image,uint8_t row,uint8_t col)    //判断白
{
   if (binimage_flag == 0)
   {
       if (image[row*MT9V03X_W+col] < threshold)
           return 0;
       else if (image[row*MT9V03X_W+col] >= threshold)
           return 1;
   }
   else if (binimage_flag == 1)
   {
       if (image[row*MT9V03X_W+col] == 1)
           return 1;
       else if (image[row*MT9V03X_W+col] == 0)
           return 0;
   }
   else if(binimage_flag == 2)
   {
       /** 卷积核大小 */
       int32_t KERNEL_SIZE = 3;
       int32_t temp[4];
       /* 计算不同方向梯度幅值  */
       /* 90 deg  */
       temp[0] = -(int32_t)image[(row - 1)*MT9V03X_W + (col - 1)] + (int32_t)image[(row - 1)*MT9V03X_W + (col + 1)]     // -1,  0,  1
           - (int32_t)image[(row)*MT9V03X_W + (col - 1)] + (int32_t)image[(row)*MT9V03X_W + (col + 1)]                  // -1,  0,  1
           - (int32_t)image[(row + 1)*MT9V03X_W + (col - 1)] + (int32_t)image[(row + 1)*MT9V03X_W + (col + 1)];         // -1,  0,  1
       /* 0 deg  */
       temp[1] = -(int32_t)image[(row - 1)*MT9V03X_W + col - 1] + (int32_t)image[(row + 1)*MT9V03X_W + col - 1]     // -1, -1, -1
           - (int32_t)image[(row - 1)*MT9V03X_W + col] + (int32_t)image[(row + 1)*MT9V03X_W + col]                  //  0,  0,  0
           - (int32_t)image[(row - 1)*MT9V03X_W + col + 1] + (int32_t)image[(row + 1)*MT9V03X_W + col + 1];         //  1,  1,  1

       /* 45 deg  */
       temp[2] = -(int32_t)image[(row - 1)*MT9V03X_W + col] + (int32_t)image[(row)*MT9V03X_W + col - 1]             //  0, -1, -1
           - (int32_t)image[(row)*MT9V03X_W + col + 1] + (int32_t)image[(row + 1)*MT9V03X_W + col]                  //  1,  0, -1
           - (int32_t)image[(row - 1)*MT9V03X_W + col + 1] + (int32_t)image[(row + 1)*MT9V03X_W + col - 1];         //  1,  1,  0
       /* 135 deg  */
       temp[3] = -(int32_t)image[(row - 1)*MT9V03X_W + col] + (int32_t)image[(row)*MT9V03X_W + col + 1]             // -1, -1,  0
           - (int32_t)image[(row)*MT9V03X_W + col - 1] + (int32_t)image[(row + 1)*MT9V03X_W + col]                  // -1,  0,  1
           - (int32_t)image[(row - 1)*MT9V03X_W + col - 1] + (int32_t)image[(row + 1)*MT9V03X_W + col + 1];         //  0,  1,  1

       temp[0] = abs(temp[0]);
       temp[1] = abs(temp[1]);
       temp[2] = abs(temp[2]);
       temp[3] = abs(temp[3]);

       /* 找出梯度幅值最大值  */
       for (int16_t k = 1; k < 4; k++)
       {
           if (temp[0] < temp[k])
           {
               temp[0] = temp[k];
           }
       }

       /* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
       temp[3] = (int32_t)image[(row - 1)*MT9V03X_W + col - 1] + (int32_t)image[(row - 1)*MT9V03X_W + col] + (int32_t)image[(row - 1)*MT9V03X_W + col + 1]
               + (int32_t)image[(row)*MT9V03X_W + col - 1] + (int32_t)image[(row)*MT9V03X_W + col] + (int32_t)image[(row)*MT9V03X_W + col + 1]
               + (int32_t)image[(row + 1)*MT9V03X_W + col - 1] + (int32_t)image[(row + 1)*MT9V03X_W + col] + (int32_t)image[(row + 1)*MT9V03X_W + col + 1];

       if (temp[0] >= temp[3] / 10.0f)
       {
           return 1;
       }
       else
       {
           return 0;
       }
   }
   return 0;
}

/*--------------------------------------------------------------------------
* 【函数功能】：均值滤波（未使用）
* 【参    数】：无
* 【返 回 值】：无
*--------------------------------------------------------------------------*/
int32_t average_value(uint8_t* c)
{
   int32_t i;
   int32_t k;
   uint16_t t_sum;
   t_sum = 0;
   for (i = 0; i <= 8; i++)
   {
       t_sum = t_sum + c[i];
   }
   k = t_sum / 9;
   return k;
}

void average_filter(void)
{
   uint8_t i, j;
   uint8_t  c[9];
   for (i = 1; i < Image_H - 1; i++)
   {
       for (j = 1; j < Image_W - 1; j++)
       {
           if (i >= 8)
           {
               c[0] = image[i - 1][j - 1];
               c[1] = image[i - 1][j];
               c[2] = image[i - 1][j + 1];
               c[3] = image[i][j - 1];
               c[4] = image[i][j];
               c[5] = image[i][j + 1];
               c[6] = image[i + 1][j - 1];
               c[7] = image[i + 1][j];
               c[8] = image[i + 1][j + 1];
               image[i][j] = average_value(c);
           }

       }
   }

}


// 点集三角滤波（未使用）
void blur_points(struct LEFT_EDGE pts_in[120], struct LEFT_EDGE pts_out[120], int32_t num, int32_t kernel)
{
   int32_t half = kernel / 2;
   for (int32_t i = 0; i < num; i++) {
       pts_out[i].row = pts_out[i].col = 0;
       for (int32_t j = -half; j <= half; j++) {
           pts_out[i].row += pts_in[clip(i + j, 0, num - 1)].row * (half + 1 - abs(j));
           pts_out[i].col += pts_in[clip(i + j, 0, num - 1)].col * (half + 1 - abs(j));
       }
       pts_out[i].row /= (2 * half + 2) * (half + 1) / 2;
       pts_out[i].col /= (2 * half + 2) * (half + 1) / 2;
       //printf("row:%d col:%d\n", pts_out[i].row);
   }
}

//边线截断处理（未使用）
void edge_truncation(uint8_t side)
{
   if (side == 0)
   {
       //开始丢线情况下，处理方式
       if (L_start_lost)
       {
           FitStraightLine(L_top_i, L_top_i + 10, 0);
           printf("左开始丢线:k=%f,b=%f\n", k_l, b_l);
       }
       else if (center_lost_flag_l) //中间丢线情况下，处理方式
       {
           float k = 0, b = 0;
           FitStraightLine(L_top_i, L_top_i + 10, 0);
           k = k_l;
           b = b_l;

           k_l = (L_corner_col - center_lost_corner_col_l) / (L_corner_row - center_lost_corner_row_l);
           b_l = L_corner_col - k_l * L_corner_row;
           k_l = (k_l + k) / 2.0;
           b_l = (b_l + b) / 2.0;
           //FitStraightLine(L_beh_i - 9, L_beh_i+1, 0);
           //k_l = (k_l + k) / 2.0;
           //b_l = (b_l + b) / 2.0;
           printf("左中间丢线:k=%f,b=%f\n", k_l, b_l);
       }
       else //左边未丢线情况
       {
           FitStraightLine(L_beh_i - 9, L_beh_i + 1, 0);
           printf("左边未丢线:k=%f,b=%f\n", k_l, b_l);
       }
       //if (L_edge_count >= 70)
       //{
       //  num_cnt = 0;//记录连续水平点的个数
       //  L_count = L_edge_count / 2;
       //  while (L_count < L_edge_count)
       //  {
       //      if (L_edge[L_count].row == L_edge[L_count + 1].row)
       //          num_cnt = num_cnt + 1;
       //      else
       //          num_cnt = 0;
       //      if (num_cnt > 5)//连续5个点水平
       //          break;
       //      L_count = L_count + 1;
       //  }
       //  L_edge_count = L_count;//截断在5个水平点处
       //}
   }
   if (side == 1)
   {
       //开始丢线情况下，处理方式
       if (R_start_lost)
       {
           FitStraightLine(R_top_i, R_top_i + 10, 1);
           printf("右开始丢线:k=%f,b=%f\n", k_r, b_r);
       }
       else if (center_lost_flag_r) //中间丢线情况下，处理方式
       {
           float k = 0, b = 0;
           FitStraightLine(R_top_i, R_top_i + 10, 1);
           k = k_r;
           b = b_r;

           k_r = (R_corner_col - center_lost_corner_col_r) / (R_corner_row - center_lost_corner_row_r);
           b_r = R_corner_col - k_r * R_corner_row;
           k_r = (k_r + k) / 2.0;
           b_r = (b_r + b) / 2.0;
           //FitStraightLine(R_beh_i - 9, R_beh_i+1, 1);
           //k_r = (k_r + k) / 2.0;
           //b_r = (b_r + b) / 2.0;
           printf("右中间丢线:k=%f,b=%f\n", k_r, b_r);
       }
       else //左边未丢线情况
       {
           FitStraightLine(R_beh_i - 9, R_beh_i + 1, 1);
           printf("右边未丢线:k=%f,b=%f\n", k_r, b_r);
       }
       //if (L_edge_count >= 70)
       //{
       //  num_cnt = 0;//记录连续水平点的个数
       //  L_count = L_edge_count / 2;
       //  while (L_count < L_edge_count)
       //  {
       //      if (L_edge[L_count].row == L_edge[L_count + 1].row)
       //          num_cnt = num_cnt + 1;
       //      else
       //          num_cnt = 0;
       //      if (num_cnt > 5)//连续5个点水平
       //          break;
       //      L_count = L_count + 1;
       //  }
       //  L_edge_count = L_count;//截断在5个水平点处
       //}
   }

}

/**
 * @brief 最小二乘拟合
 * @param start     (uint8_t)起始点
 * @param end       (uint8_t)终止点
 * @param side      (uint8_t)左  or 右
 * @return NONE
 */
void FitStraightLine(int32_t start, int32_t end, uint8_t side)
{
   int32_t n = end - start;
   int32_t x_add = 0;
   int32_t y_add = 0;
   int32_t xy_add = 0;
   int32_t xx_add = 0;
   float x_bar = 0.0;
   float y_bar = 0.0;
   float xy_bar = 0.0;
   float xx_bar = 0.0;
   float b_add = 0.0;

   if (side == 0)
   {
       for (int32_t i = start; i < end; i++)
       {
           //printf("(%d,%d)\n", L_edge[i].row, L_edge[i].col);
           x_add += L_edge[i].row;
           y_add += L_edge[i].col;
           xx_add += (L_edge[i].row * L_edge[i].row);
           xy_add += (L_edge[i].row * L_edge[i].col);
       }

       x_bar = x_add * 1.0 / n;
       y_bar = y_add * 1.0 / n;
       xx_bar = xx_add * 1.0 / n;
       xy_bar = xy_add * 1.0 / n;
       //printf("kf=%f", (xy_bar - x_bar * y_bar) / (xx_bar - x_bar * x_bar));

       k_l = (xy_bar - x_bar * y_bar) / (xx_bar - x_bar * x_bar); //拟合直线斜率
       b_l = y_bar - k_l * x_bar;//拟合直线的常数项
   }
   if (side == 1)
   {
       for (int32_t i = start; i < end; i++)
       {
           //printf("(%d,%d)\n", R_edge[i].row, R_edge[i].col);
           x_add += R_edge[i].row;
           y_add += R_edge[i].col;
           xx_add += (R_edge[i].row * R_edge[i].row);
           xy_add += (R_edge[i].row * R_edge[i].col);
       }

       x_bar = x_add * 1.0 / n;
       y_bar = y_add * 1.0 / n;
       xx_bar = xx_add * 1.0 / n;
       xy_bar = xy_add * 1.0 / n;

       k_r = (xy_bar - x_bar * y_bar) / (xx_bar - x_bar * x_bar); //拟合直线斜率
       b_r = y_bar - k_r * x_bar;//拟合直线的常数项
   }
}


// 左手迷宫巡线  （未使用）
void findline_lefthand_adaptive(uint8_t image[MT9V03X_H][MT9V03X_W],int32_t jilu_row, int32_t jilu_col, int32_t search_amount)
{
   L_edge[0].row = jilu_row;
   L_edge[0].col = jilu_col;
   L_edge[0].flag = 1;
   uint8_t curr_row = jilu_row;//初始化行坐标
   uint8_t curr_col = jilu_col;//初始化列坐标
   dire_left = 0; //初始化上个边界点的来向
   center_turn_flag = 1;//初始化在未标定状态
   center_lost_flag_l = 0;//中间左丢线标志位
   uint8_t turn=0;
   //开始搜线，最多取150个点，不会往下搜，共7个方位
   for (int32_t i = 1; i < search_amount; i++)    //最多搜索70个点
   {
       ////越界退出 行越界和列越界（向上向下向左向右）
       if (curr_row < L_edge_end_row || curr_row>Image_H - 1 || curr_row + 1 < L_edge_end_row)  break;
       if (curr_col > max_col || curr_col < min_col && !center_lost_flag_l)   //max=160-5 min=5
       {
           if (++L_search_edge_count == edge_count_amount)//连续3次搜索到边界，退出      //连续搜索到边界次数需要调整  5和155需要调整
           {
               curr_col = L_edge_lost_start_col;
//              if (L_edge_count > line_amount)break;
               for (uint8_t rowi = curr_row; rowi > 0; rowi--)
               {
                   //printf("row=%d", rowi);
//                  if (black_(image[rowi - 3][curr_col]) && black_(image[rowi - 2][curr_col]) && white_(image[rowi - 1][curr_col]) && white_(image[rowi][curr_col]))
//                  if (black_2((uint8_t*)imageout[0],rowi - 3,curr_col) && black_2((uint8_t*)imageout[0],rowi - 2,curr_col) && white_2((uint8_t*)imageout[0],rowi - 1,curr_col) && white_2((uint8_t*)imageout[0],rowi,curr_col))
                   if (black_2((uint8_t*)image[0],rowi - 3,curr_col) && black_2((uint8_t*)image[0],rowi - 2,curr_col) && white_2((uint8_t*)image[0],rowi - 1,curr_col) && white_2((uint8_t*)image[0],rowi,curr_col))
                   {
                       printf("中间左丢线情况，左边线起始点：(x=%d,y=%d)\n", rowi - 1, L_edge_lost_start_col);
                       curr_row = rowi - 1;
                       curr_col = L_edge_lost_start_col;
                       center_lost_flag_l = 1;
                       center_lost_row_l = curr_row;     //中间左丢线开始行坐标
                       center_lost_col_l = curr_col + 1; //中间左丢线开始列坐标
                       dire_left = 0; //初始化上个边界点的来向
                       L_top_corner_start = i;//左上拐点开始序号
                       break;
                   }
                   if(rowi==1)turn=1;
               }
           }
       }
       else
           L_search_edge_count = 0;
       if(turn==1)break;
       //搜线过程
//      if (black_(image[curr_row + dir_front[dire_left][0]][curr_col + dir_front[dire_left][1]]))
//      if (black_2((uint8_t*)imageout[0],curr_row + dir_front[dire_left][0],curr_col + dir_front[dire_left][1]))
       if (black_2((uint8_t*)image[0],curr_row + dir_front[dire_left][0],curr_col + dir_front[dire_left][1]))
       {
           dire_left = (dire_left + 1) % 4;
           i = i - 1;
           //turn++;
       }
//      else if (black_(image[curr_row + dir_frontleft[dire_left][0]][curr_col + dir_frontleft[dire_left][1]]))
//      else if (black_2((uint8_t*)imageout[0],curr_row + dir_frontleft[dire_left][0],curr_col + dir_frontleft[dire_left][1]))
       else if (black_2((uint8_t*)image[0],curr_row + dir_frontleft[dire_left][0],curr_col + dir_frontleft[dire_left][1]))
       {
           curr_row += dir_front[dire_left][0];
           curr_col += dir_front[dire_left][1];
           L_edge[i].row = curr_row;
           L_edge[i].col = curr_col;
           L_edge[i].flag = 1;
           L_edge_count = L_edge_count + 1;
       }
       else
       {
           curr_row += dir_frontleft[dire_left][0];
           curr_col += dir_frontleft[dire_left][1];
           dire_left = (dire_left + 3) % 4;
           L_edge[i].row = curr_row;
           L_edge[i].col = curr_col;
           L_edge[i].flag = 1;
           L_edge_count = L_edge_count + 1;
       }
       //printf("(x1=%d,y1=%d)\n", L_edge[i].row, L_edge[i].col);
       //dd = dd + 1;
   }
   //printf("count%d ", L_edge_count);
}

// 右手迷宫巡线  （未使用）
void findline_righthand_adaptive(uint8_t image[MT9V03X_H][MT9V03X_W],int32_t jilu_row, int32_t jilu_col, int32_t search_amount)
{
   R_edge[0].row = jilu_row;
   R_edge[0].col = jilu_col;
   R_edge[0].flag = 1;
   uint8_t curr_row = jilu_row;//初始化行坐标
   uint8_t curr_col = jilu_col;//初始化列坐标
   dire_right = 0; //初始化上个边界点的来向
   center_turn_flag = 1;//初始化在未标定状态
   center_lost_flag_r = 0;//中间左丢线标志位
   uint8_t turn=0;
   //开始搜线，最多取150个点，不会往下搜，共7个方位
   for (int32_t i = 1; i < search_amount; i++)    //最多搜索70个点
   {
       ////越界退出 行越界和列越界（向上向下向左向右）
       if (curr_row < R_edge_end_row || curr_row>Image_H - 1 || curr_row + 1 < R_edge_end_row)  break;
       if (curr_col > max_col || curr_col < min_col && !center_lost_flag_r)   //max=160-5 min=5
       {
           if (++R_search_edge_count == edge_count_amount)//连续3次搜索到边界，退出      //连续搜索到边界次数需要调整  5和155需要调整
           {
               curr_col = R_edge_lost_start_col;
//              if (R_edge_count > line_amount)break;
               for (uint8_t rowi = curr_row; rowi > 0; rowi--)
               {
                   //printf("row=%d", rowi);
//                  if (black_(image[rowi - 3][curr_col]) && black_(image[rowi - 2][curr_col]) && white_(image[rowi - 1][curr_col]) && white_(image[rowi][curr_col]))
//                  if (black_2((uint8_t*)imageout[0],rowi - 3,curr_col) && black_2((uint8_t*)imageout[0],rowi - 2,curr_col) && white_2((uint8_t*)imageout[0],rowi - 1,curr_col) && white_2((uint8_t*)imageout[0],rowi,curr_col))
                   if (black_2((uint8_t*)image[0],rowi - 3,curr_col) && black_2((uint8_t*)image[0],rowi - 2,curr_col) && white_2((uint8_t*)image[0],rowi - 1,curr_col) && white_2((uint8_t*)image[0],rowi,curr_col))
                   {
                       printf("中间右丢线情况，右边线起始点：(x=%d,y=%d)\n", rowi - 1, R_edge_lost_start_col);
                       curr_row = rowi - 1;
                       curr_col = R_edge_lost_start_col;
                       center_lost_flag_r = 1;
                       center_lost_row_r = curr_row;     //中间左丢线开始行坐标
                       center_lost_col_r = curr_col + 1; //中间左丢线开始列坐标
                       dire_right = 0; //初始化上个边界点的来向
                       R_top_corner_start = i;//左上拐点开始序号
                       break;
                   }
                   if(rowi==1)turn=1;
               }
           }
       }
       else
           R_search_edge_count = 0;
       if(turn==1)break;
       //搜线过程
//      if (black_(image[curr_row + dir_front[dire_right][0]][curr_col + dir_front[dire_right][1]]))
//      if (black_2((uint8_t*)imageout[0],curr_row + dir_front[dire_right][0],curr_col + dir_front[dire_right][1]))
       if (black_2((uint8_t*)image[0],curr_row + dir_front[dire_right][0],curr_col + dir_front[dire_right][1]))
       {
           dire_right = (dire_right + 3) % 4;
           i = i - 1;
           //turn++;
       }
//      else if (black_(image[curr_row + dir_frontright[dire_right][0]][curr_col + dir_frontright[dire_right][1]]))
//      else if (black_2((uint8_t*)imageout[0],curr_row + dir_frontright[dire_right][0],curr_col + dir_frontright[dire_right][1]))
       else if (black_2((uint8_t*)image[0],curr_row + dir_frontright[dire_right][0],curr_col + dir_frontright[dire_right][1]))
       {
           curr_row += dir_front[dire_right][0];
           curr_col += dir_front[dire_right][1];
           R_edge[i].row = curr_row;
           R_edge[i].col = curr_col;
           R_edge[i].flag = 1;
           R_edge_count = R_edge_count + 1;
       }
       else
       {
           curr_row += dir_frontright[dire_right][0];
           curr_col += dir_frontright[dire_right][1];
           dire_right = (dire_right + 1) % 4;
           R_edge[i].row = curr_row;
           R_edge[i].col = curr_col;
           R_edge[i].flag = 1;
           R_edge_count = R_edge_count + 1;
       }
       //printf("(x1=%d,y1=%d)\n", L_edge[i].row, L_edge[i].col);
       //dd = dd + 1;
   }
   //printf("count%d ", L_edge_count);
}

/**
 * @brief 计算方差
 * @param mode     (uint8_t)左  or 右
 * @return NONE
 */
float statistics(uint8_t mode)
{
   uint8_t mean = 0.0;
   int32_t sum = 0;
   float rlt = 0.0;
   if (mode == 0)
   {
       for (int32_t i = 0; i < L_edge_count; i++)
       {
           sum += L_edge[i].col;
       }
       mean = sum / L_edge_count;
       for (int32_t i = 0; i < L_edge_count; i++)
       {
           rlt += (L_edge[i].col - mean) * (L_edge[i].col - mean);
       }
       rlt = rlt / L_edge_count;
       return rlt;
   }
   if (mode == 1)
   {
       for (int32_t i = 0; i < R_edge_count; i++)
       {
           sum += R_edge[i].col;
       }
       mean = sum / R_edge_count;
       for (int32_t i = 0; i < R_edge_count; i++)
       {
           rlt += (R_edge[i].col - mean) * (R_edge[i].col - mean);
       }
       rlt = rlt / R_edge_count;
       return rlt;
   }
}


/**
* @brief OTSU大津法二值化计算阈值
* @param image (uint8_t*)图像起始指针
* @param col   (uint16_t)列数
* @param row   (uint16_t)行数
* @return (uint8_t)计算得出的阈值
*/
uint8_t XLW_otsuThreshold(uint8_t* image, uint16_t col, uint16_t row)
{
   /* 定义灰度调整等级：128档 */
#define GrayScale 128
   static uint32 pixelCount[GrayScale];
   static float pixelPro[GrayScale];
   uint16_t th_max = 130, th_min;
   th_min = 0;
   //th_min = 999;
   uint16_t width = col;
   uint16_t height = row;
   uint32 i, j, pixelSum = width * height;
   uint8_t threshold_in = 0;
   uint8_t* data = &image;
   for (i = 0; i < GrayScale; i++)
   {
       pixelCount[i] = 0;
       pixelPro[i] = 0;
   }
   //统计灰度级中每个像素在整幅图像中的个数
   for (i = 28; i < height; i += 2)
   {
       for (j = 0; j < width; j += 2)
       {
           pixelCount[(int32_t)data[i * width + j] / 2]++; //将像素值作为计数数组的下标
       }
   }
   //计算每个像素在整幅图像中的比例
   float maxPro = 0.0;
   for (i = 0; i < GrayScale; i++)
   {
       pixelPro[i] = (float)pixelCount[i] / pixelSum;
       if (pixelPro[i] > maxPro)
       {
           maxPro = pixelPro[i];
       }
   }

   //遍历灰度级[0,255]
   float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
   for (i = 0; i < GrayScale; i++) // i作为阈值
   {
       w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
       for (j = 0; j < GrayScale; j++)
       {
           if (j <= i) //背景部分
           {
               w0 += pixelPro[j];
               u0tmp += j * pixelPro[j];
           }
           else //前景部分
           {
               w1 += pixelPro[j];
               u1tmp += j * pixelPro[j];
           }
       }
       u0 = u0tmp / w0;
       u1 = u1tmp / w1;
       u = u0tmp + u1tmp;
       deltaTmp = w0 * (u0 - u) * (u0 - u) + w1 * (u1 - u) * (u1 - u);
       if (deltaTmp > deltaMax)
       {
           deltaMax = deltaTmp;
           threshold_in = (uint8_t)i;
       }
   }
   threshold_in -= 1;
//  rt_kprintf("th:%d\r\n",threshold);
   if (threshold_in * 2 > th_max)
       threshold_in = th_max / 2;
   if (threshold_in * 2 < th_min)
       threshold_in = th_min / 2;
   rt_kprintf("th:%d\r\n",threshold_in);
   return threshold_in*2;
}


/*!
 * @brief    大津法求阈值大小(龙邱)
 * @param    tmImage ： 图像数据
 * @return   阈值
* @note     Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
 * @note     1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
 * @note     2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
 * @note     3) i表示分类的阈值，也即一个灰度级，从0开始迭代    1
 * @note     4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像的比例w0，并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背景像素) 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
 * @note     5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
 * @note     6) i++；转到4)，直到i为256时结束迭代
 * @note     7) 将最大g相应的i值作为图像的全局阈值
 * @note     缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
 * @note     解决光照不均匀  https://blog.csdn.net/kk55guang2/article/details/78475414
 * @note     https://blog.csdn.net/kk55guang2/article/details/78490069
 * @note     https://wenku.baidu.com/view/84e5eb271a37f111f0855b2d.html
 * @see      GetOSTU(Image_Use);//大津法阈值
 * @date     2019/6/25 星期二
 */
int32_t GetOSTU(uint8_t* tmImage)
{
   signed short i, j;
   unsigned long Amount = 0;
   unsigned long PixelBack = 0;
   unsigned long PixelIntegralBack = 0;
   unsigned long PixelIntegral = 0;
   signed long PixelIntegralFore = 0;
   signed long PixelFore = 0;
   float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
   signed short MinValue, MaxValue;
   signed short Threshold = 0;
   unsigned char HistoGram[256];              //

   for (j = 0; j < 256; j++)  HistoGram[j] = 0; //初始化灰度直方图

   for (j = 0; j < MT9V03X_H; j++)
   {
       for (i = 0; i < MT9V03X_W; i++)
       {
           HistoGram[tmImage[i*MT9V03X_W+j]]++; //统计灰度级中每个像素在整幅图像中的个数
       }
   }

   for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
   for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

   if (MaxValue == MinValue)     return MaxValue;         // 图像中只有一个颜色
   if (MinValue + 1 == MaxValue)  return MinValue;        // 图像中只有二个颜色

   for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  像素总数

   PixelIntegral = 0;
   for (j = MinValue; j <= MaxValue; j++)
   {
       PixelIntegral += HistoGram[j] * j;//灰度值总数
   }
   SigmaB = -1;
   for (j = MinValue; j < MaxValue; j++)
   {
       PixelBack = PixelBack + HistoGram[j];   //前景像素点数
       PixelFore = Amount - PixelBack;         //背景像素点数
       OmegaBack = (float)PixelBack / Amount;//前景像素百分比
       OmegaFore = (float)PixelFore / Amount;//背景像素百分比
       PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
       PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
       MicroBack = (float)PixelIntegralBack / PixelBack;   //前景灰度百分比
       MicroFore = (float)PixelIntegralFore / PixelFore;   //背景灰度百分比
       Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//计算类间方差
       if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
       {
           SigmaB = Sigma;
           Threshold = j;
       }
   }
   return Threshold;                        //返回最佳阈值;
}


/**
 * @brief    基于soble边沿检测算子的一种边沿检测
 * @param    imageIn    输入数组
 * @param    imageOut   输出数组      保存的二值化后的边沿信息
 * @param    Threshold  阈值
 * @return
 * @note
 * @date     2020/5/15
 */
void SobelThreshold(uint8_t imageIn[MT9V03X_H][MT9V03X_W], uint8_t Threshold)
{
   /** 卷积核大小 */
   int32_t KERNEL_SIZE = 3;
   int32_t xStart = KERNEL_SIZE / 2;
   int32_t xEnd = MT9V03X_W - KERNEL_SIZE / 2;
   int32_t yStart = KERNEL_SIZE / 2;
   int32_t yEnd = MT9V03X_H - KERNEL_SIZE / 2;
   int32_t i, j, k;
   int32_t temp[4];
   uint8_t imageOut[MT9V03X_H][MT9V03X_W];
   for (i = yStart; i < yEnd; i++)
   {
       for (j = xStart; j < xEnd; j++)
       {
           /* 计算不同方向梯度幅值  */
           /* 90 deg  */
           temp[0] = -(int32_t)imageIn[i - 1][j - 1] + (int32_t)imageIn[i - 1][j + 1]     // -1,  0,  1
               - (int32_t)imageIn[i][j - 1] + (int32_t)imageIn[i][j + 1]                  // -1,  0,  1
               - (int32_t)imageIn[i + 1][j - 1] + (int32_t)imageIn[i + 1][j + 1];         // -1,  0,  1
           /* 0 deg  */
           temp[1] = -(int32_t)imageIn[i - 1][j - 1] + (int32_t)imageIn[i + 1][j - 1]     // -1, -1, -1
               - (int32_t)imageIn[i - 1][j] + (int32_t)imageIn[i + 1][j]                  //  0,  0,  0
               - (int32_t)imageIn[i - 1][j + 1] + (int32_t)imageIn[i + 1][j + 1];         //  1,  1,  1

           /* 45 deg  */
           temp[2] = -(int32_t)imageIn[i - 1][j] + (int32_t)imageIn[i][j - 1]             //  0, -1, -1
               - (int32_t)imageIn[i][j + 1] + (int32_t)imageIn[i + 1][j]                  //  1,  0, -1
               - (int32_t)imageIn[i - 1][j + 1] + (int32_t)imageIn[i + 1][j - 1];         //  1,  1,  0
           /* 135 deg  */
           temp[3] = -(int32_t)imageIn[i - 1][j] + (int32_t)imageIn[i][j + 1]             // -1, -1,  0
               - (int32_t)imageIn[i][j - 1] + (int32_t)imageIn[i + 1][j]                  // -1,  0,  1
               - (int32_t)imageIn[i - 1][j - 1] + (int32_t)imageIn[i + 1][j + 1];         //  0,  1,  1

           temp[0] = abs(temp[0]);
           temp[1] = abs(temp[1]);
           temp[2] = abs(temp[2]);
           temp[3] = abs(temp[3]);

           /* 找出梯度幅值最大值  */
           for (k = 1; k < 4; k++)
           {
               if (temp[0] < temp[k])
               {
                   temp[0] = temp[k];
               }
           }

           if (temp[0] > Threshold)
           {
               imageOut[i][j] = 255;
           }
           else
           {
               imageOut[i][j] = 1;
           }
       }
   }
   for (int32_t i = 0; i < MT9V03X_H; i++)
   {
       for (int32_t j = 0; j < MT9V03X_W; j++)
       {
           imageIn[i][j] = imageOut[i][j];
       }
   }
}

/**
 * @brief    基于soble边沿检测算子的一种自动阈值边沿检测
 * @param    imageIn    输入数组
 * @param    imageOut   输出数组      保存的二值化后的边沿信息
 * @return
 * @note
 * @date     2020/5/15
 */
void SobelAutoThreshold(uint8_t* image,uint8_t* imageout)
{
   /** 卷积核大小 */
   int32_t KERNEL_SIZE = 3;
   int32_t xStart = KERNEL_SIZE / 2;
   int32_t xEnd = MT9V03X_W - KERNEL_SIZE / 2;
   int32_t yStart = KERNEL_SIZE / 2;
   int32_t yEnd = MT9V03X_H - KERNEL_SIZE / 2;
   int32_t row, col, k;
   int32_t temp[4];
//  uint8_t imageOut[MT9V03X_H][MT9V03X_W];
   for (row = yStart; row < yEnd; row++)
   {
       for (col = xStart; col < xEnd; col++)
       {
           /* 计算不同方向梯度幅值  */
           /* 90 deg  */
           temp[0] = -(int32_t)image[(row - 1)*MT9V03X_W + (col - 1)] + (int32_t)image[(row - 1)*MT9V03X_W + (col + 1)]     // -1,  0,  1
               - (int32_t)image[(row)*MT9V03X_W + (col - 1)] + (int32_t)image[(row)*MT9V03X_W + (col + 1)]                  // -1,  0,  1
               - (int32_t)image[(row + 1)*MT9V03X_W + (col - 1)] + (int32_t)image[(row + 1)*MT9V03X_W + (col + 1)];         // -1,  0,  1
           /* 0 deg  */
           temp[1] = -(int32_t)image[(row - 1)*MT9V03X_W + col - 1] + (int32_t)image[(row + 1)*MT9V03X_W + col - 1]     // -1, -1, -1
               - (int32_t)image[(row - 1)*MT9V03X_W + col] + (int32_t)image[(row + 1)*MT9V03X_W + col]                  //  0,  0,  0
               - (int32_t)image[(row - 1)*MT9V03X_W + col + 1] + (int32_t)image[(row + 1)*MT9V03X_W + col + 1];         //  1,  1,  1

           /* 45 deg  */
           temp[2] = -(int32_t)image[(row - 1)*MT9V03X_W + col] + (int32_t)image[(row)*MT9V03X_W + col - 1]             //  0, -1, -1
               - (int32_t)image[(row)*MT9V03X_W + col + 1] + (int32_t)image[(row + 1)*MT9V03X_W + col]                  //  1,  0, -1
               - (int32_t)image[(row - 1)*MT9V03X_W + col + 1] + (int32_t)image[(row + 1)*MT9V03X_W + col - 1];         //  1,  1,  0
           /* 135 deg  */
           temp[3] = -(int32_t)image[(row - 1)*MT9V03X_W + col] + (int32_t)image[(row)*MT9V03X_W + col + 1]             // -1, -1,  0
               - (int32_t)image[(row)*MT9V03X_W + col - 1] + (int32_t)image[(row + 1)*MT9V03X_W + col]                  // -1,  0,  1
               - (int32_t)image[(row - 1)*MT9V03X_W + col - 1] + (int32_t)image[(row + 1)*MT9V03X_W + col + 1];         //  0,  1,  1

           temp[0] = abs(temp[0]);
           temp[1] = abs(temp[1]);
           temp[2] = abs(temp[2]);
           temp[3] = abs(temp[3]);

           /* 找出梯度幅值最大值  */
           for (int16_t k = 1; k < 4; k++)
           {
               if (temp[0] < temp[k])
               {
                   temp[0] = temp[k];
               }
           }

           /* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
           temp[3] = (int32_t)image[(row - 1)*MT9V03X_W + col - 1] + (int32_t)image[(row - 1)*MT9V03X_W + col] + (int32_t)image[(row - 1)*MT9V03X_W + col + 1]
                   + (int32_t)image[(row)*MT9V03X_W + col - 1] + (int32_t)image[(row)*MT9V03X_W + col] + (int32_t)image[(row)*MT9V03X_W + col + 1]
                   + (int32_t)image[(row + 1)*MT9V03X_W + col - 1] + (int32_t)image[(row + 1)*MT9V03X_W + col] + (int32_t)image[(row + 1)*MT9V03X_W + col + 1];

           if (temp[0] > temp[3] / 10.0f)
           {
               imageout[(row)*MT9V03X_W + col] = 0;
           }
           else
           {
               imageout[(row)*MT9V03X_W + col] = 1;
           }
       }
   }
}

/**********************************************************
** 函数名:u32tostr
** 功能描述: 将一个32位的变量dat转为字符串，比如把1234转为"1234"
** 输入参数: dat:带转的long型的变量
             str:指向字符数组的指针，转换后的字节串放在其中
** 输出参数: 无
***********************************************************/
void u32tostr(unsigned long dat,char *str)
{
    char temp[20];
    unsigned char i=0,j=0;
    i=0;
    while(dat)
    {
        temp[i]=dat%10+0x30;
        i++;
        dat/=10;
    }
    j=i;
    for(i=0;i<j;i++)
    {
          str[i]=temp[j-i-1];
    }
    if(!i) {str[i++]='0';}
    str[i]=0;
}

void tobin(uint32_t a,char* str)
{
  char *p=(char*)&a,c=0,f=0,pos=-1;//p指向a的首地址
  for(int o=0;o<4;++o){
    for(int i=0;i<8;++i){
      c=p[3-o]&(1<<(7-i));
      if(!f&&!(f=c))continue;
      str[++pos]=c?'1':'0';
    }
  }
}


int clip(int x, int low, int up) {
    return x > up ? up : x < low ? low : x;
}

float fclip(float x, float low, float up) {
    return x > up ? up : x < low ? low : x;
}

