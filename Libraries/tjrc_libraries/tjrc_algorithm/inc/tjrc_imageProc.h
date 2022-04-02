#ifndef CODE_NOSAYDIE_CAMERA_H_
#define CODE_NOSAYDIE_CAMERA_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "rtthread.h"
#include "tjrc_mt9v03x.h"


//宏定义相关变量-------------------------------
#define LCDW 120
#define LCDH 80
//配置摄像头参数
//自定义结构体及相关变量

typedef struct
{
    //###控制参数列表
    uint8_t L_basic_row_start;  //左边线行搜线开始点
    uint8_t R_basic_row_start;  //右边线行搜线开始点
    uint8_t L_edge_start_col;   //左边界列搜索起始点
    uint8_t R_edge_start_col;   //右边界列搜索起始点
    uint8_t L_search_amount;    //左右边界搜点时最多允许的点
    uint8_t R_search_amount;    //左右边界搜点时最多允许的点
    uint8_t min_col;  //最小列坐标，即边界值
    uint8_t max_col;  //最大列坐标，即边界值
    uint8_t L_lost_;  //左丢线限制次数
    uint8_t R_lost_;  //右丢线限制次数
    uint8_t L_edge_lost_start_col; //中间左丢线列搜索起始点
    uint8_t R_edge_lost_start_col; //中间右丢线列搜索起始点
    uint8_t dist;  //求拐点角度的点数间隔
    uint8_t effective_points; //用于防止单段边线长度过长导致拐点误判 #全局 控制 使用
    float dist_control;//判断是否为直线的控制参数  #全局 控制 使用

    //###标志位参数列表
    uint8_t enable_bin;   //使能二值化标志位
    uint8_t enable_balinyu; //使能八邻域搜线标志位
    uint8_t enable_turnpoint;  //使能拐点搜索标准位
    uint8_t enable_element;  //使能识别元素标志位

    //uint8_t L_start_lost;//左边线起始丢线标志
    //uint8_t R_start_lost;//右边线起始丢线标志
    //bool dot_left_bottom_l;   //左下拐点，趋势左
    //bool dot_left_bottom_r;   //左下拐点，趋势右
    //bool dot_left_top_l;      //左上拐点，趋势左
    //bool dot_left_top_r;      //左上拐点，趋势右
    //bool dot_right_bottom_l;  //右下拐点，趋势左
    //bool dot_right_bottom_r;  //右下拐点，趋势右
    //bool dot_right_top_l;     //右上拐点，趋势左
    //bool dot_right_top_r;     //右上拐点，趋势右
    ////是否为直线标志 #全局 flag 使用
    //bool is_line_left = 0;
    //bool is_line_right = 0;

}tjrc_image_info;

struct LEFT_EDGE
{
	uint8_t row;  //行坐标
	uint8_t col;  //列坐标
	int32_t flag; //存在边界的标志
};
struct RIGHT_EDGE
{
	uint8_t row;  //行坐标
	uint8_t col;  //列坐标
	int32_t flag; //存在边界的标志
};
//自定义函数-----------------------------------
//int32_t average_value(uint8_t* c);//求均值（均值滤波用）
//void average_filter(void);  //均值滤波

uint8_t black_(uint8_t x);  //判断是否是黑像素点
uint8_t white_(uint8_t x);  //判断是否是白像素点
int32_t edge_point_ornot(uint8_t image[MT9V03X_H][MT9V03X_W],uint8_t row, uint8_t side);//判断是否存在边界点,并返回【-1：没有找到边界点，正值：返回找到的边界点 】
int32_t Get_angle(uint8_t ax, uint8_t ay, uint8_t bx, uint8_t by, uint8_t cx, uint8_t cy);//求取拐角的角度值
void get_mid();         //拟合中线
uint8_t* Image_process(uint8_t imagein[LCDH][LCDW], uint8_t sidee[150 * 2 * 3]);
void blur_points(struct LEFT_EDGE pts_in[120], struct LEFT_EDGE pts_out[120], int32_t num, int32_t kernel);
int32_t clip(int32_t x, int32_t low, int32_t up);
float fclip(float x, float low, float up);
void findline_lefthand_adaptive(uint8_t image[MT9V03X_H][MT9V03X_W],int32_t jilu_row, int32_t jilu_col, int32_t search_amount);
void findline_righthand_adaptive(uint8_t image[MT9V03X_H][MT9V03X_W],int32_t jilu_row, int32_t jilu_col, int32_t search_amount);
void edge_start();
void edge_truncation(uint8_t side);
void FitStraightLine(int32_t start, int32_t end, uint8_t side);
//void git_gui_value(uint8_t value_in[30]);
float statistics(uint8_t mode);
void tjrc_imageProcess(uint8_t* image);


/* 二值化函数组 */
uint8_t tjrc_binarization_otsu(const uint8_t* image, uint16_t col, uint16_t row);
uint8_t tjrc_binarization_avg(const uint8_t* image, uint16_t col, uint16_t row);
void tjrc_binarization_getBinImage(uint8_t threshold,const uint8_t* image_in, uint8_t* image_out, uint16_t width, uint16_t height);
void tjrc_sobel_autoThreshold(const uint8_t* imageIn, uint8_t* imageOut,uint16_t width,uint16_t height);
//int32_t GetOSTU(uint8_t* tmImage);
uint8_t OSTU_bin(uint8_t width, uint8_t height, uint8_t* Image);//大津法求动态阈值
uint8_t GetOSTUThreshold(uint8_t(*img)[LCDW], uint16_t start_row, uint16_t end_row, uint16_t start_col, uint16_t end_col); //大津法求动态阈值
void get_binImage(uint8_t thres);//二值化

#endif /* CODE_NOSAYDIE_CAMERA_H_ */

