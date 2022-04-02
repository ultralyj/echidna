/*
 * nosaydie_camera_bianliang.h
 *
 *  Created on: 2021年11月11日
 *      Author: Administrator
 */

#ifndef _TJRC_IMAGEPROC_VALUE_H_
#define _TJRC_IMAGEPROC_VALUE_H_
#include "common.h"
#include "tjrc_imageProc.h"
//#define LCDW 120
// //相关变量
////gui相关  #全局 gui 使用
//uint8_t value3[30];
////  发车/停车标志  #全局 未使用
//uint8_t run_flag = 0;
//uint8_t stop_flag = 0;
////  图像处理：一般处理
////      使能相关变量  #全局 使用
//uint8_t enable_balinyu = 1;                                   //八邻域爬线使能标志，默认使能
//uint8_t enable_midline = 1;                                   //使能中线拟合，默认不开启
//uint8_t enable_midpross = 1;                                  //使能中线处理，默认开启
//uint8_t enable_check_l_r_edge_same = 0;                       //使能左、右边线是否重合，默认不开启
////      搜线限制相关  #全局 控制 使用
//uint8_t L_edge_end_row = 5;                                   //左边界行结束点
//uint8_t R_edge_end_row = 5;                                   //右
//uint8_t M_edge_end_row = 5;                                   //中
//uint8_t min_col = 3, max_col = LCDW - 3;        //搜索结束列值
//uint8_t L_edge_start_col = 3;         //左边界列搜索起始点
//uint8_t R_edge_start_col = 117;       //右边界列搜索起始点
//
//// #全局 flag 使用
//uint8_t L_start_lost = 0;//左边线起始丢线标志
//uint8_t R_start_lost = 0;//右边线起始丢线标志
////      搜线计数相关  #全局 控制 使用
//uint8_t Image_H = MT9V03X_H;
//uint8_t Image_W = MT9V03X_W;
//uint8_t left_findflag, right_findflag;
//uint8_t L_basic_row_start = MT9V03X_H - 1;                      //左边线搜线开始点
//uint8_t R_basic_row_start = MT9V03X_H - 2;                      //右边线搜线开始点
//uint8_t L_search_edge_count = 0, R_search_edge_count = 0;     //搜索到边界
//uint8_t line_point_count_left, line_point_count_right;        //左右线有效点计数
//uint8_t L_edge_count = 0, R_edge_count = 0;                     //左右边点的个数
//uint8_t Mid_count = 0;                                          //中线点的个数
//uint8_t center_arry_count;
//uint8_t line_lose_center_left;
//uint8_t line_lose_center_right;
//uint8_t dire_left;                                            //记录上一个点的相对位置
//uint8_t dire_right;                                           //记录上一个点的相对位置
//uint8_t center_turn_flag;                                     //0 左趋势， 1 右趋势
//uint8_t center_biaoxiang_arry[5];
//
//// #全局 flag 使用
//uint8_t center_lost_flag_l = 0;//中间左丢线标志位;
//uint8_t center_lost_row_l = 0;//中间左丢线开始行坐标
//uint8_t center_lost_col_l = 0;//中间左丢线开始列坐标
//uint8_t center_lost_flag_r = 0;//中间右丢线标志位
//uint8_t center_lost_row_r = 0;//中间右丢线开始行坐标
//uint8_t center_lost_col_r = 0;//中间右丢线开始列坐标
//uint8_t L_edge_lost_start_col = 4;//中间左丢线列搜索起始点
//uint8_t R_edge_lost_start_col = 115;//中间右丢线列搜索起始点
//
//// #全局 控制 使用
//uint8_t L_lost_count = 0;//左丢线计数
//uint8_t L_lost_ = 10;//左丢线限制次数
//uint8_t R_lost_count = 0;//右丢线计数
//uint8_t R_lost_ = 10;//右丢线限制次数
////      边界预处理 #全局 使用
//uint8_t edge_process_flag = 0;
//uint8_t pre_L_edge_count = 0;
//uint8_t pre_R_edge_count = 0;
//uint8_t num_cnt = 0;//记录连续水平点的个数
//uint8_t L_count = 0;
//uint8_t R_count = 0;
//float line_blur_kernel = 7;  //边线三角滤波核的大小
////          拐点处理  #全局 使用
//uint8_t enable_L_corner = 1;//左拐点搜索使能标志 默认使能
//uint8_t enable_R_corner = 1;//右拐点搜索是能标志 默认使能
//uint8_t L_corner_flag = 0;//左拐点存在标志
//uint8_t L_corner_row = 0;//左拐点所在行
//uint8_t L_corner_col = 0;//左拐点所在列
//int L_corner_angle = 0;//左拐点角度
//uint8_t R_corner_flag = 0;//右拐点存在标志
//uint8_t R_corner_row = 0;//右拐点所在行
//uint8_t R_corner_col = 0;//右拐点所在列
//int R_corner_angle = 0;//右拐点角度
//
//// #全局 使用
//uint8_t L_top_corner_start = 0;//左上拐点开始序号
//uint8_t R_top_corner_start = 0;//右上拐点开始序号
//
//// #全局 使用
//uint8_t L_top_i = 0;//左上拐角对应i
//uint8_t L_beh_i = 0;//左下拐角对应i
//uint8_t R_top_i = 0;//右上拐角对应i
//uint8_t R_beh_i = 0;//右下拐角对应i
//
//// #全局 控制 使用
//uint8_t dist = 4;//求拐点角度的点数间隔
//
////      中线相关  #全局 使用
//uint8_t center_lost_corner_row_l = 0;//中间左丢线开始拐点行坐标
//uint8_t center_lost_corner_col_l = 0;//中间左丢线开始拐点列坐标
//int L_center_lost_corner_angle = 0;//中间左丢线开始左拐点角度
//uint8_t center_lost_corner_row_r = 0;//中间右丢线开始拐点行坐标
//uint8_t center_lost_corner_col_r = 0;//中间右丢线开始拐点列坐标
//int R_center_lost_corner_angle = 0;//中间右丢线开始左拐点角度
//
//// 拟合斜率  #全局 使用
//float k_l = 0;
//float b_l = 0;
//
//float k_r = 0;
//float b_r = 0;
//
////方差  #全局 使用
//float statistics1 = 0;

//质量矩，赛道宽度(后期修改)
uint8_t Road_Width[LCDH] = { 113,112,111,110,109,108,107,106,105,104,103,102,101,100,99 ,98 ,97 ,96 ,95 ,94 ,
						93 ,92 ,91 ,90 , 89, 88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74,
						73 ,72 ,71 ,70 ,69 , 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54,
						53 ,52 , 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34 };
//  元素识别
//      环岛
uint8_t HuanDao_FLAG = 0; //进入环岛，默认为0
//      车库
uint8_t In_CheKu_flag = 0;//入库标志，默认为0



#endif /* CODE_NOSAYDIE_CAMERA_BIANLIANG_H_ */
