
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
/*!
  * @file     imageprocess.cpp
  * @brief    图像处理
  * @author   YYY
  * @version  V1.2
  * @date     2022/2/7
  */


#include "tjrc_imageProc.h"
#include "common.h"
#include "tjrc_imageProc_value.h"
  //声明全局变量-------------------------------------------
  //  图像
  //      图像预处理
uint8_t threshold;                  //二值化阈值
uint8_t mt9v03x_image[MT9V03X_H][MT9V03X_W];
uint8_t image[MT9V03X_H][MT9V03X_W];        //使用二维指针image代替信息图像，便于后期二值化图像和深度图像的转换
uint8_t binimage_flag = 1;          //二值化标志，默认没有二值化
struct LEFT_EDGE L_edge[150];     //左边界结构体  #全局 使用
struct LEFT_EDGE L_edge2[150];    // #全局 未使用
struct LEFT_EDGE R_edge[150];    //右边界结构体  #全局 使用
struct LEFT_EDGE M_Line[150];     //中线结构体，备用选择2
struct LEFT_EDGE Last_M_Line[150];//上次中线结构体
struct LEFT_EDGE MID_LINE[150];   //中线结构体，备用选择1

uint8_t effective_points = 65; //用于防止单段边线长度过长导致拐点误判 #全局 控制 使用

struct LEFT_EDGE L_edge_use[80];     //左边界结构体 最后使用  #全局 使用
struct LEFT_EDGE R_edge_use[80];    //右边界结构体 最后使用  #全局 使用

struct LEFT_EDGE* Mid_Line = M_Line;       //中线结构体【实际使用】
//      用于中线处理
uint8_t searchpoint = 1;            //正常求出来的中线点标志，用于结构体.flag，便于后期处理
uint8_t nihepoint = 2;              //拟合求出来的中线点标志，用于结构体.flag，便于后期处理
extern uint8_t Road_Width[MT9V03X_H];//质量矩，用于丢线
//      图像显示
uint8_t l_display_cnt = 0;
uint8_t r_display_cnt = 0;

//直线斜率数据存放数组  #全局 使用
float line_left_bottom[2];
float line_left_top[2];
float line_right_top[2];
float line_right_bottom[2];

//拐点标志 #全局 flag 使用
bool dot_left_bottom_l = 0;   //左下拐点，趋势左
bool dot_left_bottom_r = 0;   //左下拐点，趋势右
bool dot_left_top_l = 0;      //左上拐点，趋势左
bool dot_left_top_r = 0;      //左上拐点，趋势右
bool dot_right_bottom_l = 0;  //右下拐点，趋势左
bool dot_right_bottom_r = 0;  //右下拐点，趋势右
bool dot_right_top_l = 0;     //右上拐点，趋势左
bool dot_right_top_r = 0;     //右上拐点，趋势右

//是否为直线标志 #全局 flag 使用
bool is_line_left = 0;
bool is_line_right = 0;

//直线总体偏差 #全局 使用
float dist_control = 180;//判断是否为直线的控制参数  #全局 控制 使用
float dist_sum_left_bottom = 0;
float dist_sum_left_top = 0;
float dist_sum_right_bottom = 0;
float dist_sum_right_top = 0;

/*--------------------------------------------------------------------------
* 【函数功能】：传递gui参数
* 【参    数】：无
* 【返 回 值】：无
*--------------------------------------------------------------------------*/
void git_gui_value(uint8_t value_in[30])
{
	for (int32_t i = 0; i < 30; i++)
	{
		value3[i] = value_in[i];
	}
}
/*--------------------------------------------------------------------------
* 【函数功能】：图像处理（中线、元素识别）_八邻域容易越界
* 【参    数】：无
* 【返 回 值】：无
*--------------------------------------------------------------------------*/
int32_t x1, x2, x3, x4;


////###全局存放参数列表
//struct LEFT_EDGE L_edge[150];     //左边界结构体  #全局 使用
//struct LEFT_EDGE R_edge[150];    //右边界结构体  #全局 使用
//struct LEFT_EDGE Mid_Line[80];       //中线结构体【实际使用】
//struct LEFT_EDGE L_edge_use[80];     //左边界结构体 最后使用  #全局 使用
//struct LEFT_EDGE R_edge_use[80];    //右边界结构体 最后使用  #全局 使用
////直线斜率数据存放数组  #全局 使用
//float line_left_bottom[2];
//float line_left_top[2];
//float line_right_top[2];
//float line_right_bottom[2];
////直线总体偏差 #全局 使用
//float dist_sum_left_bottom = 0;
//float dist_sum_left_top = 0;
//float dist_sum_right_bottom = 0;
//float dist_sum_right_top = 0;



const tjrc_image_info tjrc_imageProc_ctr = {
	//###控制参数列表
	78,    //左边线行搜线开始点
	78,    //右边线行搜线开始点
	2,     //左边界列搜索起始点
	117,   //右边界列搜索起始点
	150,   //左右边界搜点时最多允许的点
	150,   //左右边界搜点时最多允许的点
	3,     //最小列坐标，即边界值
	117,   //最大列坐标，即边界值
	10,    //左丢线限制次数
	10,    //右丢线限制次数
	4,     //中间左丢线列搜索起始点
	115,   //中间右丢线列搜索起始点
	4,     //求拐点角度的点数间隔
	65,    //用于防止单段边线长度过长导致拐点误判 #全局 控制 使用
	180,   //判断是否为直线的控制参数  #全局 控制 使用

	//###标志位参数列表
	0,     //使能二值化标志位
	0,     //使能八邻域搜线标志位
	0,     //使能拐点搜索标准位
	0      //使能识别元素标志位
};



/*!
  * @file       tjrc_imageProc.cpp
  * @brief      图像处理
  * @param      imagein[LCDH][LCDW]:输入图像数组
			    info:输入控制参数结构体
  * @relative	XLW_otsuThreshold()
				GetOSTU()
				SobelThreshold()
				SobelAutoThreshold()
				Get_01_Value()
				get_binImage()
				edge_point_ornot()
				findline_lefthand_adaptive()
				findline_righthand_adaptive()
				clear_point()
				black_()
				white_()
				FitStraightLine()
  * @return		none   
  * @author		YYY
  * @version	V1.0
  * @date		2022/3/27
  */
void tjrc_imageProc(uint8_t imagein[LCDH][LCDW], tjrc_image_info* info)
{
	//变量相关
	//  全局、外部变量--------------------------
	//      全局变量
	//      外部变量
	//  函数内部变量-------------------
	//      一般图像处理
	uint8_t Bottom2 = Image_H - 40;   //倒数第二行
	uint8_t max_col = Image_W - 5, min_col = 5;         //最大/小列坐标，即边界值  #内部 控制 使用
	uint8_t L_search_amount = 150, R_search_amount = 150;  //左右边界搜点时最多允许的点  #内部 控制 使用
	uint8_t jilu_row_l = 0, jilu_col_l = 0, jilu_row_r = 0, jilu_col_r = 0;  //记录搜索到的基础边界点行列值   #内部 使用
   //图像处理-----------------------
   //  一般图像处理
   //      初始化相关变量
	enable_check_l_r_edge_same = 0;    //使能检查做右边线是否爬重合，默认不开启，当爬线起始点过高时开启
	left_findflag = 0;                 //左边界存在标志，1找到左边界，0没找到左边界        默认没有找到左边界
	right_findflag = 0;                //右边界存在标志，1找到有边界，0没找到右边界        默认没有找到右边界

	// #全局 控制 使用
	L_basic_row_start = Image_H - 2;    //左开始搜线点
	R_basic_row_start = Image_H - 2;    //右开始搜线点
   //      获取图像二值化阈值和图像二值化
	//get_deal_image();

	/* 获取gui传递参数 */
	for (int32_t i = 0; i < LCDH; i++)
	{
		for (int32_t j = 0; j < LCDW; j++)
		{
			image[i][j] = imagein[i][j];
			//printf("%d", image[i][j]);
		}
	}

	/* 获取gui传递参数 */
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

	clear_point();//边界初始化

	//变量初始化
	for (int32_t i = 0; i < 80; i++)
	{
		L_edge_use[i].row = 0;
		L_edge_use[i].col = 0;
		L_edge_use[i].flag = 0;
		R_edge_use[i].row = 0;
		R_edge_use[i].col = 0;
		R_edge_use[i].flag = 0;
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
	line_left_bottom[2] = { 0 };
	line_left_top[2] = { 0 };
	line_right_top[2] = { 0 };
	line_right_bottom[2] = { 0 };

	//变量初始化
	is_line_left = 0;
	is_line_right = 0;

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

	// 二值化
	if (tjrc_imageProc_ctr.enable_bin)
	{
		uint8_t threshold = 0;
		//2574行之后有四种二值化方法，暂时没有在该代码中调用，在gui中可用
		threshold = XLW_otsuThreshold(imagein, 120, 80);
		//average_filter();
		//threshold = OSTU_bin(Image_W,Image_H,mt9v03x_image);//获取动态阈值
		//threshold = GetOSTUThreshold(image, 0, Image_H - 1, 0, Image_W - 1);
		get_binImage(threshold);//获取二值化图像
	}
	

	// 开始爬边线
	if (tjrc_imageProc_ctr.enable_balinyu)//如果使能八邻域爬线即一般处理
	{
		//相关变量
		line_lose_center_left = 0;
		line_lose_center_right = 0;
		line_point_count_left = 0;
		line_point_count_right = 0;
		L_edge_count = 0;//左边点个数清0
		R_edge_count = 0;//右边点个数清0
		int32_t exist_edge_size = 0;   //判断是否存在左/右边界

		/* 寻找搜线起始点 */
		//寻找左/右线开始点，并判断是否存在当前边
		clear_point();
		exist_edge_size = edge_point_ornot(L_basic_row_start, 0);
		if (exist_edge_size >= 0) { jilu_row_l = L_basic_row_start; jilu_col_l = exist_edge_size; left_findflag = 1; }
		//printf("左边线起始点：(x=%d,y=%d)\n", jilu_row_l, jilu_col_l);

		exist_edge_size = edge_point_ornot(R_basic_row_start, (uint8_t)(1));
		if (exist_edge_size >= 0) { jilu_row_r = R_basic_row_start; jilu_col_r = exist_edge_size; right_findflag = 1; }
		//printf("右边线起始点：(x=%d,y=%d)\n", jilu_row_r, jilu_col_r);

		/* 进行八邻域搜线  */
		if (left_findflag)//如果左边界点存在并找到,则开始爬线
		{
			//变量声明
			L_edge[0].row = jilu_row_l;
			L_edge[0].col = jilu_col_l;
			L_edge[0].flag = 1;
			uint8_t curr_row = jilu_row_l;//初始化行坐标
			uint8_t curr_col = jilu_col_l;//初始化列坐标
			dire_left = 0; //初始化上个边界点的来向
			center_turn_flag = 1;//初始化在未标定状态
			center_lost_flag_l = 0;//中间左丢线标志位
			//开始搜线，最多取150个点，不会往下搜，共7个方位
			findline_lefthand_adaptive(jilu_row_l, jilu_col_l, L_search_amount);
			
		}
		if (right_findflag)//如果右边界存在并搜到
		{
			R_edge[0].row = jilu_row_r;
			R_edge[0].col = jilu_col_r;
			R_edge[0].flag = 1;
			uint8_t curr_row = jilu_row_r;
			uint8_t curr_col = jilu_col_r;
			dire_right = 0;
			center_lost_flag_r = 0; //中间右丢线标志位
			findline_righthand_adaptive(jilu_row_r, jilu_col_r, R_search_amount);
			
		}
		
		//检测拐点,判断边线的左右趋势
		/* 寻找上下拐点 */
		if (tjrc_imageProc_ctr.enable_turnpoint)
		{
			L_corner_flag = 0;// 初始化变量
			L_corner_row = 0;
			L_corner_col = 0;
			L_corner_angle = 0;
			if (!L_start_lost && !center_lost_flag_l)//只有左下边线情况
			{
				uint8_t m3 = 0;
				if (L_edge_count > effective_points)L_edge_count = effective_points;
				else L_edge_count = L_edge_count;
				//直线顶点坐标(L_edge[0].col,L_edge[0].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
				printf("\n直线顶点为：(%d,%d),(%d,%d)", L_edge[0].row, L_edge[0].col, L_edge[L_edge_count - 1].row, L_edge[L_edge_count - 1].col);
				line_left_bottom[0] = (float)(L_edge[L_edge_count - 1].col - L_edge[0].col) / (float)(L_edge[L_edge_count - 1].row - L_edge[0].row);   //k
				line_left_bottom[1] = (float)(L_edge[0].col - line_left_bottom[0] * L_edge[0].row);  //b
				printf("  直线斜率：k=%f,b=%f\n", line_left_bottom[0], line_left_bottom[1]);																							 //显示该直线
				uint8_t m1, m2;
				if (L_edge[0].row > L_edge[L_edge_count - 1].row)
				{
					m1 = L_edge[L_edge_count - 1].row;
					m2 = L_edge[0].row;
				}
				else
				{
					m2 = L_edge[L_edge_count - 1].row;
					m1 = L_edge[0].row;
				}
				for (int32_t i = m1; i < m2; i++)
				{
					L_edge_use[i].row = i;
					L_edge_use[i].col = (int32_t)(i * line_left_bottom[0] + line_left_bottom[1]);
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
				is_line_left = 0;
				for (int32_t i = 0; i < L_edge_count; i++)
				{
					distance = (float)(line_left_bottom[0] * L_edge[i].row - L_edge[i].col + line_left_bottom[1]) / sqrt(line_left_bottom[0] * line_left_bottom[0] + 1);
					//printf("%f\n", distance);
					if (distance > max)
					{
						max = distance;
						L_corner_row = L_edge[i].row;
						L_corner_col = L_edge[i].col;
						L_beh_i = i;
					}
					//printf("%f\n", distance);
					dist_sum_left_bottom += distance;//求总体差值判断是否为直线
				}
				L_corner_angle = max;
				if (dist_sum_left_bottom < 0)dot_left_bottom_l = 1;//左拐线
				else dot_left_bottom_r = 1;//右拐线
				if ((dist_sum_left_bottom * (dist_sum_left_bottom < 0 ? -1 : 1)) < dist_control)is_line_left = 1;

				if (dot_left_bottom_l)printf("turn left\n");
				if (dot_left_bottom_r)printf("turn right\n");
				printf("distance bottom left:%f\n", dist_sum_left_bottom);
			}
			else if (L_start_lost)//只有左上边线情况
			{
				uint8_t m3 = 0;
				if (L_edge_count > effective_points)L_edge_count = effective_points;
				else L_edge_count = L_edge_count;
				//直线顶点坐标(L_edge[1].col,L_edge[1].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
				printf("\n直线顶点为：(%d,%d),(%d,%d)", L_edge[1].row, L_edge[1].col, L_edge[L_edge_count - 1].row, L_edge[L_edge_count - 1].col);
				line_left_top[0] = (float)(L_edge[L_edge_count - 1].col - L_edge[1].col) / (float)(L_edge[L_edge_count - 1].row - L_edge[1].row);   //k
				line_left_top[1] = (float)(L_edge[1].col - line_left_top[0] * L_edge[1].row);  //b
				printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
				//显示该直线
				uint8_t m1, m2;
				if (L_edge[1].row > L_edge[L_edge_count - 1].row)
				{
					m1 = L_edge[L_edge_count - 1].row;
					m2 = L_edge[1].row;
				}
				else
				{
					m2 = L_edge[L_edge_count - 1].row;
					m1 = L_edge[1].row;
				}
				//printf("  m1=%d,m2=%d\n", m1, m2);
				for (int32_t i = m1; i < m2; i++)
				{
					L_edge_use[i].row = i;
					L_edge_use[i].col = (int32_t)(i * line_left_top[0] + line_left_top[1]);
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
				is_line_left = 0;
				for (int32_t i = 1; i < L_edge_count; i++)
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
				L_center_lost_corner_angle = max;
				if (dist_sum_left_top < 0)dot_left_top_l = 1;//左拐线
				else dot_left_top_r = 1;//右拐线
				if ((dist_sum_left_top * (dist_sum_left_top < 0 ? -1 : 1)) < dist_control)is_line_left = 1;

				if (dot_left_top_l)printf("turn left\n");
				if (dot_left_top_r)printf("turn right\n");
				printf("distance top left:%f\n", dist_sum_left_top);
			}
			else if (center_lost_flag_l)//有左上和左下边线情况   #TODO  增加对distance正负的判断
			{
				//左下
				uint8_t m3 = L_top_corner_start;
				if (L_top_corner_start > effective_points)L_top_corner_start = effective_points;
				else L_top_corner_start = L_top_corner_start;
				//直线顶点坐标(L_edge[0].col,L_edge[0].row),(L_edge[L_top_corner_start-1].col,L_edge[L_top_corner_start-1].row)
				printf("\n直线顶点为：(%d,%d),(%d,%d)", L_edge[0].row, L_edge[0].col, L_edge[L_top_corner_start - 1].row, L_edge[L_top_corner_start - 1].col);
				line_left_bottom[0] = (float)(L_edge[L_top_corner_start - 1].col - L_edge[0].col) / (float)(L_edge[L_top_corner_start - 1].row - L_edge[0].row);   //k
				line_left_bottom[1] = (float)(L_edge[0].col - line_left_bottom[0] * L_edge[0].row);  //b
				//显示该直线
				uint8_t m1, m2;
				if (L_edge[0].row > L_edge[L_top_corner_start - 1].row)
				{
					m1 = L_edge[L_top_corner_start - 1].row;
					m2 = L_edge[0].row;
				}
				else
				{
					m2 = L_edge[L_top_corner_start - 1].row;
					m1 = L_edge[0].row;
				}
				printf("m1m2%d,%d\n", m1, m2);
				for (int32_t i = m1; i < m2; i++)
				{
					L_edge_use[i].row = i;
					L_edge_use[i].col = (int32_t)(i * line_left_bottom[0] + line_left_bottom[1]);
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
				float max = 0;
				float distance = 0;
				//uint8_t m3 = 0;
				dist_sum_left_bottom = 0;
				dot_left_bottom_l = 0;
				dot_left_bottom_r = 0;

				for (int32_t i = 0; i < L_top_corner_start; i++)
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
					dist_sum_left_bottom += distance;//求总体差值判断是否为直线
				}
				L_corner_angle = max;
				if (dist_sum_left_bottom < 0)dot_left_bottom_l = 1;//左拐线
				else dot_left_bottom_r = 1;//右拐线
				if (dot_left_bottom_l)printf("turn left\n");
				if (dot_left_bottom_r)printf("turn right\n");
				printf("distance bottom left:%f\n", dist_sum_left_bottom);

				//左上
				L_top_corner_start = m3;
				if (L_edge_count - L_top_corner_start > effective_points)L_edge_count = L_top_corner_start + effective_points;
				else L_edge_count = L_edge_count;
				//直线顶点坐标(L_edge[L_top_corner_start].col,L_edge[L_top_corner_start].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
				line_left_top[0] = (float)(L_edge[L_edge_count - 1].col - L_edge[L_top_corner_start].col) / (float)(L_edge[L_edge_count - 1].row - L_edge[L_top_corner_start].row);   //k
				line_left_top[1] = (float)(L_edge[L_top_corner_start].col - line_left_top[0] * L_edge[L_top_corner_start].row);  //b
				//显示该直线
				if (L_edge[L_top_corner_start].row > L_edge[L_edge_count - 1].row)
				{
					m1 = L_edge[L_edge_count - 1].row;
					m2 = L_edge[L_top_corner_start].row;
				}
				else
				{
					m2 = L_edge[L_edge_count - 1].row;
					m1 = L_edge[L_top_corner_start].row;
				}
				printf("m1m2%d,%d\n", m1, m2);
				for (int32_t i = m1; i < m2; i++)
				{
					L_edge_use[i].row = i;
					L_edge_use[i].col = (int32_t)(i * line_left_top[0] + line_left_top[1]);
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
				max = 0;
				distance = 0;
				dist_sum_left_top = 0;
				dot_left_top_l = 0;
				dot_left_top_r - 0;

				for (int32_t i = L_top_corner_start; i < L_edge_count; i++)
				{
					distance = (float)(line_left_top[0] * L_edge[i].row - L_edge[i].col + line_left_top[1]) / sqrt(line_left_top[0] * line_left_top[0] + 1);
					//printf("dist:%f", distance);
					if (distance < max)
					{
						max = distance;
						center_lost_corner_row_l = L_edge[i].row;
						center_lost_corner_col_l = L_edge[i].col;
						L_top_i = i;
					}
					dist_sum_left_top += distance;//求总体差值判断是否为直线
				}
				L_center_lost_corner_angle = max;
				if (dist_sum_left_top < 0)dot_left_top_l = 1;//左拐线
				else dot_left_top_r = 1;//右拐线
				if (dot_left_top_l)printf("turn left\n");
				if (dot_left_top_r)printf("turn right\n");
				printf("distance top left:%f\n", dist_sum_left_top);
			}
			//another solution
			//if (enable_L_corner /*&& !L_start_lost*/) //如果使能搜索左拐点,且开始未丢线
			//{
			//	if (L_edge_count > 2 * dist + 1)
			//	{
			//		//printf("\nL_ed:%d ", L_top_corner_start - 9);
			//		//for (int32_t i = 0; i < L_top_corner_start - (2 * dist + 1); i++)//L_edge_count - 9
			//		//{
			//		//	//printf("L_ed2:%d ", L_edge[i].row);
			//		//	if (L_edge[i + dist].row > dist+1)
			//		//	{
			//		//		int32_t ang = (L_edge[i].col - L_edge[i + dist].col) * (L_edge[i + 2 * dist].col - L_edge[i + dist].col) +
			//		//			(L_edge[i].row - L_edge[i + dist].row) * (L_edge[i + 2 * dist].row - L_edge[i + dist].row);
			//		//		//printf("ang3:%d ", ang);
			//		//		if ( ang<= 0) //初步确认为锐角或者直角 向量法
			//		//		{
			//		//			//printf("L_ed3:%d ", L_edge[i].row);
			//		//			L_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col); //求角度
			//		//			L_corner_angle = 180 - L_corner_angle;
			//		//			if (L_edge[i + dist].col > L_edge[i + 2 * dist].col)    //确定拐角朝向，左拐点没有朝向做的
			//		//			{
			//		//				L_corner_flag = 1;//异常拐点
			//		//				L_corner_row = L_edge[i + dist].row;
			//		//				L_corner_col = L_edge[i + dist].col;
			//		//				//printf("左拐点：(x=%d,y=%d)，角度=%d\n", L_corner_row, L_corner_col, L_corner_angle);
			//		//				break;
			//		//			}
			//		//		}
			//		//	}
			//		//}
			//		int32_t max = 0;
			//		for (int32_t i = 0; i < L_top_corner_start - (2 * dist + 1); i++)//L_edge_count - 9
			//		{
			//			//printf("L_ed2:%d ", L_edge[i].row);
			//			if (L_edge[i + dist].row > dist + 1)
			//			{
			//				L_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col); //求角度
			//				L_corner_angle = 180 - L_corner_angle;
			//				//printf("L_ang:%d ", L_corner_angle);
			//				if (L_corner_angle > max)
			//				{
			//					max = L_corner_angle;
			//					L_corner_row = L_edge[i + dist].row;
			//					L_corner_col = L_edge[i + dist].col;
			//					L_beh_i = i + dist;
			//				}
			//			}
			//		}
			//		L_corner_angle = max;
			//	}
			//}
			//if(L_start_lost)
			//{
			//	int32_t min = 999;
			//	for (int32_t i = 1; i < L_edge_count - (2 * dist + 1); i++)
			//	{
			//		//printf("ang:%d \n", Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col));
			//		if (L_edge[i + dist].row > dist + 1)
			//		{
			//			L_center_lost_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col);
			//			if (L_center_lost_corner_angle < min)
			//			{
			//				min = L_center_lost_corner_angle;
			//				center_lost_corner_row_l = L_edge[i + dist].row;
			//				center_lost_corner_col_l = L_edge[i + dist].col;
			//				L_top_i = i + dist;
			//			}
			//			//int32_t ang = (L_edge[i].col - L_edge[i + dist].col) * (L_edge[i + 2 * dist].col - L_edge[i + dist].col) +
			//			//	(L_edge[i].row - L_edge[i + dist].row) * (L_edge[i + 2 * dist].row - L_edge[i + dist].row);
			//			//if (ang <= 0) //初步确认为锐角或者直角 向量法
			//			//{
			//			//	L_center_lost_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col); //求角度
			//			//	if (L_edge[i + dist].col < L_edge[i + 2 * dist].col && L_center_lost_corner_angle != 180)    //确定拐角朝向，左拐点没有朝向做的
			//			//	{
			//			//		L_corner_flag = 1;//异常拐点
			//			//		center_lost_corner_row_l = L_edge[i + dist].row;
			//			//		center_lost_corner_col_l = L_edge[i + dist].col;
			//			//		break;
			//			//	}
			//			//}
			//		}
			//	}
			//	L_center_lost_corner_angle = min;
			//}
			//if (center_lost_flag_l /*|| L_start_lost*/)
			//{
			//	int32_t min = 999;
			//	for (int32_t i = L_top_corner_start; i < L_edge_count - (2 * dist + 1); i++)
			//	{
			//		//printf("ang:%d \n", Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col));
			//		if (L_edge[i + dist].row > dist+1)
			//		{
			//			L_center_lost_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col);
			//			if (L_center_lost_corner_angle < min)
			//			{
			//				min = L_center_lost_corner_angle;
			//				center_lost_corner_row_l = L_edge[i + dist].row;
			//				center_lost_corner_col_l = L_edge[i + dist].col;
			//				L_top_i = i + dist;
			//			}
			//			//int32_t ang = (L_edge[i].col - L_edge[i + dist].col) * (L_edge[i + 2 * dist].col - L_edge[i + dist].col) +
			//			//	(L_edge[i].row - L_edge[i + dist].row) * (L_edge[i + 2 * dist].row - L_edge[i + dist].row);
			//			//if (ang <= 0 ) //初步确认为锐角或者直角 向量法
			//			//{
			//			//	L_center_lost_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col); //求角度
			//			//	if (L_edge[i + dist].col < L_edge[i + 2 * dist].col && L_center_lost_corner_angle!=180)    //确定拐角朝向，左拐点没有朝向做的
			//			//	{
			//			//		L_corner_flag = 1;//异常拐点
			//			//		center_lost_corner_row_l = L_edge[i + dist].row;
			//			//		center_lost_corner_col_l = L_edge[i + dist].col;
			//			//		break;
			//			//	}
			//			//}
			//		}
			//	}
			//	L_center_lost_corner_angle = min;
			//}
			printf("左下拐点：(x=%d,y=%d)，角度=%d\n", L_corner_row, L_corner_col, L_corner_angle);
			printf("左上拐点：(x=%d,y=%d)，角度=%d\n", center_lost_corner_row_l, center_lost_corner_col_l, L_center_lost_corner_angle);
			R_corner_flag = 0;//初始化变量
			R_corner_row = 0;
			R_corner_col = 0;
			R_corner_angle = 0;
			if (!R_start_lost && !center_lost_flag_r)//只有右下边线情况
			{
				if (R_edge_count > effective_points)R_edge_count = effective_points;
				else R_edge_count = R_edge_count;
				//直线顶点坐标(L_edge[0].col,L_edge[0].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
				printf("\n直线顶点为：(%d,%d),(%d,%d)", R_edge[0].row, R_edge[0].col, R_edge[R_edge_count - 1].row, R_edge[R_edge_count - 1].col);
				line_right_bottom[0] = (float)(R_edge[R_edge_count - 1].col - R_edge[0].col) / (float)(R_edge[R_edge_count - 1].row - R_edge[0].row);   //k
				line_right_bottom[1] = (float)(R_edge[0].col - line_right_bottom[0] * R_edge[0].row);  //b
				printf("  直线斜率：k=%f,b=%f\n", line_right_bottom[0], line_right_bottom[1]);																							 //显示该直线
				uint8_t m1, m2;
				if (R_edge[0].row > R_edge[R_edge_count - 1].row)
				{
					m1 = R_edge[R_edge_count - 1].row;
					m2 = R_edge[0].row;
				}
				else
				{
					m2 = R_edge[R_edge_count - 1].row;
					m1 = R_edge[0].row;
				}
				printf("m1m2:%d,%d", m1, m2);
				for (int32_t i = m1; i < m2; i++)
				{
					R_edge_use[i].row = i;
					R_edge_use[i].col = (int32_t)(i * line_right_bottom[0] + line_right_bottom[1]);
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
				//uint8_t m3;
				dist_sum_right_bottom = 0;
				dot_right_bottom_l = 0;
				dot_right_bottom_r = 0;
				is_line_right = 0;

				//printf("m3=%d\n", m3);
				for (int32_t i = 0; i < R_edge_count; i++)
				{
					distance = (float)(line_right_bottom[0] * R_edge[i].row - R_edge[i].col + line_right_bottom[1]) / sqrt(line_right_bottom[0] * line_right_bottom[0] + 1);
					printf("%f\n", distance);
					if (distance < max)
					{
						max = distance;
						R_corner_row = R_edge[i].row;
						R_corner_col = R_edge[i].col;
						R_beh_i = i;
					}
					dist_sum_right_bottom += distance;//求总体差值判断是否为直线
				}
				R_corner_angle = max;
				if (dist_sum_right_bottom < 0)dot_right_bottom_l = 1;//左拐线
				else dot_right_bottom_r = 1;//右拐线
				if ((dist_sum_right_bottom * (dist_sum_right_bottom < 0 ? -1 : 1)) < dist_control)is_line_right = 1;

				if (dot_right_bottom_l)printf("turn left\n");
				if (dot_right_bottom_r)printf("turn right\n");
				printf("distance bottom right:%f\n", dist_sum_right_bottom);
			}
			else if (R_start_lost)//只有右上边线情况
			{
				if (R_edge_count > effective_points)R_edge_count = effective_points;
				else R_edge_count = R_edge_count;
				//直线顶点坐标(L_edge[1].col,L_edge[1].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
				printf("\n直线顶点为：(%d,%d),(%d,%d)", R_edge[1].row, R_edge[1].col, R_edge[R_edge_count - 1].row, R_edge[R_edge_count - 1].col);
				line_right_top[0] = (float)(R_edge[R_edge_count - 1].col - R_edge[1].col) / (float)(R_edge[R_edge_count - 1].row - R_edge[1].row);   //k
				line_right_top[1] = (float)(R_edge[1].col - line_right_top[0] * R_edge[1].row);  //b
				printf("  直线斜率：k=%f,b=%f\n", line_right_top[0], line_right_top[1]);
				//显示该直线
				uint8_t m1, m2;
				if (R_edge[1].row > R_edge[R_edge_count - 1].row)
				{
					m1 = R_edge[R_edge_count - 1].row;
					m2 = R_edge[1].row;
				}
				else
				{
					m2 = R_edge[R_edge_count - 1].row;
					m1 = R_edge[1].row;
				}
				printf("  m1=%d,m2=%d\n", m1, m2);
				for (int32_t i = m1; i < m2; i++)
				{
					R_edge_use[i].row = i;
					R_edge_use[i].col = (int32_t)(i * line_right_top[0] + line_right_top[1]);
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
				float max = 0;
				float distance = 0;
				//uint8_t m3;
				dist_sum_right_top = 0;
				dot_right_top_l = 0;
				dot_right_top_r = 0;
				is_line_right = 0;

				for (int32_t i = 1; i < R_edge_count; i++)
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
				R_center_lost_corner_angle = max;
				if (dist_sum_right_top < 0)dot_right_top_l = 1;//左拐线
				else dot_right_top_r = 1;//右拐线
				if ((dist_sum_right_top * (dist_sum_right_top < 0 ? -1 : 1)) < dist_control)is_line_right = 1;

				if (dot_right_top_l)printf("turn left\n");
				if (dot_right_top_r)printf("turn right\n");
				printf("distance top right:%f\n", dist_sum_right_top);
			}
			else if (center_lost_flag_r)//有右上和右下边线情况   #TODO  增加对distance正负的判断
			{
				//右下
				uint8_t m3 = R_top_corner_start;
				if (R_top_corner_start > effective_points)R_top_corner_start = effective_points;
				else R_top_corner_start = R_top_corner_start;
				//直线顶点坐标(L_edge[0].col,L_edge[0].row),(L_edge[L_top_corner_start-1].col,L_edge[L_top_corner_start-1].row)
				line_right_bottom[0] = (float)(R_edge[R_top_corner_start - 1].col - R_edge[0].col) / (float)(R_edge[R_top_corner_start - 1].row - R_edge[0].row);   //k
				line_right_bottom[1] = (float)(R_edge[0].col - line_right_bottom[0] * R_edge[0].row);  //b
				//显示该直线
				uint8_t m1, m2;
				if (R_edge[0].row > R_edge[R_top_corner_start - 1].row)
				{
					m1 = R_edge[R_top_corner_start - 1].row;
					m2 = R_edge[0].row;
				}
				else
				{
					m2 = R_edge[R_top_corner_start - 1].row;
					m1 = R_edge[0].row;
				}
				for (int32_t i = m1; i < m2; i++)
				{
					R_edge_use[i].row = i;
					R_edge_use[i].col = (int32_t)(i * line_right_bottom[0] + line_right_bottom[1]);
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
				float max = 0;
				float distance = 0;
				//uint8_t m3;
				dist_sum_right_bottom = 0;
				dot_right_bottom_l = 0;
				dot_right_bottom_r = 0;

				for (int32_t i = 0; i < R_top_corner_start; i++)
				{
					distance = (float)(line_right_bottom[0] * R_edge[i].row - R_edge[i].col + line_right_bottom[1]) / sqrt(line_right_bottom[0] * line_right_bottom[0] + 1);
					if (distance > max)
					{
						max = distance;
						R_corner_row = R_edge[i].row;
						R_corner_col = R_edge[i].col;
						R_beh_i = i;
					}
					dist_sum_right_bottom += distance;//求总体差值判断是否为直线
				}
				R_corner_angle = max;
				if (dist_sum_right_bottom < 0)dot_right_bottom_l = 1;//左拐线
				else dot_right_bottom_r = 1;//右拐线
				if (dot_right_bottom_l)printf("turn left\n");
				if (dot_right_bottom_r)printf("turn right\n");
				printf("distance bottom right:%f\n", dist_sum_right_bottom);

				//右上
				R_top_corner_start = m3;
				if (R_edge_count - R_top_corner_start > effective_points)R_edge_count = R_top_corner_start + effective_points;
				else R_edge_count = R_edge_count;
				//直线顶点坐标(L_edge[L_top_corner_start].col,L_edge[L_top_corner_start].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
				line_right_top[0] = (float)(R_edge[R_edge_count - 1].col - R_edge[R_top_corner_start].col) / (float)(R_edge[R_edge_count - 1].row - R_edge[R_top_corner_start].row);   //k
				line_right_top[1] = (float)(R_edge[R_top_corner_start].col - line_right_top[0] * R_edge[R_top_corner_start].row);  //b
				//显示该直线
				if (R_edge[R_top_corner_start].row > R_edge[R_edge_count - 1].row)
				{
					m1 = R_edge[R_edge_count - 1].row;
					m2 = R_edge[R_top_corner_start].row;
				}
				else
				{
					m2 = R_edge[R_edge_count - 1].row;
					m1 = R_edge[R_top_corner_start].row;
				}
				for (int32_t i = m1; i < m2; i++)
				{
					R_edge_use[i].row = i;
					R_edge_use[i].col = (int32_t)(i * line_right_top[0] + line_right_top[1]);
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
				max = 0;
				distance = 0;
				dist_sum_right_top = 0;
				dot_right_top_l = 0;
				dot_right_top_r = 0;

				for (int32_t i = R_top_corner_start; i < R_edge_count; i++)
				{
					distance = (float)(line_right_top[0] * R_edge[i].row - R_edge[i].col + line_right_top[1]) / sqrt(line_right_top[0] * line_right_top[0] + 1);
					//printf("dist:%f", distance);
					if (distance > max)
					{
						max = distance;
						center_lost_corner_row_r = R_edge[i].row;
						center_lost_corner_col_r = R_edge[i].col;
						R_top_i = i;
					}
					dist_sum_right_top += distance;//求总体差值判断是否为直线
				}
				R_center_lost_corner_angle = max;
				if (dist_sum_right_top < 0)dot_right_top_l = 1;//左拐线
				else dot_right_top_r = 1;//右拐线
				if (dot_right_top_l)printf("turn left\n");
				if (dot_right_top_r)printf("turn right\n");
				printf("distance top right:%f\n", dist_sum_right_top);
			}
			//another solution
			//if (enable_R_corner /*&& !R_start_lost*/)    //如果使能搜索右拐点
			//{
			//	if (R_edge_count > (2 * dist + 1))
			//	{
			//		//for (int32_t i = 0; i < R_top_corner_start - (2 * dist + 1); i++)//R_edge_count - 9
			//		//{
			//		//	if (R_edge[i + dist].row > dist+1)
			//		//	{
			//		//		int32_t ang = (R_edge[i].col - R_edge[i + dist].col) * (R_edge[i + 2 * dist].col - R_edge[i + dist].col) +
			//		//			(R_edge[i].row - R_edge[i + dist].row) * (R_edge[i + 2 * dist].row - R_edge[i + dist].row);
			//		//		//printf("ang=%d ", ang);
			//		//		if ( ang<= 0) //初步确认为锐角或者直角 向量法
			//		//		{
			//		//			R_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //求角度
			//		//			R_corner_angle = 180 - R_corner_angle;
			//		//			if (R_edge[i + 2 * dist].col > R_edge[i + dist].col)    //确定拐角朝向，左拐点没有朝向做的
			//		//			{
			//		//				R_corner_flag = 1;//异常拐点
			//		//				R_corner_row = R_edge[i + dist].row;
			//		//				R_corner_col = R_edge[i + dist].col;
			//		//				break;
			//		//			}
			//		//		}
			//		//	}
			//		//}
			//		int32_t max = 0;
			//		for (int32_t i = 0; i < R_top_corner_start - (2 * dist + 1); i++)//L_edge_count - 9
			//		{
			//			//printf("L_ed2:%d ", L_edge[i].row);
			//			if (R_edge[i + dist].row > dist + 1)
			//			{
			//				R_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //求角度
			//				R_corner_angle = 180 - R_corner_angle;
			//				//printf("R_ang:%d ", R_corner_angle);
			//				if (R_corner_angle > max)
			//				{
			//					max = R_corner_angle;
			//					R_corner_row = R_edge[i + dist].row;
			//					R_corner_col = R_edge[i + dist].col;
			//					R_beh_i = i + dist;
			//				}
			//			}
			//		}
			//		R_corner_angle = max;
			//	}
			//}
			//if (R_start_lost)
			//{
			//	int32_t min = 999;
			//	//printf("LLL%d", R_edge_count);
			//	//限制在70点以内  有bug需要调
			//	for (int32_t i = 0; i < 70 - (2 * dist + 1); i++)
			//	{
			//		if (R_edge[i + dist].row > dist + 1)
			//		{
			//			R_center_lost_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //求角度
			//			if (R_center_lost_corner_angle < min && R_center_lost_corner_angle>90)
			//			{
			//				min = R_center_lost_corner_angle;
			//				center_lost_corner_row_r = R_edge[i + dist].row;
			//				center_lost_corner_col_r = R_edge[i + dist].col;
			//				R_top_i = i + dist;
			//			}
			//			//int32_t ang = (R_edge[i].col - R_edge[i + dist].col) * (R_edge[i + 2 * dist].col - R_edge[i + dist].col) +
			//			//	(R_edge[i].row - R_edge[i + dist].row) * (R_edge[i + 2 * dist].row - R_edge[i + dist].row);
			//			////printf("ang2=%d ", ang);
			//			//if (ang <= 0) //初步确认为锐角或者直角 向量法
			//			//{
			//			//	R_center_lost_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //求角度
			//			//	if (R_edge[i + dist].col > R_edge[i + 2 * dist].col && R_center_lost_corner_angle != 180)    //确定拐角朝向，左拐点没有朝向做的
			//			//	{
			//			//		R_corner_flag = 1;//异常拐点
			//			//		center_lost_corner_row_r = R_edge[i + dist].row;
			//			//		center_lost_corner_col_r = R_edge[i + dist].col;
			//			//		break;
			//			//	}
			//			//}
			//		}
			//	}
			//	R_center_lost_corner_angle = min;
			//}
			//if (center_lost_flag_r /*|| R_start_lost*/)
			//{
			//	int32_t min = 999;
			//	for (int32_t i = R_top_corner_start; i < R_edge_count - (2 * dist + 1); i++)
			//	{
			//		if (R_edge[i + dist].row > dist+1)
			//		{
			//			R_center_lost_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //求角度
			//			if (R_center_lost_corner_angle < min)
			//			{
			//				min = R_center_lost_corner_angle;
			//				center_lost_corner_row_r = R_edge[i + dist].row;
			//				center_lost_corner_col_r = R_edge[i + dist].col;
			//				R_top_i = i + dist;
			//			}																																									  //int32_t ang = (R_edge[i].col - R_edge[i + dist].col) * (R_edge[i + 2 * dist].col - R_edge[i + dist].col) +
			//			//	(R_edge[i].row - R_edge[i + dist].row) * (R_edge[i + 2 * dist].row - R_edge[i + dist].row);
			//			////printf("ang2=%d ", ang);
			//			//if (ang <= 0) //初步确认为锐角或者直角 向量法
			//			//{
			//			//	R_center_lost_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //求角度
			//			//	if (R_edge[i + dist].col > R_edge[i + 2 * dist].col && R_center_lost_corner_angle != 180)    //确定拐角朝向，左拐点没有朝向做的
			//			//	{
			//			//		R_corner_flag = 1;//异常拐点
			//			//		center_lost_corner_row_r = R_edge[i + dist].row;
			//			//		center_lost_corner_col_r = R_edge[i + dist].col;
			//			//		break;
			//			//	}
			//			//}
			//		}
			//	}
			//	R_center_lost_corner_angle = min;
			//}
			printf("右下拐点：(x=%d,y=%d)，角度=%d\n", R_corner_row, R_corner_col, R_corner_angle);
			printf("右上拐点：(x=%d,y=%d)，角度=%d\n", center_lost_corner_row_r, center_lost_corner_col_r, R_center_lost_corner_angle);
		}

	}

	//判断元素，生成最终的边线数组
	if (tjrc_imageProc_ctr.enable_element)
	{
		if (dot_left_bottom_l && dot_right_bottom_r && dot_right_top_r && dot_left_top_l)//正十字
		{
			printf("this is zheng shi zi\n");
			//printf("\n直线顶点为：(%d,%d),(%d,%d)",L_corner_col, L_corner_row, center_lost_corner_col_l, center_lost_corner_row_l);
			line_left_top[0] = (float)(center_lost_corner_col_l - L_corner_col) / (float)(center_lost_corner_row_l - L_corner_row);   //k
			line_left_top[1] = (float)(L_corner_col - line_left_top[0] * L_corner_row);  //b
			//printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
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
			for (int32_t i = m1; i < m2; i++)
			{
				L_edge_use[i].row = i;
				L_edge_use[i].col = (int32_t)(i * line_left_top[0] + line_left_top[1]);
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
			for (int32_t i = L_beh_i; i > 0; i--)
			{
				L_edge_use[m2].row = L_edge[i].row;
				L_edge_use[m2].col = L_edge[i].col;
				m2++;
			}
			for (int32_t i = L_top_i; i < L_edge_count; i++)
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
			for (int32_t i = m1; i < m2; i++)
			{
				R_edge_use[i].row = i;
				R_edge_use[i].col = (int32_t)(i * line_right_top[0] + line_right_top[1]);
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
			for (int32_t i = R_beh_i; i > 0; i--)
			{
				R_edge_use[m2].row = R_edge[i].row;
				R_edge_use[m2].col = R_edge[i].col;
				m2++;
			}
			for (int32_t i = R_top_i; i < R_edge_count; i++)
			{
				R_edge_use[m1].row = R_edge[i].row;
				R_edge_use[m1].col = R_edge[i].col;
				m1--;
			}
		}

		if (!dot_left_bottom_l && !dot_right_bottom_r && dot_right_top_r && dot_left_top_l)//中十字
		{
			printf("this is zhong shi zi\n");
			//edge_truncation(0);
			FitStraightLine(L_top_i, L_top_i + 10, 0);
			printf("k_l:%f,b_l:%f\n", k_l, b_l);
			for (int32_t i = L_edge[L_top_i].row; i < 80; i++)
			{
				L_edge_use[i].row = i;
				L_edge_use[i].col = (int32_t)(i * k_l + b_l);
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
			for (int32_t i = L_top_i; i < L_edge_count; i++)
			{
				L_edge_use[m1].row = L_edge[i].row;
				L_edge_use[m1].col = L_edge[i].col;
				m1--;
			}

			//edge_truncation(1);
			FitStraightLine(R_top_i, R_top_i + 10, 1);
			printf("k_r:%f,b_r:%f\n", k_r, b_r);
			for (int32_t i = R_edge[R_top_i].row; i < 80; i++)
			{
				R_edge_use[i].row = i;
				R_edge_use[i].col = (int32_t)(i * k_r + b_r);
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
			for (int32_t i = R_top_i; i < R_edge_count; i++)
			{
				R_edge_use[m2].row = R_edge[i].row;
				R_edge_use[m2].col = R_edge[i].col;
				m2--;
			}
		}

		if (!dot_left_bottom_l && dot_right_bottom_r && dot_right_top_l && dot_left_top_r)//左斜入十字
		{
			printf("this is zuo xieru shi zi\n");
		}

		if (dot_left_bottom_l && !dot_right_bottom_r && dot_right_top_l && dot_left_top_r)//右斜入十字
		{
			printf("this is you xieru shi zi\n");
		}

		if (dot_left_bottom_l && dot_left_top_l && is_line_right)//左圆环
		{
			printf("this is zuo yuan huan\n");
			//printf("\n直线顶点为：(%d,%d),(%d,%d)",L_corner_col, L_corner_row, center_lost_corner_col_l, center_lost_corner_row_l);
			line_left_top[0] = (float)(center_lost_corner_col_l - L_corner_col) / (float)(center_lost_corner_row_l - L_corner_row);   //k
			line_left_top[1] = (float)(L_corner_col - line_left_top[0] * L_corner_row);  //b
			//printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
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
			for (int32_t i = m1; i < m2; i++)
			{
				L_edge_use[i].row = i;
				L_edge_use[i].col = (int32_t)(i * line_left_top[0] + line_left_top[1]);
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
			for (int32_t i = L_beh_i; i > 0; i--)
			{
				L_edge_use[m2].row = L_edge[i].row;
				L_edge_use[m2].col = L_edge[i].col;
				m2++;
			}
			for (int32_t i = L_top_i; i < L_edge_count; i++)
			{
				L_edge_use[m1].row = L_edge[i].row;
				L_edge_use[m1].col = L_edge[i].col;
				m1--;
			}
			m1 = 79;
			for (int32_t i = 0; i < 75; i++)
			{
				R_edge_use[m1].row = R_edge[i].row;
				R_edge_use[m1].col = R_edge[i].col;
				m1--;
			}
		}

		if (dot_right_bottom_r && dot_right_top_r && is_line_left)//右圆环
		{
			printf("this is you yuan huan\n");
			line_right_top[0] = (float)(center_lost_corner_col_r - R_corner_col) / (float)(center_lost_corner_row_r - R_corner_row);   //k
			line_right_top[1] = (float)(R_corner_col - line_right_top[0] * R_corner_row);  //b
			//printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
			//显示该直线
			uint8_t m1, m2;
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
			for (int32_t i = m1; i < m2; i++)
			{
				R_edge_use[i].row = i;
				R_edge_use[i].col = (int32_t)(i * line_right_top[0] + line_right_top[1]);
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
			for (int32_t i = R_beh_i; i > 0; i--)
			{
				R_edge_use[m2].row = R_edge[i].row;
				R_edge_use[m2].col = R_edge[i].col;
				m2++;
			}
			for (int32_t i = R_top_i; i < R_edge_count; i++)
			{
				R_edge_use[m1].row = R_edge[i].row;
				R_edge_use[m1].col = R_edge[i].col;
				m1--;
			}
			m1 = 79;
			for (int32_t i = 0; i < 75; i++)
			{
				L_edge_use[m1].row = L_edge[i].row;
				L_edge_use[m1].col = L_edge[i].col;
				m1--;
			}
		}

		//if (dot_left_top_l && is_line_right)//左入圆环 #状态机中使用
		//{
		//	printf("this is zuo ru yuan huan\n");
		//	//printf("\n直线顶点为：(%d,%d),(%d,%d)",L_corner_col, L_corner_row, center_lost_corner_col_l, center_lost_corner_row_l);
		//	line_left_top[0] = (float)(center_lost_corner_col_l - 119) / (float)(center_lost_corner_row_l - 79);   //k
		//	line_left_top[1] = (float)(119 - line_left_top[0] * 79);  //b
		//	//printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
		//	//显示该直线
		//	uint8_t m1, m2;
		//	if (79 > center_lost_corner_row_l)
		//	{
		//		m1 = center_lost_corner_row_l;
		//		m2 = 79;
		//	}
		//	else
		//	{
		//		m2 = center_lost_corner_row_l;
		//		m1 = 79;
		//	}
		//	//printf("  m1=%d,m2=%d\n", m1, m2);
		//	for (int32_t i = m1; i < m2; i++)
		//	{
		//		L_edge_use[i].row = i;
		//		L_edge_use[i].col = (int32_t)(i * line_left_top[0] + line_left_top[1]);
		//		//printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
		//		if (L_edge_use[i].col < 0)
		//		{
		//			L_edge_use[i].col = 0;
		//		}
		//		if (L_edge_use[i].col > 119)
		//		{
		//			L_edge_use[i].col = 119;
		//		}
		//	}
		//}

		//if (dot_right_top_l && is_line_left)//右入圆环  #状态机中使用
		//{
		//	printf("this is you ru yuan huan\n");
		//	line_right_top[0] = (float)(center_lost_corner_col_r - 0) / (float)(center_lost_corner_row_r - 79);   //k
		//	line_right_top[1] = (float)(0 - line_right_top[0] * 79);  //b
		//	//printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
		//	//显示该直线
		//	uint8_t m1, m2;
		//	if (79 > center_lost_corner_row_r)
		//	{
		//		m1 = center_lost_corner_row_r;
		//		m2 = 79;
		//	}
		//	else
		//	{
		//		m2 = center_lost_corner_row_r;
		//		m1 = 79;
		//	}
		//	//printf("  m1=%d,m2=%d\n", m1, m2);
		//	for (int32_t i = m1; i < m2; i++)
		//	{
		//		R_edge_use[i].row = i;
		//		R_edge_use[i].col = (int32_t)(i * line_right_top[0] + line_right_top[1]);
		//		//printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
		//		if (R_edge_use[i].col < 0)
		//		{
		//			R_edge_use[i].col = 0;
		//		}
		//		if (R_edge_use[i].col > 119)
		//		{
		//			R_edge_use[i].col = 119;
		//		}
		//	}
		//}
	}
	

	//计算中线
	for (int32_t i = 0; i < 80; i++)
	{
		Mid_Line[i].col = (L_edge_use[i].col + R_edge_use[i].col) / 2;
		Mid_Line[i].row = i;
	}

	

	//gui传递参数
	//显示左右的搜线，需要调整为tft函数
	//for (int32_t i = 0, m = 0; i < 300; i = i + 2, m++)
	//{
	//	sidee[i] = L_edge[m].row;
	//	sidee[i + 1] = L_edge[m].col;
	//	if (sidee[i] > 79) sidee[i] = 79;
	//	if (sidee[i + 1] > 119) sidee[i + 1] = 119;
	//	//if (sidee[i] == 0) sidee[i] = 1;
	//	//if (sidee[i+1] == 0) sidee[i+1] = 1;
	//	//printf("%d ", sidee[i]);
	//}
	//for (int32_t i = 300, m = 0; i < 600; i = i + 2, m++)
	//{
	//	sidee[i] = R_edge[m].row;
	//	sidee[i + 1] = R_edge[m].col;
	//	if (sidee[i] > 79) sidee[i] = 79;
	//	if (sidee[i + 1] > 119) sidee[i + 1] = 119;
	//	//if (sidee[i] == 0) sidee[i] = 1;
	//	//if (sidee[i + 1] == 0) sidee[i + 1] = 1;
	//}

	//显示中线，需要调整为tft函数
	//for (int32_t i = 600, m = 0; i < 900; i = i + 2, m++)
	//{
	//	sidee[i] = Mid_Line[m].row;
	//	sidee[i + 1] = Mid_Line[m].col;
	//	if (sidee[i] > 79) sidee[i] = 79;
	//	if (sidee[i + 1] > 119) sidee[i + 1] = 119;
	//	//if (sidee[i] == 0) sidee[i] = 1;
	//	//if (sidee[i + 1] == 0) sidee[i + 1] = 1;
	//}

	//显示的中间参数
	//sidee[900] = jilu_row_l; //左边线起始点
	//sidee[901] = jilu_col_l;
	//sidee[902] = jilu_row_r; //右边线起始点
	//sidee[903] = jilu_col_r;

	//sidee[904] = L_basic_row_start;//丢线情况，左边线起始点
	//sidee[905] = L_edge_start_col;
	//sidee[906] = R_basic_row_start;//丢线情况，右边线起始点
	//sidee[907] = R_edge_start_col;

	//sidee[908] = center_lost_row_l;//中间左丢线情况，左边线起始点
	//sidee[909] = center_lost_col_l;
	//sidee[910] = center_lost_row_r;//中间右丢线情况，右边线起始点
	//sidee[911] = center_lost_col_r;

	//sidee[912] = L_corner_row;//左下拐点
	//sidee[913] = L_corner_col;
	//sidee[914] = L_corner_angle;

	//sidee[915] = R_corner_row;//右下拐点
	//sidee[916] = R_corner_col;
	//sidee[917] = R_corner_angle;

	//sidee[918] = center_lost_corner_row_l;//左上拐点
	//sidee[919] = center_lost_corner_col_l;
	//sidee[920] = L_center_lost_corner_angle;

	//sidee[921] = center_lost_corner_row_r;//右上拐点
	//sidee[922] = center_lost_corner_col_r;
	//sidee[923] = R_center_lost_corner_angle;

	//最终的左右边线，需要调整为tft函数
	//for (int32_t i = 1000, m = 0; i < 1240; i = i + 2, m++)
	//{
	//	sidee[i] = L_edge_use[m].row;
	//	sidee[i + 1] = L_edge_use[m].col;
	//	if (sidee[i] > 79) sidee[i] = 79;
	//	if (sidee[i + 1] > 119) sidee[i + 1] = 119;
	//	if (sidee[i] < 0) sidee[i] = 0;
	//	if (sidee[i + 1] < 0) sidee[i + 1] = 0;
	//	//printf("hhhh(%d,%d)\n", L_edge_use[m].row, L_edge_use[m].col);
	//}
	//for (int32_t i = 1240, m = 0; i < 1480; i = i + 2, m++)
	//{
	//	sidee[i] = R_edge_use[m].row;
	//	sidee[i + 1] = R_edge_use[m].col;
	//	if (sidee[i] > 79) sidee[i] = 79;
	//	if (sidee[i + 1] > 119) sidee[i + 1] = 119;
	//	if (sidee[i] < 0) sidee[i] = 0;
	//	if (sidee[i + 1] < 0) sidee[i + 1] = 0;
	//}

	//return sidee;
}


/*--------------------------------------------------------------------------
* 【函数功能】：获取中线
* 【参    数】：无
* 【返 回 值】：无
* 【备    注】：无
*--------------------------------------------------------------------------*/
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
/*--------------------------------------------------------------------------
* 【函数功能】：初始化左边界右边界结构体
* 【参    数】：无
* 【返 回 值】：无
*--------------------------------------------------------------------------*/
void clear_point()
{
	for (int32_t i = 0; i < L_edge_count; i++)
	{
		L_edge[i].row = 0;
		L_edge[i].col = 0;
		L_edge[i].flag = 0;
	}
	for (int32_t i = 0; i < R_edge_count; i++)
	{
		R_edge[i].row = 0;
		R_edge[i].col = 0;
		R_edge[i].flag = 0;
	}
}
/*--------------------------------------------------------------------------
* 【函数功能】：求拐点的角度值
* 【参    数】：三个点的行列坐标
* 【返 回 值】：角度值
* 【备    注】：无
*--------------------------------------------------------------------------*/
//int32_t Get_angle(uint8_t ax, uint8_t ay, uint8_t bx, uint8_t by, uint8_t cx, uint8_t cy)
//{
//	int8 abx = ax - bx;
//	int8 aby = ay - by;
//	int8 cbx = cx - bx;
//	int8 cby = cy - by;
//	int8 ab_muti_cb = abx * cbx + aby * cby;
//	int8 dist_ab = sqrt(abx * abx + aby * aby);
//	int8 dist_cb = sqrt(cbx * cbx + cby * cby);
//	int8 cosvalue = ab_muti_cb / (dist_ab * dist_cb);
//	return (int32_t)(acos(cosvalue) * 180 / 3.14159);
//}
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
* 【函数功能】：判断是否存在左/右边界点
* 【参    数】：搜线基本行数值      左/右边界选择标志(0:左边，1:右边)
* 【返 回 值】：-1 或 边界点列值，-1表示没有找到边界点
* 【备    注】：无
*--------------------------------------------------------------------------*/
int32_t edge_point_ornot(uint8_t row, uint8_t side)  //5和155需要调整
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
				for (int32_t col = L_edge_start_col; col < Image_W / 2; col++)
				{
					//printf("image[%d][%d],is black:%d\n", rowi, col, black_(image[rowi][col]));
					if (black_(image[rowi][col]))
					{
						//printf("col:%d\n ", col);
						//如果出现黑黑白白，则判断为边界线，退出循环
						if (black_(image[rowi][col]) && black_(image[rowi][col + 1]) && white_(image[rowi][col + 2]) && white_(image[rowi][col + 3]))
						{
							L_basic_row_start = rowi;   //赋值开始搜线行（左）
							printf("左边线起始点：(x=%d,y=%d)\n", rowi, col + 1);
							//if (rowi<Image_H / 2 && col>Image_W / 2) return -1;
							return col + 1;               //返回列值         //!!!   1->2
						}
					}
					if (col == Image_W / 2 - 1)
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
				if (black_(image[rowi - 3][L_edge_start_col]) && black_(image[rowi - 2][L_edge_start_col]) && white_(image[rowi - 1][L_edge_start_col]) && white_(image[rowi][L_edge_start_col]))
				{
					L_basic_row_start = rowi - 2;   //赋值开始搜线行（左）
					printf("丢线情况，左边线起始点：(x=%d,y=%d)\n", L_basic_row_start, L_edge_start_col);
					L_start_lost = 1;
					//if (rowi<Image_H / 2 && col>Image_W / 2) return -1;
					return L_edge_start_col;               //返回列值         //!!!   1->2
				}
			}
			if (find_edge == 2) return -1;
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
				for (int32_t col = R_edge_start_col; col > Image_W / 2; col--)
				{
					if (black_(image[rowi][R_edge_start_col]))
					{
						//如果出现黑黑白白，则判断为边界线，退出循环
						if (white_(image[rowi][col - 3]) && white_(image[rowi][col - 2]) && black_(image[rowi][col - 1]) && black_(image[rowi][col]))
						{
							R_basic_row_start = rowi;
							printf("右边线起始点：(x=%d,y=%d)\n", rowi, col + 1);
							if (rowi < Image_H / 2 && col < Image_W / 2) return -1;
							return col - 1;
						}
					}
					if (col == Image_W / 2 + 1)
					{
						R_lost_count++;
					}
				}
			}

			if (R_lost_count > R_lost_)//丢线大于次数限制开启行搜索
			{
				if (black_(image[rowi - 3][R_edge_start_col]) && black_(image[rowi - 2][R_edge_start_col]) && white_(image[rowi - 1][R_edge_start_col]) && white_(image[rowi][R_edge_start_col]))
				{
					R_basic_row_start = rowi - 2;   //赋值开始搜线行（左）
					printf("丢线情况，右边线起始点：(x=%d,y=%d)\n", R_basic_row_start, R_edge_start_col);
					R_start_lost = 1;
					//if (rowi<Image_H / 2 && col>Image_W / 2) return -1;
					return R_edge_start_col;               //返回列值         //!!!   1->2
				}
			}
			if (find_edge == 2) return -1;
		}

	}
	return -1;
}
/*--------------------------------------------------------------------------
* 【函数功能】：判断是否是黑像素/白像素点，为后期不适用二值化图像做准备
* 【参    数】：该点的像素值
* 【返 回 值】：无
* 【备    注】：无
*--------------------------------------------------------------------------*/
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
/*--------------------------------------------------------------------------
* 【函数功能】：获取二值化图像
* 【参    数】：无
* 【返 回 值】：无
* 【备    注】：无
*--------------------------------------------------------------------------*/
void get_binImage(uint8_t thres)
{
	binimage_flag = 1;  //二值化标志置为1
	for (int32_t row = 0; row < Image_H; row++)
	{
		for (int32_t col = 0; col < Image_W; col++)
		{
			if (image[row][col] < thres)
			{
				image[row][col] = 0;    //小于阈值赋值为0，显示为黑色
			}
			else
				image[row][col] = 1;    //大于阈值赋值为1，显示为白色
		}
	}
}
/*--------------------------------------------------------------------------
* 【函数功能】：大津法求动态阈值
* 【参    数】：无
* 【返 回 值】：无
*--------------------------------------------------------------------------*/
#if 0
#define GrayScale 256   //frame

typedef unsigned char uchar;
int32_t pixel[256] = { 0 };
uint8_t OSTU_bin(uint8_t width, uint8_t height, uint8_t* Image)
{
	int32_t threshold1 = 0;
	int32_t sum_gray = 0;
	int32_t sum_pix_num = 0;
	int32_t pl_pix_num = 0;
	int32_t p2_pix_mum = 0;
	int32_t p1_sum_gray = 0;
	float m1 = 0;
	float m2 = 0;
	float V = 0;
	float variance = 0;
	int32_t i, j, k = 0;

	for (i = 0; i < 256; i++)
		pixel[i] = 0;


	for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
		{
			pixel[(int32_t)Image[i * width + j]]++;
		}
	}

	for (k = 0; k < GrayScale; k++)
	{
		sum_gray += k * pixel[k];
		sum_pix_num += pixel[k];
	}

	for (k = 0; k < GrayScale - 1; k++)
	{
		pl_pix_num += pixel[k];
		p2_pix_mum = sum_pix_num - pl_pix_num;
		p1_sum_gray += k * pixel[k];
		m1 = (float)p1_sum_gray / pl_pix_num;
		m2 = (float)(sum_gray - p1_sum_gray) / p2_pix_mum;

		V = pl_pix_num * p2_pix_mum * (m1 - m2) * (m1 - m2);

		if (V > variance)
		{
			variance = V;
			threshold1 = k;
		}
	}
	return threshold1;
}

#endif
/*--------------------------------------------------------------------------
* 【函数功能】：大津法求动态阈值
* 【参    数】：无
* 【返 回 值】：无
*--------------------------------------------------------------------------*/
#if 1
uint8_t GetOSTUThreshold(uint8_t(*img)[MT9V03X_W], uint16_t start_row, uint16_t end_row, uint16_t start_col, uint16_t end_col)
{
	int32_t     threshold1 = 0;
	int32_t sum_gray = 0;
	int32_t sum_pix_num = 0;
	int32_t pl_pix_num = 0;
	int32_t p2_pix_mum = 0;
	int32_t p1_sum_gray = 0;
	float m1 = 0;
	float m2 = 0;
	float V = 0;
	float variance = 0;
	int32_t i, j, k = 0;
	uint16_t MinValue = 0, MaxValue = 255;
	uint16_t DeleGrayClass1 = 30; //高灰度级
	uint16_t  HistoGram[256] = { 0 };

	for (i = 0; i < 256; i++)
		HistoGram[i] = 0;

	for (i = start_row; i < end_row; i++)
	{
		for (j = start_col; j < end_col; j++)
		{
			HistoGram[(int32_t)img[i][j]]++;
		}
	}

	//优化--删除灰度级顶部<x个点的灰度级  删除灰度级底部<x个点的灰度级 x==> 10-25之间
	//for(k=255;k>0;--k) {if(HistoGram[k]<=DeleGrayClass1)  HistoGram[k] = 0; else break;}
	//for(k=0;k<256;++k) {if(HistoGram[k]<=DeleGrayClass2)  HistoGram[k] = 0; else break;}

	for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; ++MinValue);        //获取最小灰度级
	for (MaxValue = 255; MaxValue > MinValue && HistoGram[MaxValue] == 0; --MaxValue); //获取最大灰度级

	for (k = MinValue; k <= MaxValue; k++)
	{
		sum_gray += k * HistoGram[k];
		sum_pix_num += HistoGram[k];
	}

	for (k = MinValue; k <= MaxValue; k++)
	{
		pl_pix_num += HistoGram[k];
		p2_pix_mum = sum_pix_num - pl_pix_num;
		p1_sum_gray += k * HistoGram[k];
		m1 = (float)p1_sum_gray / pl_pix_num;
		m2 = (float)(sum_gray - p1_sum_gray) / p2_pix_mum;

		V = pl_pix_num * p2_pix_mum * (m1 - m2) * (m1 - m2);

		if (V > variance)
		{
			variance = V;
			threshold1 = k;
		}
	}

#if 1
	uint8_t t1 = threshold1 / 2;
	uint8_t t2 = threshold1 + 10;  //拉伸范围
	float rate = 1.25;          //拉伸比例！！
	//uint8_t top_y   = 15;            //顶部行域 0-top_y
	uint8_t side_x = 20;            //侧边列域 0-side_x && (COL_1-side_x)-COL


	/* 初始化 */
	for (i = 255; i > 0; i--)
		HistoGram[i] = i;

	/* 梯度变换--对比度拉伸 */
	for (i = t1; i < t2; i++)
	{
		HistoGram[i] = (uint16_t)(i * rate);
		if (HistoGram[i] > t2) HistoGram[i] = t2;
	}

	//左侧
	for (i = 0; i < MT9V03X_H; i++)
	{
		for (j = 0; j < side_x; j++)
		{
			mt9v03x_image[i][j] = HistoGram[mt9v03x_image[i][j]];
		}
	}

	//右侧
	for (i = 0; i < MT9V03X_H; i++)
	{
		for (j = MT9V03X_W - 1 - side_x; j < MT9V03X_W; j++)
		{
			mt9v03x_image[i][j] = HistoGram[mt9v03x_image[i][j]];
		}
	}
#endif
	return threshold1;
}
#endif
/*--------------------------------------------------------------------------
* 【函数功能】：均值滤波
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

void get_deal_image()
{
	for (int32_t row = 0; row < Image_H; row++)
		for (int32_t col = 0; col < Image_W; col++)
		{
			image[row][col] = mt9v03x_image[row][col];
		}
}


// 点集三角滤波
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

int32_t clip(int32_t x, int32_t low, int32_t up) {
	return x > up ? up : x < low ? low : x;
}

float fclip(float x, float low, float up) {
	return x > up ? up : x < low ? low : x;
}

void edge_start()
{
	for (int32_t i = 0; i < 150; i++)
	{
		L_edge[i].row = 0;
		L_edge[i].col = 0;
		R_edge[i].row = 0;
		R_edge[i].col = 0;
	}
}

//边线截断处理
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
		//	num_cnt = 0;//记录连续水平点的个数
		//	L_count = L_edge_count / 2;
		//	while (L_count < L_edge_count)
		//	{
		//		if (L_edge[L_count].row == L_edge[L_count + 1].row)
		//			num_cnt = num_cnt + 1;
		//		else
		//			num_cnt = 0;
		//		if (num_cnt > 5)//连续5个点水平
		//			break;
		//		L_count = L_count + 1;
		//	}
		//	L_edge_count = L_count;//截断在5个水平点处
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
		//	num_cnt = 0;//记录连续水平点的个数
		//	L_count = L_edge_count / 2;
		//	while (L_count < L_edge_count)
		//	{
		//		if (L_edge[L_count].row == L_edge[L_count + 1].row)
		//			num_cnt = num_cnt + 1;
		//		else
		//			num_cnt = 0;
		//		if (num_cnt > 5)//连续5个点水平
		//			break;
		//		L_count = L_count + 1;
		//	}
		//	L_edge_count = L_count;//截断在5个水平点处
		//}
	}

}

//最小二乘拟合
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

//void variance(LEFT_EDGE side[], uint8_t start, uint8_t end)
//{
//	int32_t sum=0;
//	for (int32_t i = start; i < end; i++)
//	{
//		sum += side[i].col;
//	}
//}

// 左手迷宫巡线  //y=115,x=30
void findline_lefthand_adaptive(int32_t jilu_row, int32_t jilu_col, int32_t search_amount)
{
	L_edge[0].row = jilu_row;
	L_edge[0].col = jilu_col;
	L_edge[0].flag = 1;
	uint8_t curr_row = jilu_row;//初始化行坐标
	uint8_t curr_col = jilu_col;//初始化列坐标
	dire_left = 0; //初始化上个边界点的来向
	center_turn_flag = 1;//初始化在未标定状态
	center_lost_flag_l = 0;//中间左丢线标志位
	//开始搜线，最多取150个点，不会往下搜，共7个方位
	for (int32_t i = 1; i < search_amount; i++)    //最多搜索70个点
	{
		////越界退出 行越界和列越界（向上向下向左向右）
		if (curr_row < L_edge_end_row || curr_row>Image_H - 1 || curr_row + 1 < L_edge_end_row)  break;
		if (curr_col > max_col || curr_col < min_col && !center_lost_flag_l)   //max=160-5 min=5
		{
			if (++L_search_edge_count == 3)//连续3次搜索到边界，退出      //连续搜索到边界次数需要调整  5和155需要调整
			{
				curr_col = L_edge_lost_start_col;

				for (uint8_t rowi = curr_row; rowi > 0; rowi--)
				{
					//printf("row=%d", rowi);
					if (black_(image[rowi - 3][curr_col]) && black_(image[rowi - 2][curr_col]) && white_(image[rowi - 1][curr_col]) && white_(image[rowi][curr_col]))
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
				}
			}
		}
		else
			L_search_edge_count = 0;
		//搜线过程
		if (black_(image[curr_row + dir_front[dire_left][0]][curr_col + dir_front[dire_left][1]]))
		{
			dire_left = (dire_left + 1) % 4;
			i = i - 1;
			//turn++;
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
		//printf("(x1=%d,y1=%d)\n", L_edge[i].row, L_edge[i].col);
		//dd = dd + 1;
	}
	//printf("count%d ", L_edge_count);
}

// 右手迷宫巡线  //y=115,x=30
void findline_righthand_adaptive(int32_t jilu_row, int32_t jilu_col, int32_t search_amount)
{
	R_edge[0].row = jilu_row;
	R_edge[0].col = jilu_col;
	R_edge[0].flag = 1;
	uint8_t curr_row = jilu_row;//初始化行坐标
	uint8_t curr_col = jilu_col;//初始化列坐标
	dire_right = 0; //初始化上个边界点的来向
	center_turn_flag = 1;//初始化在未标定状态
	center_lost_flag_r = 0;//中间左丢线标志位
	//开始搜线，最多取150个点，不会往下搜，共7个方位
	for (int32_t i = 1; i < search_amount; i++)    //最多搜索70个点
	{
		////越界退出 行越界和列越界（向上向下向左向右）
		if (curr_row < R_edge_end_row || curr_row>Image_H - 1 || curr_row + 1 < R_edge_end_row)  break;
		if (curr_col > max_col || curr_col < min_col && !center_lost_flag_r)   //max=160-5 min=5
		{
			if (++R_search_edge_count == 3)//连续3次搜索到边界，退出      //连续搜索到边界次数需要调整  5和155需要调整
			{
				curr_col = R_edge_lost_start_col;

				for (uint8_t rowi = curr_row; rowi > 0; rowi--)
				{
					//printf("row=%d", rowi);
					if (black_(image[rowi - 3][curr_col]) && black_(image[rowi - 2][curr_col]) && white_(image[rowi - 1][curr_col]) && white_(image[rowi][curr_col]))
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
				}
			}
		}
		else
			R_search_edge_count = 0;
		//搜线过程
		if (black_(image[curr_row + dir_front[dire_right][0]][curr_col + dir_front[dire_right][1]]))
		{
			dire_right = (dire_right + 3) % 4;
			i = i - 1;
			//turn++;
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
		//printf("(x1=%d,y1=%d)\n", L_edge[i].row, L_edge[i].col);
		//dd = dd + 1;
	}
	//printf("count%d ", L_edge_count);
}

//计算方差
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
uint8_t XLW_otsuThreshold(uint8_t image[MT9V03X_H][MT9V03X_W], uint16_t col, uint16_t row)
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
	uint8_t threshold = 0;
	uint8_t* data = image[0];
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
			threshold = (uint8_t)i;
		}
	}
	threshold -= 1;
	if (threshold * 2 > th_max)
		threshold = th_max / 2;
	if (threshold * 2 < th_min)
		threshold = th_min / 2;

	return threshold * 2;
}


/*!
  * @brief    大津法求阈值大小(龙邱)
  * @param    tmImage ： 图像数据
  * @return   阈值
 * @note     Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
  * @note     1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
  * @note     2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
  * @note     3) i表示分类的阈值，也即一个灰度级，从0开始迭代	1
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
int32_t GetOSTU(unsigned char tmImage[MT9V03X_H][MT9V03X_W])
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
			HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
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
			temp[2] = -(int32_t)imageIn[i - 1][j] + (int32_t)imageIn[i][j - 1]			   //  0, -1, -1
				- (int32_t)imageIn[i][j + 1] + (int32_t)imageIn[i + 1][j]				   //  1,  0, -1
				- (int32_t)imageIn[i - 1][j + 1] + (int32_t)imageIn[i + 1][j - 1];		   //  1,  1,  0
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
void SobelAutoThreshold(uint8_t imageIn[MT9V03X_H][MT9V03X_W])
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
			temp[2] = -(int32_t)imageIn[i - 1][j] + (int32_t)imageIn[i][j - 1]			   //  0, -1, -1
				- (int32_t)imageIn[i][j + 1] + (int32_t)imageIn[i + 1][j]				   //  1,  0, -1
				- (int32_t)imageIn[i - 1][j + 1] + (int32_t)imageIn[i + 1][j - 1];		   //  1,  1,  0
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

			/* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
			temp[3] = (int32_t)imageIn[i - 1][j - 1] + (int32_t)imageIn[i - 1][j] + (int32_t)imageIn[i - 1][j + 1]
				+ (int32_t)imageIn[i][j - 1] + (int32_t)imageIn[i][j] + (int32_t)imageIn[i][j + 1]
				+ (int32_t)imageIn[i + 1][j - 1] + (int32_t)imageIn[i + 1][j] + (int32_t)imageIn[i + 1][j + 1];

			if (temp[0] > temp[3] / 10.0f)
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


/*!
  * @brief    二值化
  * @param    mode ：0：使用龙邱大津法阈值,1：使用平均阈值,2: sobel 算子改进型-手动阈值,3：sobel 算子改进型-动态阈值,4：OTSU
  * @return   无
  * @note     无
  * @see      Get_01_Value(0); //使用大津法二值化
  * @date     2020/5/15 星期二
  */
  //uint8_t Get_01_Value(uint8_t image_in[MT9V03X_H][MT9V03X_W],unsigned char mode)
  //{
  //	int32_t i = 0, j = 0;
  //	int32_t Threshold = 0;
  //	unsigned long  tv = 0;
  //	char txt[16];
  //
  //	if (mode == 0)
  //	{
  //		Threshold = GetOSTU(image_in);//大津法阈值
  //		return Threshold;
  //	}
  //	if (mode == 1)
  //	{
  //		//累加
  //		for (i = 0; i < MT9V03X_H; i++)
  //		{
  //			for (j = 0; j < MT9V03X_W; j++)
  //			{
  //				tv += image_in[i][j];   //累加
  //			}
  //		}
  //		Threshold = tv / MT9V03X_H / MT9V03X_W;        //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
  //		Threshold = Threshold + 20;      //此处阈值设置，根据环境的光线来设定
  //		return Threshold;
  //	}
  //	else if (mode == 2)
  //	{
  //		Threshold = 50;
  //		//手动调节阈值
  //		SobelThreshold(image_in, (uint8_t)Threshold);
  //		return 1;
  //
  //	}
  //	else if (mode == 3)
  //	{
  //		SobelAutoThreshold(image_in);  //动态调节阈值
  //		return 1;
  //	}
  //	else if (mode == 4)
  //	{
  //		Threshold = XLW_otsuThreshold(image_in, MT9V03X_H, MT9V03X_W);  //OTSU
  //		//return Threshold ;
  //	}
  //	/* 二值化 */
  //	for (i = 0; i < MT9V03X_H; i++)
  //	{
  //		for (j = 0; j < MT9V03X_W; j++)
  //		{
  //			if (image_in[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
  //			{
  //				image_in[i][j] = 255;
  //			}
  //			else
  //			{
  //				image_in[i][j] = 1;
  //			}
  //		}
  //	}
  //	return 1;
  //}

  // TODO
uint8_t* Get_01_Value(uint8_t image_in[MT9V03X_H][MT9V03X_W], uint8_t image_out[MT9V03X_H * MT9V03X_W + 1], unsigned char mode)
{
	int32_t i = 0, j = 0;
	int32_t Threshold = 0;
	unsigned long  tv = 0;
	char txt[16];

	if (mode == 0)
	{
		Threshold = GetOSTU(image_in);//大津法阈值
		//return Threshold;
	}
	if (mode == 1)
	{
		//累加
		for (i = 0; i < MT9V03X_H; i++)
		{
			for (j = 0; j < MT9V03X_W; j++)
			{
				tv += image_in[i][j];   //累加
			}
		}
		Threshold = tv / MT9V03X_H / MT9V03X_W;        //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
		Threshold = Threshold + 20;      //此处阈值设置，根据环境的光线来设定
		//return Threshold;
	}
	else if (mode == 2)
	{
		Threshold = 50;
		//手动调节阈值
		SobelThreshold(image_in, (uint8_t)Threshold);
		//return 1;

	}
	else if (mode == 3)
	{
		SobelAutoThreshold(image_in);  //动态调节阈值
		//return 1;
	}
	else if (mode == 4)
	{
		Threshold = XLW_otsuThreshold(image_in, MT9V03X_H, MT9V03X_W);  //OTSU
		//return Threshold ;
	}
	/* 二值化 */
	int32_t m = 0;
	if (mode == 0 || mode == 1 || mode == 4)
	{
		for (i = 0; i < MT9V03X_H; i++)
		{
			for (j = i * 120, m = 0; j < (i + 1) * 120; j++, m++)
			{
				if (image_in[i][m] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
				{
					//	//
					image_out[j] = 255; //printf("%d，", image_out[i][j]);
				}
				else
				{
					//
					image_out[j] = 1; //printf("%d，", image_out[i][j]);
				}
			}
			//printf("||||%d",i*188);
			//printf("\nTH:%d", Threshold);
		}
	}
	else
	{
		for (i = 0; i < MT9V03X_H; i++)
		{
			for (j = i * 120, m = 0; j < (i + 1) * 120; j++, m++)
			{
				image_out[j] = image_in[i][m]; //printf("%d，", image_out[i][j]);
			}
			//printf("||||%d",i*188);
			//printf("\nTH:%d", Threshold);
		}
	}
	image_out[MT9V03X_H * MT9V03X_W] = Threshold;
	//printf("\nTH:%d\n%d", Threshold, image_out[MT9V03X_H * MT9V03X_W]);

	return image_out;
}

int32_t add(int32_t num1, int32_t num2)
{
	return num1 + num2;
}


