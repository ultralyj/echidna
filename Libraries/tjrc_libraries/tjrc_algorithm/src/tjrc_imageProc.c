/**
 * @file tjrc_imageProc.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 图像处理主文件，将灰度图像进行赛道元素识别，输出方向误差信息
 * @version 1.2
 * @date 2022-04-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_imageProc.h"

//声明全局变量-------------------------------------------
//  图像
//      图像预处理
uint8_t threshold = 70; //二值化阈值
IFX_ALIGN(4) uint8_t image_binarization[MT9V03X_H][MT9V03X_W];

uint8_t binimage_flag = 0;		   //二值化标志，默认没有二值化
struct LEFT_EDGE L_edge[150];	   //左边界结构体  #全局 使用
struct LEFT_EDGE L_edge2[150];	   // #全局 未使用
struct LEFT_EDGE R_edge[150];	   //右边界结构体  #全局 使用
struct LEFT_EDGE M_Line[150];	   //中线结构体，备用选择2
struct LEFT_EDGE Last_M_Line[150]; //上次中线结构体
struct LEFT_EDGE MID_LINE[150];	   //中线结构体，备用选择1

uint8_t effective_points = 65; //用于防止单段边线长度过长导致拐点误判 #全局 控制 使用

struct LEFT_EDGE L_edge_use[80]; //左边界结构体 最后使用  #全局 使用
struct LEFT_EDGE R_edge_use[80]; //右边界结构体 最后使用  #全局 使用

struct LEFT_EDGE *Mid_Line = M_Line; //中线结构体【实际使用】
//      用于中线处理
uint8_t searchpoint = 1;			  //正常求出来的中线点标志，用于结构体.flag，便于后期处理
uint8_t nihepoint = 2;				  //拟合求出来的中线点标志，用于结构体.flag，便于后期处理
extern uint8_t Road_Width[MT9V03X_H]; //质量矩，用于丢线
//      图像显示
uint8_t l_display_cnt = 0;
uint8_t r_display_cnt = 0;

//直线斜率数据存放数组  #全局 使用
float line_left_bottom[2];
float line_left_top[2];
float line_right_top[2];
float line_right_bottom[2];

//拐点标志 #全局 flag 使用
uint8_t dot_left_bottom_l = 0;	//左下拐点，趋势左
uint8_t dot_left_bottom_r = 0;	//左下拐点，趋势右
uint8_t dot_left_top_l = 0;		//左上拐点，趋势左
uint8_t dot_left_top_r = 0;		//左上拐点，趋势右
uint8_t dot_right_bottom_l = 0; //右下拐点，趋势左
uint8_t dot_right_bottom_r = 0; //右下拐点，趋势右
uint8_t dot_right_top_l = 0;	//右上拐点，趋势左
uint8_t dot_right_top_r = 0;	//右上拐点，趋势右

//是否为直线标志 #全局 flag 使用
uint8_t is_line_left = 0;
uint8_t is_line_right = 0;

//直线总体偏差 #全局 使用
float dist_control = 180; //判断是否为直线的控制参数  #全局 控制 使用
float dist_sum_left_bottom = 0;
float dist_sum_left_top = 0;
float dist_sum_right_bottom = 0;
float dist_sum_right_top = 0;

//相关变量

//  发车/停车标志  #全局 未使用
uint8_t run_flag = 0;
uint8_t stop_flag = 0;
//  图像处理：一般处理
//      使能相关变量  #全局 使用
uint8_t enable_balinyu = 1;				//八邻域爬线使能标志，默认使能
uint8_t enable_midline = 1;				//使能中线拟合，默认不开启
uint8_t enable_midpross = 1;			//使能中线处理，默认开启
uint8_t enable_check_l_r_edge_same = 0; //使能左、右边线是否重合，默认不开启
//      搜线限制相关  #全局 控制 使用
uint8_t L_edge_end_row = 5;					  //左边界行结束点
uint8_t R_edge_end_row = 5;					  //右
uint8_t M_edge_end_row = 5;					  //中
uint8_t min_col = 3, max_col = MT9V03X_W - 3; //搜索结束列值
uint8_t L_edge_start_col = 3;				  //左边界列搜索起始点
uint8_t R_edge_start_col = 117;				  //右边界列搜索起始点

// #全局 flag 使用
uint8_t L_start_lost = 0; //左边线起始丢线标志
uint8_t R_start_lost = 0; //右边线起始丢线标志
//      搜线计数相关  #全局 控制 使用
uint8_t Image_H = MT9V03X_H;
uint8_t Image_W = MT9V03X_W;
uint8_t left_findflag, right_findflag;
uint8_t L_basic_row_start = MT9V03X_H - 1;				  //左边线搜线开始点
uint8_t R_basic_row_start = MT9V03X_H - 2;				  //右边线搜线开始点
uint8_t L_search_edge_count = 0, R_search_edge_count = 0; //搜索到边界
uint8_t line_point_count_left, line_point_count_right;	  //左右线有效点计数
uint8_t L_edge_count = 0, R_edge_count = 0;				  //左右边点的个数
uint8_t Mid_count = 0;									  //中线点的个数
uint8_t center_arry_count;
uint8_t line_lose_center_left;
uint8_t line_lose_center_right;
uint8_t dire_left;		  //记录上一个点的相对位置
uint8_t dire_right;		  //记录上一个点的相对位置
uint8_t center_turn_flag; // 0 左趋势， 1 右趋势
uint8_t center_biaoxiang_arry[5];

// #全局 flag 使用
uint8_t center_lost_flag_l = 0;		 //中间左丢线标志位;
uint8_t center_lost_row_l = 0;		 //中间左丢线开始行坐标
uint8_t center_lost_col_l = 0;		 //中间左丢线开始列坐标
uint8_t center_lost_flag_r = 0;		 //中间右丢线标志位
uint8_t center_lost_row_r = 0;		 //中间右丢线开始行坐标
uint8_t center_lost_col_r = 0;		 //中间右丢线开始列坐标
uint8_t L_edge_lost_start_col = 4;	 //中间左丢线列搜索起始点
uint8_t R_edge_lost_start_col = 115; //中间右丢线列搜索起始点

// #全局 控制 使用
uint8_t L_lost_count = 0; //左丢线计数
uint8_t L_lost_ = 10;	  //左丢线限制次数
uint8_t R_lost_count = 0; //右丢线计数
uint8_t R_lost_ = 10;	  //右丢线限制次数
//      边界预处理 #全局 使用
uint8_t edge_process_flag = 0;
uint8_t pre_L_edge_count = 0;
uint8_t pre_R_edge_count = 0;
uint8_t num_cnt = 0; //记录连续水平点的个数
uint8_t L_count = 0;
uint8_t R_count = 0;
float line_blur_kernel = 7; //边线三角滤波核的大小
//          拐点处理  #全局 使用
uint8_t enable_L_corner = 1; //左拐点搜索使能标志 默认使能
uint8_t enable_R_corner = 1; //右拐点搜索是能标志 默认使能
uint8_t L_corner_flag = 0;	 //左拐点存在标志
uint8_t L_corner_row = 0;	 //左拐点所在行
uint8_t L_corner_col = 0;	 //左拐点所在列
int L_corner_angle = 0;		 //左拐点角度
uint8_t R_corner_flag = 0;	 //右拐点存在标志
uint8_t R_corner_row = 0;	 //右拐点所在行
uint8_t R_corner_col = 0;	 //右拐点所在列
int R_corner_angle = 0;		 //右拐点角度

// #全局 使用
uint8_t L_top_corner_start = 0; //左上拐点开始序号
uint8_t R_top_corner_start = 0; //右上拐点开始序号

// #全局 使用
uint8_t L_top_i = 0; //左上拐角对应i
uint8_t L_beh_i = 0; //左下拐角对应i
uint8_t R_top_i = 0; //右上拐角对应i
uint8_t R_beh_i = 0; //右下拐角对应i

// #全局 控制 使用
uint8_t dist = 4; //求拐点角度的点数间隔

//      中线相关  #全局 使用
uint8_t center_lost_corner_row_l = 0; //中间左丢线开始拐点行坐标
uint8_t center_lost_corner_col_l = 0; //中间左丢线开始拐点列坐标
int L_center_lost_corner_angle = 0;	  //中间左丢线开始左拐点角度
uint8_t center_lost_corner_row_r = 0; //中间右丢线开始拐点行坐标
uint8_t center_lost_corner_col_r = 0; //中间右丢线开始拐点列坐标
int R_center_lost_corner_angle = 0;	  //中间右丢线开始左拐点角度

// 拟合斜率  #全局 使用
float k_l = 0;
float b_l = 0;

float k_r = 0;
float b_r = 0;

//方差  #全局 使用
float statistics1 = 0;

//  数学参数  #全局 使用
/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */
int dir_front[4][2] = {{-1, 0},
					   {0, 1},
					   {1, 0},
					   {0, -1}};
/* 前进方向定义：
 * 0   1
 *
 * 3   2
 */
int dir_frontleft[4][2] = {{-1, -1},
						   {-1, 1},
						   {1, 1},
						   {1, -1}};
/* 前进方向定义：
 * 3   0
 *
 * 2   1
 */
int dir_frontright[4][2] = {{-1, 1},
							{1, 1},
							{1, -1},
							{-1, -1}};

static void clear_point(void);
/*--------------------------------------------------------------------------
 * 【函数功能】：图像处理（中线、元素识别）_八邻域容易越界
 * 【参    数】：无
 * 【返 回 值】：无
 *--------------------------------------------------------------------------*/
int32_t x1, x2, x3, x4;

tjrc_image_info tjrc_imageProc_ctr = {
	//###控制参数列表
	78,	 //左边线行搜线开始点
	78,	 //右边线行搜线开始点
	2,	 //左边界列搜索起始点
	117, //右边界列搜索起始点
	150, //左右边界搜点时最多允许的点
	150, //左右边界搜点时最多允许的点
	3,	 //最小列坐标，即边界值
	117, //最大列坐标，即边界值
	10,	 //左丢线限制次数
	10,	 //右丢线限制次数
	4,	 //中间左丢线列搜索起始点
	115, //中间右丢线列搜索起始点
	4,	 //求拐点角度的点数间隔
	65,	 //用于防止单段边线长度过长导致拐点误判 #全局 控制 使用
	180, //判断是否为直线的控制参数  #全局 控制 使用

	//###标志位参数列表
	1, //使能二值化标志位
	0, //使能八邻域搜线标志位
	0, //使能拐点搜索标准位
	0  //使能识别元素标志位
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
void tjrc_imageProcess(uint8_t *image)
{
	//变量相关
	//  全局、外部变量--------------------------
	//      全局变量
	//      外部变量
	//  函数内部变量-------------------
	// uint8_t Bottom2 = Image_H - 40;   //倒数第二行
	uint8_t max_col = Image_W - 5, min_col = 5;								//最大/小列坐标，即边界值  #内部 控制 使用
	uint8_t L_search_amount = 150, R_search_amount = 150;					//左右边界搜点时最多允许的点  #内部 控制 使用
	uint8_t jilu_row_l = 0, jilu_col_l = 0, jilu_row_r = 0, jilu_col_r = 0; //记录搜索到的基础边界点行列值   #内部 使用
																			//图像处理-----------------------
																			//  一般图像处理
																			//      初始化相关变量
	enable_check_l_r_edge_same = 0;											//使能检查做右边线是否爬重合，默认不开启，当爬线起始点过高时开启
	left_findflag = 0;														//左边界存在标志，1找到左边界，0没找到左边界        默认没有找到左边界
	right_findflag = 0;														//右边界存在标志，1找到有边界，0没找到右边界        默认没有找到右边界

	// #全局 控制 使用
	L_basic_row_start = Image_H - 2; //左开始搜线点
	R_basic_row_start = Image_H - 2; //右开始搜线点
									 //      获取图像二值化阈值和图像二值化
	// get_deal_image();

	/* 控制参数传入 */
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

	/* 变量初始化 */
	for (int32_t i = 0; i < 80; i++)
	{
		L_edge_use[i].row = 0;
		L_edge_use[i].col = 0;
		L_edge_use[i].flag = 0;
		R_edge_use[i].row = 0;
		R_edge_use[i].col = 0;
		R_edge_use[i].flag = 0;
	}
	dot_left_bottom_l = 0;	//左下拐点，趋势左
	dot_left_bottom_r = 0;	//左下拐点，趋势右
	dot_left_top_l = 0;		//左上拐点，趋势左
	dot_left_top_r = 0;		//左上拐点，趋势右
	dot_right_bottom_l = 0; //右下拐点，趋势左
	dot_right_bottom_r = 0; //右下拐点，趋势右
	dot_right_top_l = 0;	//右上拐点，趋势左
	dot_right_top_r = 0;	//右上拐点，趋势右

	//直线斜率数据存放数组 //变量初始化
	//	line_left_bottom[2] = { 0,0 };
	//	line_left_top[2] = { 0,0 };
	//	line_right_top[2] = { 0,0 };
	//	line_right_bottom[2] = { 0,0 };

	//变量初始化
	is_line_left = 0;
	is_line_right = 0;

	//直线总体偏差 //变量初始化
	dist_sum_left_bottom = 0;
	dist_sum_left_top = 0;
	dist_sum_right_bottom = 0;
	dist_sum_right_top = 0;

	//变量初始化
	center_lost_row_l = 0; //中间左丢线开始行坐标
	center_lost_col_l = 0; //中间左丢线开始列坐标
	center_lost_row_r = 0; //中间左丢线开始行坐标
	center_lost_col_r = 0; //中间左丢线开始列坐标

	/* 二值化 */
	if (tjrc_imageProc_ctr.enable_bin)
	{
		/* method1: 使用大津法计算二值化的阈值 */
		uint8_t th_otsu = tjrc_binarization_otsu(image, Image_W, Image_H);
		//	    /* method2: 使用平均法计算二值化的阈值 */
		//	    uint8_t th_avg = tjrc_binarization_avg(image,Image_W,Image_H);
		/* 修改二值化阈值，并加上偏置 */
		uint8_t th_bias = 5;
		threshold = th_otsu + th_bias;
		rt_kprintf("[imp]:binarization mothed: OTSU, threshold:%d\r\n", threshold);
		tjrc_binarization_getBinImage(threshold, image, (uint8_t *)image_binarization[0], Image_W, Image_H);
		// 	    /* method3: sobel */
		// 	    tjrc_sobel_autoThreshold(image,(uint8_t*)image_binarization[0],Image_W,Image_H);
		tjrc_st7735_dispImage_gray((uint8_t *)image_binarization[0], Image_W, Image_H, 0, 28);
	}

	// 开始爬边线
	if (tjrc_imageProc_ctr.enable_balinyu) //如果使能八邻域爬线即一般处理
	{
		//相关变量
		line_lose_center_left = 0;
		line_lose_center_right = 0;
		line_point_count_left = 0;
		line_point_count_right = 0;
		L_edge_count = 0;			 //左边点个数清0
		R_edge_count = 0;			 //右边点个数清0
		int32_t exist_edge_size = 0; //判断是否存在左/右边界

		/* 寻找搜线起始点 */
		//寻找左/右线开始点，并判断是否存在当前边
		clear_point();
		exist_edge_size = edge_point_ornot(image, L_basic_row_start, 0);
		if (exist_edge_size >= 0)
		{
			jilu_row_l = L_basic_row_start;
			jilu_col_l = exist_edge_size;
			left_findflag = 1;
		}
		//		rt_kprintf("左边线起始点：(x=%d,y=%d)\r\n", jilu_row_l, jilu_col_l);

		exist_edge_size = edge_point_ornot(image, R_basic_row_start, (uint8_t)(1));
		if (exist_edge_size >= 0)
		{
			jilu_row_r = R_basic_row_start;
			jilu_col_r = exist_edge_size;
			right_findflag = 1;
		}
		//		rt_kprintf("右边线起始点：(x=%d,y=%d)\r\n", jilu_row_r, jilu_col_r);

		/* 进行八邻域搜线  */
		if (left_findflag) //如果左边界点存在并找到,则开始爬线
		{
			//变量声明
			L_edge[0].row = jilu_row_l;
			L_edge[0].col = jilu_col_l;
			L_edge[0].flag = 1;
			// uint8_t curr_row = jilu_row_l;//初始化行坐标
			// uint8_t curr_col = jilu_col_l;//初始化列坐标
			dire_left = 0;			//初始化上个边界点的来向
			center_turn_flag = 1;	//初始化在未标定状态
			center_lost_flag_l = 0; //中间左丢线标志位
			//开始搜线，最多取150个点，不会往下搜，共7个方位
			findline_lefthand_adaptive(image, jilu_row_l, jilu_col_l, L_search_amount);
			for (int i = 0; i < L_edge_count; i++)
			{
				tjrc_st7735_drawPoint(L_edge[i].col, 80 + L_edge[i].row, 0xF800);
			}
		}
		if (right_findflag) //如果右边界存在并搜到
		{
			R_edge[0].row = jilu_row_r;
			R_edge[0].col = jilu_col_r;
			R_edge[0].flag = 1;
			// uint8_t curr_row = jilu_row_r;
			// uint8_t curr_col = jilu_col_r;
			dire_right = 0;
			center_lost_flag_r = 0; //中间右丢线标志位
			findline_righthand_adaptive(image, jilu_row_r, jilu_col_r, R_search_amount);
			for (int i = 0; i < L_edge_count; i++)
			{
				tjrc_st7735_drawPoint(R_edge[i].col, 80 + R_edge[i].row, 0x07E0);
			}
		}

		//检测拐点,判断边线的左右趋势
		/* 寻找上下拐点 */
		if (tjrc_imageProc_ctr.enable_turnpoint)
		{
			L_corner_flag = 0; // 初始化变量
			L_corner_row = 0;
			L_corner_col = 0;
			L_corner_angle = 0;
			if (!L_start_lost && !center_lost_flag_l) //只有左下边线情况
			{
				uint8_t m3 = 0;
				if (L_edge_count > effective_points)
					L_edge_count = effective_points;
				//				else L_edge_count = L_edge_count;
				//直线顶点坐标(L_edge[0].col,L_edge[0].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
				rt_kprintf("[imp]line(%d,%d),(%d,%d)\r\n", L_edge[0].row, L_edge[0].col, L_edge[L_edge_count - 1].row, L_edge[L_edge_count - 1].col);
				line_left_bottom[0] = (float)(L_edge[L_edge_count - 1].col - L_edge[0].col) / (float)(L_edge[L_edge_count - 1].row - L_edge[0].row); // k
				line_left_bottom[1] = (float)(L_edge[0].col - line_left_bottom[0] * L_edge[0].row);													 // b
				rt_kprintf("[imp]k=%f, b=%f\r\n", line_left_bottom[0], line_left_bottom[1]);														 //显示该直线
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
					// printf("hh(%d,%d)\n", L_edge_use[i].col, L_edge_use[i].row);
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
					// printf("%f\n", distance);
					if (distance > max)
					{
						max = distance;
						L_corner_row = L_edge[i].row;
						L_corner_col = L_edge[i].col;
						L_beh_i = i;
					}
					// printf("%f\n", distance);
					dist_sum_left_bottom += distance; //求总体差值判断是否为直线
				}
				L_corner_angle = max;
				if (dist_sum_left_bottom < 0)
					dot_left_bottom_l = 1; //左拐线
				else
					dot_left_bottom_r = 1; //右拐线
				if ((dist_sum_left_bottom * (dist_sum_left_bottom < 0 ? -1 : 1)) < dist_control)
					is_line_left = 1;

				//				if (dot_left_bottom_l)printf("turn left\n");
				//				if (dot_left_bottom_r)printf("turn right\n");
				//	printf("distance bottom left:%f\n", dist_sum_left_bottom);
			}
			else if (L_start_lost) //只有左上边线情况
			{
				uint8_t m3 = 0;
				if (L_edge_count > effective_points)
					L_edge_count = effective_points;
				//				else L_edge_count = L_edge_count;
				//直线顶点坐标(L_edge[1].col,L_edge[1].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
				// printf("\n直线顶点为：(%d,%d),(%d,%d)", L_edge[1].row, L_edge[1].col, L_edge[L_edge_count - 1].row, L_edge[L_edge_count - 1].col);
				line_left_top[0] = (float)(L_edge[L_edge_count - 1].col - L_edge[1].col) / (float)(L_edge[L_edge_count - 1].row - L_edge[1].row); // k
				line_left_top[1] = (float)(L_edge[1].col - line_left_top[0] * L_edge[1].row);													  // b
				// printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
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
				// printf("  m1=%d,m2=%d\n", m1, m2);
				for (int32_t i = m1; i < m2; i++)
				{
					L_edge_use[i].row = i;
					L_edge_use[i].col = (int32_t)(i * line_left_top[0] + line_left_top[1]);
					// printf("hh(%d,%d)\n", L_edge_use[i].col, L_edge_use[i].row);
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
					dist_sum_left_top += distance; //求总体差值判断是否为直线
				}
				L_center_lost_corner_angle = max;
				if (dist_sum_left_top < 0)
					dot_left_top_l = 1; //左拐线
				else
					dot_left_top_r = 1; //右拐线
				if ((dist_sum_left_top * (dist_sum_left_top < 0 ? -1 : 1)) < dist_control)
					is_line_left = 1;

				//				if (dot_left_top_l)printf("turn left\n");
				//				if (dot_left_top_r)printf("turn right\n");
				//	printf("distance top left:%f\n", dist_sum_left_top);
			}
			else if (center_lost_flag_l) //有左上和左下边线情况   #TODO  增加对distance正负的判断
			{
				//左下
				uint8_t m3 = L_top_corner_start;
				if (L_top_corner_start > effective_points)
					L_top_corner_start = effective_points;
				//				else L_top_corner_start = L_top_corner_start;
				//直线顶点坐标(L_edge[0].col,L_edge[0].row),(L_edge[L_top_corner_start-1].col,L_edge[L_top_corner_start-1].row)
				//	printf("\n直线顶点为：(%d,%d),(%d,%d)", L_edge[0].row, L_edge[0].col, L_edge[L_top_corner_start - 1].row, L_edge[L_top_corner_start - 1].col);
				line_left_bottom[0] = (float)(L_edge[L_top_corner_start - 1].col - L_edge[0].col) / (float)(L_edge[L_top_corner_start - 1].row - L_edge[0].row); // k
				line_left_bottom[1] = (float)(L_edge[0].col - line_left_bottom[0] * L_edge[0].row);																 // b
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
				//	printf("m1m2%d,%d\n", m1, m2);
				for (int32_t i = m1; i < m2; i++)
				{
					L_edge_use[i].row = i;
					L_edge_use[i].col = (int32_t)(i * line_left_bottom[0] + line_left_bottom[1]);
					// printf("hh(%d,%d)\n", L_edge_use[i].row, L_edge_use[i].col);
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
				// uint8_t m3 = 0;
				dist_sum_left_bottom = 0;
				dot_left_bottom_l = 0;
				dot_left_bottom_r = 0;

				for (int32_t i = 0; i < L_top_corner_start; i++)
				{
					distance = (float)(line_left_bottom[0] * L_edge[i].row - L_edge[i].col + line_left_bottom[1]) / sqrt(line_left_bottom[0] * line_left_bottom[0] + 1);
					// printf("%f\n", distance);
					if (distance < max)
					{
						max = distance;
						L_corner_row = L_edge[i].row;
						L_corner_col = L_edge[i].col;
						L_beh_i = i;
					}
					dist_sum_left_bottom += distance; //求总体差值判断是否为直线
				}
				L_corner_angle = max;
				if (dist_sum_left_bottom < 0)
					dot_left_bottom_l = 1; //左拐线
				else
					dot_left_bottom_r = 1; //右拐线
										   //				if (dot_left_bottom_l)printf("turn left\n");
										   //				if (dot_left_bottom_r)printf("turn right\n");
				// printf("distance bottom left:%f\n", dist_sum_left_bottom);

				//左上
				L_top_corner_start = m3;
				if (L_edge_count - L_top_corner_start > effective_points)
					L_edge_count = L_top_corner_start + effective_points;
				//				else L_edge_count = L_edge_count;
				//直线顶点坐标(L_edge[L_top_corner_start].col,L_edge[L_top_corner_start].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
				line_left_top[0] = (float)(L_edge[L_edge_count - 1].col - L_edge[L_top_corner_start].col) / (float)(L_edge[L_edge_count - 1].row - L_edge[L_top_corner_start].row); // k
				line_left_top[1] = (float)(L_edge[L_top_corner_start].col - line_left_top[0] * L_edge[L_top_corner_start].row);														// b
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
				//	printf("m1m2%d,%d\n", m1, m2);
				for (int32_t i = m1; i < m2; i++)
				{
					L_edge_use[i].row = i;
					L_edge_use[i].col = (int32_t)(i * line_left_top[0] + line_left_top[1]);
					// printf("hh(%d,%d)\n", L_edge_use[i].row, L_edge_use[i].col);
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
				dot_left_top_r = 0;

				for (int32_t i = L_top_corner_start; i < L_edge_count; i++)
				{
					distance = (float)(line_left_top[0] * L_edge[i].row - L_edge[i].col + line_left_top[1]) / sqrt(line_left_top[0] * line_left_top[0] + 1);
					// printf("dist:%f", distance);
					if (distance < max)
					{
						max = distance;
						center_lost_corner_row_l = L_edge[i].row;
						center_lost_corner_col_l = L_edge[i].col;
						L_top_i = i;
					}
					dist_sum_left_top += distance; //求总体差值判断是否为直线
				}
				L_center_lost_corner_angle = max;
				if (dist_sum_left_top < 0)
					dot_left_top_l = 1; //左拐线
				else
					dot_left_top_r = 1; //右拐线
										//				if (dot_left_top_l)printf("turn left\n");
										//				if (dot_left_top_r)printf("turn right\n");
										// printf("distance top left:%f\n", dist_sum_left_top);
			}

			// printf("左下拐点：(x=%d,y=%d)，角度=%d\n", L_corner_row, L_corner_col, L_corner_angle);
			// printf("左上拐点：(x=%d,y=%d)，角度=%d\n", center_lost_corner_row_l, center_lost_corner_col_l, L_center_lost_corner_angle);
			R_corner_flag = 0; //初始化变量
			R_corner_row = 0;
			R_corner_col = 0;
			R_corner_angle = 0;
			if (!R_start_lost && !center_lost_flag_r) //只有右下边线情况
			{
				if (R_edge_count > effective_points)
					R_edge_count = effective_points;
				//				else R_edge_count = R_edge_count;
				//直线顶点坐标(L_edge[0].col,L_edge[0].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
				//				printf("\n直线顶点为：(%d,%d),(%d,%d)", R_edge[0].row, R_edge[0].col, R_edge[R_edge_count - 1].row, R_edge[R_edge_count - 1].col);
				line_right_bottom[0] = (float)(R_edge[R_edge_count - 1].col - R_edge[0].col) / (float)(R_edge[R_edge_count - 1].row - R_edge[0].row); // k
				line_right_bottom[1] = (float)(R_edge[0].col - line_right_bottom[0] * R_edge[0].row);												  // b
				//				printf("  直线斜率：k=%f,b=%f\n", line_right_bottom[0], line_right_bottom[1]);																							 //显示该直线
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
				//				printf("m1m2:%d,%d", m1, m2);
				for (int32_t i = m1; i < m2; i++)
				{
					R_edge_use[i].row = i;
					R_edge_use[i].col = (int32_t)(i * line_right_bottom[0] + line_right_bottom[1]);
					// printf("hh(%d,%d)\n", L_edge_use[i].col, L_edge_use[i].row);
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
				// uint8_t m3;
				dist_sum_right_bottom = 0;
				dot_right_bottom_l = 0;
				dot_right_bottom_r = 0;
				is_line_right = 0;

				// printf("m3=%d\n", m3);
				for (int32_t i = 0; i < R_edge_count; i++)
				{
					distance = (float)(line_right_bottom[0] * R_edge[i].row - R_edge[i].col + line_right_bottom[1]) / sqrt(line_right_bottom[0] * line_right_bottom[0] + 1);
					//				printf("%f\n", distance);
					if (distance < max)
					{
						max = distance;
						R_corner_row = R_edge[i].row;
						R_corner_col = R_edge[i].col;
						R_beh_i = i;
					}
					dist_sum_right_bottom += distance; //求总体差值判断是否为直线
				}
				R_corner_angle = max;
				if (dist_sum_right_bottom < 0)
					dot_right_bottom_l = 1; //左拐线
				else
					dot_right_bottom_r = 1; //右拐线
				if ((dist_sum_right_bottom * (dist_sum_right_bottom < 0 ? -1 : 1)) < dist_control)
					is_line_right = 1;

				//				if (dot_right_bottom_l)printf("turn left\n");
				//				if (dot_right_bottom_r)printf("turn right\n");
				//				printf("distance bottom right:%f\n", dist_sum_right_bottom);
			}
			else if (R_start_lost) //只有右上边线情况
			{
				if (R_edge_count > effective_points)
					R_edge_count = effective_points;
				//				else R_edge_count = R_edge_count;
				//直线顶点坐标(L_edge[1].col,L_edge[1].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
				printf("\n直线顶点为：(%d,%d),(%d,%d)", R_edge[1].row, R_edge[1].col, R_edge[R_edge_count - 1].row, R_edge[R_edge_count - 1].col);
				line_right_top[0] = (float)(R_edge[R_edge_count - 1].col - R_edge[1].col) / (float)(R_edge[R_edge_count - 1].row - R_edge[1].row); // k
				line_right_top[1] = (float)(R_edge[1].col - line_right_top[0] * R_edge[1].row);													   // b
				printf("  直线斜率：k=%f,b=%f\r\n", line_right_top[0], line_right_top[1]);
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
				//				printf("  m1=%d,m2=%d\n", m1, m2);
				for (int32_t i = m1; i < m2; i++)
				{
					R_edge_use[i].row = (uint8_t)i;
					R_edge_use[i].col = (int32_t)(i * line_right_top[0] + line_right_top[1]);
					// printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
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
				// uint8_t m3;
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
					dist_sum_right_top += distance; //求总体差值判断是否为直线
				}
				R_center_lost_corner_angle = max;
				if (dist_sum_right_top < 0)
					dot_right_top_l = 1; //左拐线
				else
					dot_right_top_r = 1; //右拐线
				if ((dist_sum_right_top * (dist_sum_right_top < 0 ? -1 : 1)) < dist_control)
					is_line_right = 1;

				//				if (dot_right_top_l)printf("turn left\n");
				//				if (dot_right_top_r)printf("turn right\n");
				//				printf("distance top right:%f\n", dist_sum_right_top);
			}
			else if (center_lost_flag_r) //有右上和右下边线情况   #TODO  增加对distance正负的判断
			{
				//右下
				uint8_t m3 = R_top_corner_start;
				if (R_top_corner_start > effective_points)
					R_top_corner_start = effective_points;
				//				else R_top_corner_start = R_top_corner_start;
				//直线顶点坐标(L_edge[0].col,L_edge[0].row),(L_edge[L_top_corner_start-1].col,L_edge[L_top_corner_start-1].row)
				line_right_bottom[0] = (float)(R_edge[R_top_corner_start - 1].col - R_edge[0].col) / (float)(R_edge[R_top_corner_start - 1].row - R_edge[0].row); // k
				line_right_bottom[1] = (float)(R_edge[0].col - line_right_bottom[0] * R_edge[0].row);															  // b
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
					// printf("hh(%d,%d)\n", L_edge_use[i].row, L_edge_use[i].col);
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
				// uint8_t m3;
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
					dist_sum_right_bottom += distance; //求总体差值判断是否为直线
				}
				R_corner_angle = max;
				if (dist_sum_right_bottom < 0)
					dot_right_bottom_l = 1; //左拐线
				else
					dot_right_bottom_r = 1; //右拐线
											//				if (dot_right_bottom_l)printf("turn left\n");
											//				if (dot_right_bottom_r)printf("turn right\n");
											//				printf("distance bottom right:%f\n", dist_sum_right_bottom);

				//右上
				R_top_corner_start = m3;
				if (R_edge_count - R_top_corner_start > effective_points)
					R_edge_count = R_top_corner_start + effective_points;
				//				else R_edge_count = R_edge_count;
				//直线顶点坐标(L_edge[L_top_corner_start].col,L_edge[L_top_corner_start].row),(L_edge[L_edge_count-1].col,L_edge[L_edge_count-1].row)
				line_right_top[0] = (float)(R_edge[R_edge_count - 1].col - R_edge[R_top_corner_start].col) / (float)(R_edge[R_edge_count - 1].row - R_edge[R_top_corner_start].row); // k
				line_right_top[1] = (float)(R_edge[R_top_corner_start].col - line_right_top[0] * R_edge[R_top_corner_start].row);													 // b
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
					// printf("hh(%d,%d)\n", L_edge_use[i].row, L_edge_use[i].col);
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
					// printf("dist:%f", distance);
					if (distance > max)
					{
						max = distance;
						center_lost_corner_row_r = R_edge[i].row;
						center_lost_corner_col_r = R_edge[i].col;
						R_top_i = i;
					}
					dist_sum_right_top += distance; //求总体差值判断是否为直线
				}
				R_center_lost_corner_angle = max;
				if (dist_sum_right_top < 0)
					dot_right_top_l = 1; //左拐线
				else
					dot_right_top_r = 1; //右拐线
										 //				if (dot_right_top_l)printf("turn left\n");
										 //				if (dot_right_top_r)printf("turn right\n");
										 //				printf("distance top right:%f\n", dist_sum_right_top);
			}

			//			printf("右下拐点：(x=%d,y=%d)，角度=%d\n", R_corner_row, R_corner_col, R_corner_angle);
			//			printf("右上拐点：(x=%d,y=%d)，角度=%d\n", center_lost_corner_row_r, center_lost_corner_col_r, R_center_lost_corner_angle);
		}
	}

	//	//判断元素，生成最终的边线数组
	//	if (tjrc_imageProc_ctr.enable_element)
	//	{
	//		if (dot_left_bottom_l && dot_right_bottom_r && dot_right_top_r && dot_left_top_l)//正十字
	//		{
	//			printf("this is zheng shi zi\n");
	////			sprintf((char*)str_buff,"cnt=%d   ",cnt++);
	//			tjrc_st7735_dispStr612(5,50,"zheng shi zi",0xFFFF);
	//			//printf("\n直线顶点为：(%d,%d),(%d,%d)",L_corner_col, L_corner_row, center_lost_corner_col_l, center_lost_corner_row_l);
	//			line_left_top[0] = (float)(center_lost_corner_col_l - L_corner_col) / (float)(center_lost_corner_row_l - L_corner_row);   //k
	//			line_left_top[1] = (float)(L_corner_col - line_left_top[0] * L_corner_row);  //b
	//			//printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
	//			//显示该直线
	//			uint8_t m1, m2;
	//			if (L_corner_row > center_lost_corner_row_l)
	//			{
	//				m1 = center_lost_corner_row_l;
	//				m2 = L_corner_row;
	//			}
	//			else
	//			{
	//				m2 = center_lost_corner_row_l;
	//				m1 = L_corner_row;
	//			}
	//			//printf("  m1=%d,m2=%d\n", m1, m2);
	//			for (int32_t i = m1; i < m2; i++)
	//			{
	//				L_edge_use[i].row = i;
	//				L_edge_use[i].col = (int32_t)(i * line_left_top[0] + line_left_top[1]);
	//				//printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
	//				if (L_edge_use[i].col < 0)
	//				{
	//					L_edge_use[i].col = 0;
	//				}
	//				if (L_edge_use[i].col > 119)
	//				{
	//					L_edge_use[i].col = 119;
	//				}
	//			}
	//			for (int32_t i = L_beh_i; i > 0; i--)
	//			{
	//				L_edge_use[m2].row = L_edge[i].row;
	//				L_edge_use[m2].col = L_edge[i].col;
	//				m2++;
	//			}
	//			for (int32_t i = L_top_i; i < L_edge_count; i++)
	//			{
	//				L_edge_use[m1].row = L_edge[i].row;
	//				L_edge_use[m1].col = L_edge[i].col;
	//				m1--;
	//			}
	//
	//			line_right_top[0] = (float)(center_lost_corner_col_r - R_corner_col) / (float)(center_lost_corner_row_r - R_corner_row);   //k
	//			line_right_top[1] = (float)(R_corner_col - line_right_top[0] * R_corner_row);  //b
	//			//printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
	//			//显示该直线
	//			if (R_corner_row > center_lost_corner_row_r)
	//			{
	//				m1 = center_lost_corner_row_r;
	//				m2 = R_corner_row;
	//			}
	//			else
	//			{
	//				m2 = center_lost_corner_row_r;
	//				m1 = R_corner_row;
	//			}
	//			//printf("  m1=%d,m2=%d\n", m1, m2);
	//			for (int32_t i = m1; i < m2; i++)
	//			{
	//				R_edge_use[i].row = i;
	//				R_edge_use[i].col = (int32_t)(i * line_right_top[0] + line_right_top[1]);
	//				//printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
	//				if (R_edge_use[i].col < 0)
	//				{
	//					R_edge_use[i].col = 0;
	//				}
	//				if (R_edge_use[i].col > 119)
	//				{
	//					R_edge_use[i].col = 119;
	//				}
	//			}
	//			for (int32_t i = R_beh_i; i > 0; i--)
	//			{
	//				R_edge_use[m2].row = R_edge[i].row;
	//				R_edge_use[m2].col = R_edge[i].col;
	//				m2++;
	//			}
	//			for (int32_t i = R_top_i; i < R_edge_count; i++)
	//			{
	//				R_edge_use[m1].row = R_edge[i].row;
	//				R_edge_use[m1].col = R_edge[i].col;
	//				m1--;
	//			}
	//		}
	//		if (!dot_left_bottom_l && !dot_right_bottom_r && dot_right_top_r && dot_left_top_l)//中十字
	//		{
	//			printf("this is zhong shi zi\n");
	//			tjrc_st7735_dispStr612(5,50,"zhong shi zi",0xFFFF);
	//			//edge_truncation(0);
	//			FitStraightLine(L_top_i, L_top_i + 10, 0);
	//			printf("k_l:%f,b_l:%f\n", k_l, b_l);
	//			for (int32_t i = L_edge[L_top_i].row; i < 80; i++)
	//			{
	//				L_edge_use[i].row = i;
	//				L_edge_use[i].col = (int32_t)(i * k_l + b_l);
	//				//printf("hh(%d,%d)\n", L_edge_use[i].row, L_edge_use[i].col);
	//				if (L_edge_use[i].col < 0)
	//				{
	//					L_edge_use[i].col = 0;
	//				}
	//				if (L_edge_use[i].col > 119)
	//				{
	//					L_edge_use[i].col = 119;
	//				}
	//			}
	//			uint8_t m1 = L_edge[L_top_i].row;
	//			for (int32_t i = L_top_i; i < L_edge_count; i++)
	//			{
	//				L_edge_use[m1].row = L_edge[i].row;
	//				L_edge_use[m1].col = L_edge[i].col;
	//				m1--;
	//			}
	//
	//			//edge_truncation(1);
	//			FitStraightLine(R_top_i, R_top_i + 10, 1);
	//			printf("k_r:%f,b_r:%f\n", k_r, b_r);
	//			for (int32_t i = R_edge[R_top_i].row; i < 80; i++)
	//			{
	//				R_edge_use[i].row = i;
	//				R_edge_use[i].col = (int32_t)(i * k_r + b_r);
	//				if (R_edge_use[i].col > 119)
	//				{
	//					R_edge_use[i].col = 119;
	//				}
	//				if (R_edge_use[i].col < 0)
	//				{
	//					R_edge_use[i].col = 0;
	//				}
	//			}
	//			uint8_t m2 = R_edge[R_top_i].row;
	//			for (int32_t i = R_top_i; i < R_edge_count; i++)
	//			{
	//				R_edge_use[m2].row = R_edge[i].row;
	//				R_edge_use[m2].col = R_edge[i].col;
	//				m2--;
	//			}
	//		}
	//
	//		if (!dot_left_bottom_l && dot_right_bottom_r && dot_right_top_l && dot_left_top_r)//左斜入十字
	//		{
	//			printf("this is zuo xieru shi zi\n");
	//		}
	//
	//		if (dot_left_bottom_l && !dot_right_bottom_r && dot_right_top_l && dot_left_top_r)//右斜入十字
	//		{
	//			printf("this is you xieru shi zi\n");
	//		}
	//
	//		if (dot_left_bottom_l && dot_left_top_l && is_line_right)//左圆环
	//		{
	//			printf("this is zuo yuan huan\n");
	//			tjrc_st7735_dispStr612(5,50,"zuo yuan huan",0xFFFF);
	//			//printf("\n直线顶点为：(%d,%d),(%d,%d)",L_corner_col, L_corner_row, center_lost_corner_col_l, center_lost_corner_row_l);
	//			line_left_top[0] = (float)(center_lost_corner_col_l - L_corner_col) / (float)(center_lost_corner_row_l - L_corner_row);   //k
	//			line_left_top[1] = (float)(L_corner_col - line_left_top[0] * L_corner_row);  //b
	//			//printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
	//			//显示该直线
	//			uint8_t m1, m2;
	//			if (L_corner_row > center_lost_corner_row_l)
	//			{
	//				m1 = center_lost_corner_row_l;
	//				m2 = L_corner_row;
	//			}
	//			else
	//			{
	//				m2 = center_lost_corner_row_l;
	//				m1 = L_corner_row;
	//			}
	//			//printf("  m1=%d,m2=%d\n", m1, m2);
	//			for (int32_t i = m1; i < m2; i++)
	//			{
	//				L_edge_use[i].row = i;
	//				L_edge_use[i].col = (int32_t)(i * line_left_top[0] + line_left_top[1]);
	//				//printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
	//				if (L_edge_use[i].col < 0)
	//				{
	//					L_edge_use[i].col = 0;
	//				}
	//				if (L_edge_use[i].col > 119)
	//				{
	//					L_edge_use[i].col = 119;
	//				}
	//			}
	//			for (int32_t i = L_beh_i; i > 0; i--)
	//			{
	//				L_edge_use[m2].row = L_edge[i].row;
	//				L_edge_use[m2].col = L_edge[i].col;
	//				m2++;
	//			}
	//			for (int32_t i = L_top_i; i < L_edge_count; i++)
	//			{
	//				L_edge_use[m1].row = L_edge[i].row;
	//				L_edge_use[m1].col = L_edge[i].col;
	//				m1--;
	//			}
	//			m1 = 79;
	//			for (int32_t i = 0; i < 75; i++)
	//			{
	//				R_edge_use[m1].row = R_edge[i].row;
	//				R_edge_use[m1].col = R_edge[i].col;
	//				m1--;
	//			}
	//		}
	//
	//		if (dot_right_bottom_r && dot_right_top_r && is_line_left)//右圆环
	//		{
	//			printf("this is you yuan huan\n");
	//			tjrc_st7735_dispStr612(5,50,"you yuan huan",0xFFFF);
	//			line_right_top[0] = (float)(center_lost_corner_col_r - R_corner_col) / (float)(center_lost_corner_row_r - R_corner_row);   //k
	//			line_right_top[1] = (float)(R_corner_col - line_right_top[0] * R_corner_row);  //b
	//			//printf("  直线斜率：k=%f,b=%f\n", line_left_top[0], line_left_top[1]);
	//			//显示该直线
	//			uint8_t m1, m2;
	//			if (R_corner_row > center_lost_corner_row_r)
	//			{
	//				m1 = center_lost_corner_row_r;
	//				m2 = R_corner_row;
	//			}
	//			else
	//			{
	//				m2 = center_lost_corner_row_r;
	//				m1 = R_corner_row;
	//			}
	//			//printf("  m1=%d,m2=%d\n", m1, m2);
	//			for (int32_t i = m1; i < m2; i++)
	//			{
	//				R_edge_use[i].row = i;
	//				R_edge_use[i].col = (int32_t)(i * line_right_top[0] + line_right_top[1]);
	//				//printf("hh(%d,%d)\n", R_edge_use[i].col, R_edge_use[i].row);
	//				if (R_edge_use[i].col < 0)
	//				{
	//					R_edge_use[i].col = 0;
	//				}
	//				if (R_edge_use[i].col > 119)
	//				{
	//					R_edge_use[i].col = 119;
	//				}
	//			}
	//			for (int32_t i = R_beh_i; i > 0; i--)
	//			{
	//				R_edge_use[m2].row = R_edge[i].row;
	//				R_edge_use[m2].col = R_edge[i].col;
	//				m2++;
	//			}
	//			for (int32_t i = R_top_i; i < R_edge_count; i++)
	//			{
	//				R_edge_use[m1].row = R_edge[i].row;
	//				R_edge_use[m1].col = R_edge[i].col;
	//				m1--;
	//			}
	//			m1 = 79;
	//			for (int32_t i = 0; i < 75; i++)
	//			{
	//				L_edge_use[m1].row = L_edge[i].row;
	//				L_edge_use[m1].col = L_edge[i].col;
	//				m1--;
	//			}
	//		}
	//	}
	//
	//
	//	//计算中线
	//	for (int32_t i = 0; i < 80; i++)
	//	{
	//		Mid_Line[i].col = (L_edge_use[i].col + R_edge_use[i].col) / 2;
	//		Mid_Line[i].row = i;
	//	}
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
	uint8_t mid_cnt = 0;		//用于向下拟合，如果开启拟合，则用于记录新的中线点个数
	uint8_t last_mid_count = 0; //用于记录上一场中线的中线个数，用于中线平缓
	uint8_t up_cnt = 15;		//向上生长的限制个数
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
			if (col > Image_W - 1)
				col = Image_W - 1;
			else if (col < 0)
				col = 0;
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
			if (col > Image_W - 1)
				col = Image_W - 1;
			else if (col < 0)
				col = 0;
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
			for (int32_t i = M_Line[0].row; i < Image_H; i++) //开始向下拟合
			{
				int32_t col = K * i + B;
				if (col > Image_W - 1)
					col = Image_W - 1;
				else if (col < 0)
					col = 0;
				MID_LINE[mid_cnt].row = (uint8_t)i;
				MID_LINE[mid_cnt].col = (uint8_t)col;
				mid_cnt = mid_cnt + 1;
				if (--down_cnt == 0)
					break;
			}
		}
		//中线点个数太少，开启向上拟合
		if (Mid_count + mid_cnt < 60 && Mid_count > 2)
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
			for (int32_t i = M_Line[Mid_count - 1].row; i > 0; i--) //开始向上拟合
			{
				int32_t col = K * i + B;
				if (col > Image_W - 1)
					col = Image_W - 1;
				else if (col < 0)
					col = 0;
				M_Line[Mid_count].row = (uint8_t)i;
				M_Line[Mid_count].col = (uint8_t)col;
				Mid_count = Mid_count + 1;
				if (--up_cnt == 0)
					break;
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

/**
 * @breif 初始化左边界右边界结构体
 * @retrun NONE
 */
static void clear_point(void)
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
// int32_t Get_angle(uint8_t ax, uint8_t ay, uint8_t bx, uint8_t by, uint8_t cx, uint8_t cy)
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
// }
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
int32_t edge_point_ornot(uint8_t image[MT9V03X_H][MT9V03X_W], uint8_t row, uint8_t side) // 5和155需要调整
{
	//左边线
	if (side == 0)
	{

		uint8_t find_edge = 0;
		L_lost_count = 0;
		rt_kprintf("%d,%d\r\n", row, side);
		//从开始搜线行向上生长
		for (uint8_t rowi = row; rowi > 0; rowi--)
		{
			//如果图像最左侧为赛道外（未丢线），则开始横向生长
			if (L_lost_count <= L_lost_)
			{
				//横向生长
				for (int32_t col = L_edge_start_col; col < Image_W / 2; col++)
				{
					//					rt_kprintf("image[%d][%d],is black:%d\r\n", rowi, col, black_(image[rowi][col]));
					if (black_(image[rowi][col]))
					{
						// printf("col:%d\n ", col);
						//如果出现黑黑白白，则判断为边界线，退出循环
						if (black_(image[rowi][col]) && black_(image[rowi][col + 1]) && white_(image[rowi][col + 2]) && white_(image[rowi][col + 3]))
						{
							L_basic_row_start = rowi; //赋值开始搜线行（左）
													  //							printf("左边线起始点：(x=%d,y=%d)\n", rowi, col + 1);
							// if (rowi<Image_H / 2 && col>Image_W / 2) return -1;
							return col + 1; //返回列值         //!!!   1->2
						}
					}
					if (col == Image_W / 2 - 1)
					{
						L_lost_count++;
						// printf("L_lost_count:%d\n", L_lost_count);
					}
				}
			}
			// printf("lost %d ", L_lost_count);
			if (L_lost_count > L_lost_) //丢线大于次数限制开启行搜索
			{
				//如果出现黑黑白白，则判断为边界线，退出循环
				if (black_(image[rowi - 3][L_edge_start_col]) && black_(image[rowi - 2][L_edge_start_col]) && white_(image[rowi - 1][L_edge_start_col]) && white_(image[rowi][L_edge_start_col]))
				{
					L_basic_row_start = rowi - 2; //赋值开始搜线行（左）
					printf("丢线情况，左边线起始点：(x=%d,y=%d)\n", L_basic_row_start, L_edge_start_col);
					L_start_lost = 1;
					// if (rowi<Image_H / 2 && col>Image_W / 2) return -1;
					return L_edge_start_col; //返回列值         //!!!   1->2
				}
			}
			if (find_edge == 2)
				return -1;
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
							if (rowi < Image_H / 2 && col < Image_W / 2)
								return -1;
							return col - 1;
						}
					}
					if (col == Image_W / 2 + 1)
					{
						R_lost_count++;
					}
				}
			}

			if (R_lost_count > R_lost_) //丢线大于次数限制开启行搜索
			{
				if (black_(image[rowi - 3][R_edge_start_col]) && black_(image[rowi - 2][R_edge_start_col]) && white_(image[rowi - 1][R_edge_start_col]) && white_(image[rowi][R_edge_start_col]))
				{
					R_basic_row_start = rowi - 2; //赋值开始搜线行（左）
					printf("丢线情况，右边线起始点：(x=%d,y=%d)\n", R_basic_row_start, R_edge_start_col);
					R_start_lost = 1;
					// if (rowi<Image_H / 2 && col>Image_W / 2) return -1;
					return R_edge_start_col; //返回列值         //!!!   1->2
				}
			}
			if (find_edge == 2)
				return -1;
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
uint8_t black_(uint8_t x) //判断黑
{
	if (binimage_flag == 0) //如果没有二值化，则通过阈值判断黑白
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

uint8_t white_(uint8_t x) //判断白
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
///*--------------------------------------------------------------------------
//* 【函数功能】：获取二值化图像
//* 【参    数】：无
//* 【返 回 值】：无
//* 【备    注】：无
//*--------------------------------------------------------------------------*/
// void get_binImage(uint8_t thres)
//{
//	binimage_flag = 1;  //二值化标志置为1
//	for (int32_t row = 0; row < Image_H; row++)
//	{
//		for (int32_t col = 0; col < Image_W; col++)
//		{
//			if (image[row][col] < thres)
//			{
//				image[row][col] = 0;    //小于阈值赋值为0，显示为黑色
//			}
//			else
//				image[row][col] = 1;    //大于阈值赋值为1，显示为白色
//		}
//	}
//}
/*--------------------------------------------------------------------------
 * 【函数功能】：大津法求动态阈值
 * 【参    数】：无
 * 【返 回 值】：无
 *--------------------------------------------------------------------------*/
#if 0
#define GrayScale 256 // frame

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

// 点集三角滤波
void blur_points(struct LEFT_EDGE pts_in[120], struct LEFT_EDGE pts_out[120], int32_t num, int32_t kernel)
{
	int32_t half = kernel / 2;
	for (int32_t i = 0; i < num; i++)
	{
		pts_out[i].row = pts_out[i].col = 0;
		for (int32_t j = -half; j <= half; j++)
		{
			pts_out[i].row += pts_in[clip(i + j, 0, num - 1)].row * (half + 1 - abs(j));
			pts_out[i].col += pts_in[clip(i + j, 0, num - 1)].col * (half + 1 - abs(j));
		}
		pts_out[i].row /= (2 * half + 2) * (half + 1) / 2;
		pts_out[i].col /= (2 * half + 2) * (half + 1) / 2;
		// printf("row:%d col:%d\n", pts_out[i].row);
	}
}

int32_t clip(int32_t x, int32_t low, int32_t up)
{
	return x > up ? up : x < low ? low
								 : x;
}

float fclip(float x, float low, float up)
{
	return x > up ? up : x < low ? low
								 : x;
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
			// FitStraightLine(L_beh_i - 9, L_beh_i+1, 0);
			// k_l = (k_l + k) / 2.0;
			// b_l = (b_l + b) / 2.0;
			printf("左中间丢线:k=%f,b=%f\n", k_l, b_l);
		}
		else //左边未丢线情况
		{
			FitStraightLine(L_beh_i - 9, L_beh_i + 1, 0);
			printf("左边未丢线:k=%f,b=%f\n", k_l, b_l);
		}
		// if (L_edge_count >= 70)
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
		// }
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
			// FitStraightLine(R_beh_i - 9, R_beh_i+1, 1);
			// k_r = (k_r + k) / 2.0;
			// b_r = (b_r + b) / 2.0;
			printf("右中间丢线:k=%f,b=%f\n", k_r, b_r);
		}
		else //左边未丢线情况
		{
			FitStraightLine(R_beh_i - 9, R_beh_i + 1, 1);
			printf("右边未丢线:k=%f,b=%f\n", k_r, b_r);
		}
		// if (L_edge_count >= 70)
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
		// }
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
			// printf("(%d,%d)\n", L_edge[i].row, L_edge[i].col);
			x_add += L_edge[i].row;
			y_add += L_edge[i].col;
			xx_add += (L_edge[i].row * L_edge[i].row);
			xy_add += (L_edge[i].row * L_edge[i].col);
		}

		x_bar = x_add * 1.0 / n;
		y_bar = y_add * 1.0 / n;
		xx_bar = xx_add * 1.0 / n;
		xy_bar = xy_add * 1.0 / n;
		// printf("kf=%f", (xy_bar - x_bar * y_bar) / (xx_bar - x_bar * x_bar));

		k_l = (xy_bar - x_bar * y_bar) / (xx_bar - x_bar * x_bar); //拟合直线斜率
		b_l = y_bar - k_l * x_bar;								   //拟合直线的常数项
	}
	if (side == 1)
	{
		for (int32_t i = start; i < end; i++)
		{
			// printf("(%d,%d)\n", R_edge[i].row, R_edge[i].col);
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
		b_r = y_bar - k_r * x_bar;								   //拟合直线的常数项
	}
}

// void variance(LEFT_EDGE side[], uint8_t start, uint8_t end)
//{
//	int32_t sum=0;
//	for (int32_t i = start; i < end; i++)
//	{
//		sum += side[i].col;
//	}
// }

// 左手迷宫巡线  //y=115,x=30
void findline_lefthand_adaptive(uint8_t image[MT9V03X_H][MT9V03X_W], int32_t jilu_row, int32_t jilu_col, int32_t search_amount)
{
	L_edge[0].row = jilu_row;
	L_edge[0].col = jilu_col;
	L_edge[0].flag = 1;
	uint8_t curr_row = jilu_row; //初始化行坐标
	uint8_t curr_col = jilu_col; //初始化列坐标
	dire_left = 0;				 //初始化上个边界点的来向
	center_turn_flag = 1;		 //初始化在未标定状态
	center_lost_flag_l = 0;		 //中间左丢线标志位
	//开始搜线，最多取150个点，不会往下搜，共7个方位
	for (int32_t i = 1; i < search_amount; i++) //最多搜索70个点
	{
		////越界退出 行越界和列越界（向上向下向左向右）
		if (curr_row < L_edge_end_row || curr_row > Image_H - 1 || curr_row + 1 < L_edge_end_row)
			break;
		if (curr_col > max_col || curr_col < min_col && !center_lost_flag_l) // max=160-5 min=5
		{
			if (++L_search_edge_count == 3) //连续3次搜索到边界，退出      //连续搜索到边界次数需要调整  5和155需要调整
			{
				curr_col = L_edge_lost_start_col;

				for (uint8_t rowi = curr_row; rowi > 0; rowi--)
				{
					// printf("row=%d", rowi);
					if (black_(image[rowi - 3][curr_col]) && black_(image[rowi - 2][curr_col]) && white_(image[rowi - 1][curr_col]) && white_(image[rowi][curr_col]))
					{
						printf("中间左丢线情况，左边线起始点：(x=%d,y=%d)\n", rowi - 1, L_edge_lost_start_col);
						curr_row = rowi - 1;
						curr_col = L_edge_lost_start_col;
						center_lost_flag_l = 1;
						center_lost_row_l = curr_row;	  //中间左丢线开始行坐标
						center_lost_col_l = curr_col + 1; //中间左丢线开始列坐标
						dire_left = 0;					  //初始化上个边界点的来向
						L_top_corner_start = i;			  //左上拐点开始序号
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
			// turn++;
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
		// printf("(x1=%d,y1=%d)\n", L_edge[i].row, L_edge[i].col);
		// dd = dd + 1;
	}
	// printf("count%d ", L_edge_count);
}

// 右手迷宫巡线  //y=115,x=30
void findline_righthand_adaptive(uint8_t image[MT9V03X_H][MT9V03X_W], int32_t jilu_row, int32_t jilu_col, int32_t search_amount)
{
	R_edge[0].row = jilu_row;
	R_edge[0].col = jilu_col;
	R_edge[0].flag = 1;
	uint8_t curr_row = jilu_row; //初始化行坐标
	uint8_t curr_col = jilu_col; //初始化列坐标
	dire_right = 0;				 //初始化上个边界点的来向
	center_turn_flag = 1;		 //初始化在未标定状态
	center_lost_flag_r = 0;		 //中间左丢线标志位
	//开始搜线，最多取150个点，不会往下搜，共7个方位
	for (int32_t i = 1; i < search_amount; i++) //最多搜索70个点
	{
		////越界退出 行越界和列越界（向上向下向左向右）
		if (curr_row < R_edge_end_row || curr_row > Image_H - 1 || curr_row + 1 < R_edge_end_row)
			break;
		if (curr_col > max_col || curr_col < min_col && !center_lost_flag_r) // max=160-5 min=5
		{
			if (++R_search_edge_count == 3) //连续3次搜索到边界，退出      //连续搜索到边界次数需要调整  5和155需要调整
			{
				curr_col = R_edge_lost_start_col;

				for (uint8_t rowi = curr_row; rowi > 0; rowi--)
				{
					// printf("row=%d", rowi);
					if (black_(image[rowi - 3][curr_col]) && black_(image[rowi - 2][curr_col]) && white_(image[rowi - 1][curr_col]) && white_(image[rowi][curr_col]))
					{
						printf("中间右丢线情况，右边线起始点：(x=%d,y=%d)\n", rowi - 1, R_edge_lost_start_col);
						curr_row = rowi - 1;
						curr_col = R_edge_lost_start_col;
						center_lost_flag_r = 1;
						center_lost_row_r = curr_row;	  //中间左丢线开始行坐标
						center_lost_col_r = curr_col + 1; //中间左丢线开始列坐标
						dire_right = 0;					  //初始化上个边界点的来向
						R_top_corner_start = i;			  //左上拐点开始序号
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
			// turn++;
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
		// printf("(x1=%d,y1=%d)\n", L_edge[i].row, L_edge[i].col);
		// dd = dd + 1;
	}
	// printf("count%d ", L_edge_count);
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
	return 0;
}
