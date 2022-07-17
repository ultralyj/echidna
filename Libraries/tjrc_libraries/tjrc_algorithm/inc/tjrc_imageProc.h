
#ifndef __TJRC_IMAGEPROC_H__
#define __TJRC_IMAGEPROC_H__

#include "stdio.h"
#include "stdint.h"
#include "stdint.h"
#include "math.h"

#include "tjrc_algorithm.h"
/**
 * @brief 图像宽度与高度
 */
#define IMAGE_WIDTH     120
#define IMAGE_HEIGHT    80

#define LCDW 120
#define LCDH 80
/**
 * @brief 画面左右两侧常有噪点与黑线干扰，因此需要对其进行裁切(3,width-3)
 */
#define IMAGE_COL_KEEPOUT_PIXEL 3

/**
 * @brief 图像感兴趣的高度（0-i）
 */
#define IMAGE_INTEREST_REGION 70

/**
 * @brief 图像颜色宏定义
 */
#define IMAGE_COLOR_BLACK 0x00
#define IMAGE_COLOR_WHITE 0xff

 /**
  * @brief 判断拐点的阈值，影响判断拐点的灵敏度
  */
#define INFLECTION_THRESHOLD 10

#define INFLECTION_ADJUST_PIXEL 2
  /**
   * @brief 上下两拐点最小竖直距离
   */
#define INFLECTION_MINIMUM_DISTANCE 10


#define LINE_INFO_MAX_PIXEL 80
typedef struct
{
    uint8_t pixel_cnt;
    uint8_t x_left_edgeCnt;
    uint8_t x_right_edgeCnt;
    /* index对应图像像素的y坐标，如当前点为(10,20)，则x_left[20] = 10 */
    uint8_t x_left[LINE_INFO_MAX_PIXEL];
    uint8_t x_right[LINE_INFO_MAX_PIXEL];
    /* 左右边线搜到线标志，丢线为0 */
    uint8_t x_left_findEdge[LINE_INFO_MAX_PIXEL];
    uint8_t x_right_findEdge[LINE_INFO_MAX_PIXEL];
}line_info;

enum inflection_bitSet
{
    inflection_upper_left = 0x01,
    inflection_lower_left = 0x02,
    inflection_upper_right = 0x04,
    inflection_lower_right = 0x08,
    inflection_upper = 0x05,
    inflection_lower = 0x0A,
    inflection_left = 0x03,
    inflection_right = 0x0C,
    inflection_all = 0x0F
};

typedef struct
{
    uint8_t findFlag;
    uint8_t x_upper_left;
    uint8_t y_upper_left;
    uint8_t x_lower_left;
    uint8_t y_lower_left;
    uint8_t x_upper_right;
    uint8_t y_upper_right;
    uint8_t x_lower_right;
    uint8_t y_lower_right;
}inflection_info;


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
    uint8_t line_amount;
    uint8_t edge_count_amount;

    //###标志位参数列表
    uint8_t enable_bin;   //使能二值化标志位
    uint8_t binimage_flag;  //二值化方式标志位
    uint8_t enable_balinyu; //使能八邻域搜线标志位
    uint8_t enable_turnpoint;  //使能拐点搜索标准位
    uint8_t enable_element;  //使能识别元素标志位

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
void get_deal_image();      //取出待处理图像
int32_t average_value(uint8_t* c);//求均值（均值滤波用）
void average_filter(void);  //均值滤波
uint8_t OSTU_bin(uint8_t width, uint8_t height, uint8_t* Image);//大津法求动态阈值
uint8_t GetOSTUThreshold(uint8_t(*img)[LCDW], uint16_t start_row, uint16_t end_row, uint16_t start_col, uint16_t end_col); //大津法求动态阈值
void get_binImage(uint8_t thres);//二值化
uint8_t black_(uint8_t x);  //判断是否是黑像素点
uint8_t white_(uint8_t x);  //判断是否是白像素点
uint8_t black_2(uint8_t* image,uint8_t row,uint8_t col);
uint8_t white_2(uint8_t* image,uint8_t row,uint8_t col);
int32_t edge_point_ornot(uint8_t image[MT9V03X_H][MT9V03X_W],uint8_t row, uint8_t side);//判断是否存在边界点,并返回【-1：没有找到边界点，正值：返回找到的边界点 】
int32_t Get_angle(uint8_t ax, uint8_t ay, uint8_t bx, uint8_t by, uint8_t cx, uint8_t cy);//求取拐角的角度值
void clear_point();     //清空结构体数据
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
void git_gui_value(uint8_t value_in[30]);
float statistics(uint8_t mode);
void tjrc_imageProcess(uint8_t* image, tjrc_image_info* info);
uint8_t XLW_otsuThreshold(uint8_t* image, uint16_t col, uint16_t row);
int32_t GetOSTU(uint8_t* tmImage);
void SobelAutoThreshold(uint8_t* image,uint8_t* imageout);
int tjrc_findstartpoint_row(uint8_t image[MT9V03X_H][MT9V03X_W],uint8_t row, uint8_t side);
void tjrc_findstartpoint_col(uint8_t image[MT9V03X_H][MT9V03X_W],uint8_t row, uint8_t col, uint8_t side);
void tjrc_findline_lefthand(uint8_t image[MT9V03X_H][MT9V03X_W],int jilu_row, int jilu_col, int search_amount, int start);
void tjrc_findline_righthand(uint8_t image[MT9V03X_H][MT9V03X_W],int jilu_row, int jilu_col, int search_amount, int start);
void tjrc_findturnpoint_leftbottom(int start, int end, int search_amount);
void tjrc_findturnpoint_lefttop(int start, int end, int search_amount);
void tjrc_findturnpoint_righttop(int start, int end, int search_amount);
void tjrc_findturnpoint_rightbottom(int start, int end, int search_amount);
void tjrc_greekcross_patchLine(uint8_t L_corner_row, uint8_t L_corner_col, uint8_t center_lost_corner_col_l, uint8_t center_lost_corner_row_l, uint8_t R_corner_row, uint8_t R_corner_col, uint8_t center_lost_corner_col_r, uint8_t center_lost_corner_row_r);
void tjrc_centercross_patchLine(uint8_t L_top_i, uint8_t R_top_i);
void tjrc_leftring_patchLine(uint8_t L_corner_row, uint8_t L_corner_col, uint8_t center_lost_corner_col_l, uint8_t center_lost_corner_row_l);
void tjrc_rightring_patchLine(uint8_t R_corner_row, uint8_t R_corner_col, uint8_t center_lost_corner_col_r, uint8_t center_lost_corner_row_r);
void tjrc_leftturn_patchLine(uint8_t L_top_corner_start);
void tjrc_rightturn_patchLine(uint8_t R_top_corner_start);
void tjrc_oblique_leftcross_patchLine(uint8_t L_top_i, uint8_t R_top_i);
void tjrc_oblique_rightcross_patchLine(uint8_t L_top_i, uint8_t R_top_i);
void u32tostr(unsigned long dat,char *str) ;
void tobin(uint32_t a,char* str);
//uint8_t* tjrc_imageProc(const uint8_t* image_p);

/* 二值化函数组 */
uint8_t tjrc_binarization_otsu(const uint8_t* image, uint16_t col, uint16_t row);
uint8_t tjrc_binarization_avg(const uint8_t* image, uint16_t col, uint16_t row);
void tjrc_binarization_getBinImage(uint8_t threshold, const uint8_t* image_in, uint8_t* image_out, uint16_t width, uint16_t height);
void tjrc_sobel_autoThreshold(const uint8_t* imageIn, uint8_t* imageOut, uint16_t width, uint16_t height);

/* 搜线，拐点，补线函数组 */
void tjrc_imageProc_searchEdge_x(const uint8_t* image, line_info* line_info_out);
void tjrc_imageProc_fineInflection(const line_info* line_info_in, inflection_info* inflection_info_out);
void tjrc_imageProc_patchLine(line_info* line_info_out, inflection_info* inflection_info_in);
void tjrc_imageProc_updateImage(uint8_t* image, line_info* line_info_in);
#endif // !__TJRC_IMAGEPROC_H__
