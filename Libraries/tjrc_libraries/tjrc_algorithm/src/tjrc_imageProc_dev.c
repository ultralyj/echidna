/*
 * tjrc_imageProc_dev.c
 *
 *  Created on: 2022年3月30日
 *      Author: 11657
 */
#include "tjrc_imageProc.h"


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


///*--------------------------------------------------------------------------
//* 【函数功能】：传递gui参数
//* 【参    数】：无
//* 【返 回 值】：无
//*--------------------------------------------------------------------------*/
//void git_gui_value(uint8_t value_in[30])
//{
//    for (int32_t i = 0; i < 30; i++)
//    {
//        value3[i] = value_in[i];
//    }
//}

/* 搜索拐点方法二 */
//another solution
            //if (enable_L_corner /*&& !L_start_lost*/) //如果使能搜索左拐点,且开始未丢线
            //{
            //  if (L_edge_count > 2 * dist + 1)
            //  {
            //      //printf("\nL_ed:%d ", L_top_corner_start - 9);
            //      //for (int32_t i = 0; i < L_top_corner_start - (2 * dist + 1); i++)//L_edge_count - 9
            //      //{
            //      //  //printf("L_ed2:%d ", L_edge[i].row);
            //      //  if (L_edge[i + dist].row > dist+1)
            //      //  {
            //      //      int32_t ang = (L_edge[i].col - L_edge[i + dist].col) * (L_edge[i + 2 * dist].col - L_edge[i + dist].col) +
            //      //          (L_edge[i].row - L_edge[i + dist].row) * (L_edge[i + 2 * dist].row - L_edge[i + dist].row);
            //      //      //printf("ang3:%d ", ang);
            //      //      if ( ang<= 0) //初步确认为锐角或者直角 向量法
            //      //      {
            //      //          //printf("L_ed3:%d ", L_edge[i].row);
            //      //          L_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col); //求角度
            //      //          L_corner_angle = 180 - L_corner_angle;
            //      //          if (L_edge[i + dist].col > L_edge[i + 2 * dist].col)    //确定拐角朝向，左拐点没有朝向做的
            //      //          {
            //      //              L_corner_flag = 1;//异常拐点
            //      //              L_corner_row = L_edge[i + dist].row;
            //      //              L_corner_col = L_edge[i + dist].col;
            //      //              //printf("左拐点：(x=%d,y=%d)，角度=%d\n", L_corner_row, L_corner_col, L_corner_angle);
            //      //              break;
            //      //          }
            //      //      }
            //      //  }
            //      //}
            //      int32_t max = 0;
            //      for (int32_t i = 0; i < L_top_corner_start - (2 * dist + 1); i++)//L_edge_count - 9
            //      {
            //          //printf("L_ed2:%d ", L_edge[i].row);
            //          if (L_edge[i + dist].row > dist + 1)
            //          {
            //              L_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col); //求角度
            //              L_corner_angle = 180 - L_corner_angle;
            //              //printf("L_ang:%d ", L_corner_angle);
            //              if (L_corner_angle > max)
            //              {
            //                  max = L_corner_angle;
            //                  L_corner_row = L_edge[i + dist].row;
            //                  L_corner_col = L_edge[i + dist].col;
            //                  L_beh_i = i + dist;
            //              }
            //          }
            //      }
            //      L_corner_angle = max;
            //  }
            //}
            //if(L_start_lost)
            //{
            //  int32_t min = 999;
            //  for (int32_t i = 1; i < L_edge_count - (2 * dist + 1); i++)
            //  {
            //      //printf("ang:%d \n", Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col));
            //      if (L_edge[i + dist].row > dist + 1)
            //      {
            //          L_center_lost_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col);
            //          if (L_center_lost_corner_angle < min)
            //          {
            //              min = L_center_lost_corner_angle;
            //              center_lost_corner_row_l = L_edge[i + dist].row;
            //              center_lost_corner_col_l = L_edge[i + dist].col;
            //              L_top_i = i + dist;
            //          }
            //          //int32_t ang = (L_edge[i].col - L_edge[i + dist].col) * (L_edge[i + 2 * dist].col - L_edge[i + dist].col) +
            //          //  (L_edge[i].row - L_edge[i + dist].row) * (L_edge[i + 2 * dist].row - L_edge[i + dist].row);
            //          //if (ang <= 0) //初步确认为锐角或者直角 向量法
            //          //{
            //          //  L_center_lost_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col); //求角度
            //          //  if (L_edge[i + dist].col < L_edge[i + 2 * dist].col && L_center_lost_corner_angle != 180)    //确定拐角朝向，左拐点没有朝向做的
            //          //  {
            //          //      L_corner_flag = 1;//异常拐点
            //          //      center_lost_corner_row_l = L_edge[i + dist].row;
            //          //      center_lost_corner_col_l = L_edge[i + dist].col;
            //          //      break;
            //          //  }
            //          //}
            //      }
            //  }
            //  L_center_lost_corner_angle = min;
            //}
            //if (center_lost_flag_l /*|| L_start_lost*/)
            //{
            //  int32_t min = 999;
            //  for (int32_t i = L_top_corner_start; i < L_edge_count - (2 * dist + 1); i++)
            //  {
            //      //printf("ang:%d \n", Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col));
            //      if (L_edge[i + dist].row > dist+1)
            //      {
            //          L_center_lost_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col);
            //          if (L_center_lost_corner_angle < min)
            //          {
            //              min = L_center_lost_corner_angle;
            //              center_lost_corner_row_l = L_edge[i + dist].row;
            //              center_lost_corner_col_l = L_edge[i + dist].col;
            //              L_top_i = i + dist;
            //          }
            //          //int32_t ang = (L_edge[i].col - L_edge[i + dist].col) * (L_edge[i + 2 * dist].col - L_edge[i + dist].col) +
            //          //  (L_edge[i].row - L_edge[i + dist].row) * (L_edge[i + 2 * dist].row - L_edge[i + dist].row);
            //          //if (ang <= 0 ) //初步确认为锐角或者直角 向量法
            //          //{
            //          //  L_center_lost_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col); //求角度
            //          //  if (L_edge[i + dist].col < L_edge[i + 2 * dist].col && L_center_lost_corner_angle!=180)    //确定拐角朝向，左拐点没有朝向做的
            //          //  {
            //          //      L_corner_flag = 1;//异常拐点
            //          //      center_lost_corner_row_l = L_edge[i + dist].row;
            //          //      center_lost_corner_col_l = L_edge[i + dist].col;
            //          //      break;
            //          //  }
            //          //}
            //      }
            //  }
            //  L_center_lost_corner_angle = min;
            //}
/*________________________________________*/
//another solution
            //if (enable_R_corner /*&& !R_start_lost*/)    //如果使能搜索右拐点
            //{
            //  if (R_edge_count > (2 * dist + 1))
            //  {
            //      //for (int32_t i = 0; i < R_top_corner_start - (2 * dist + 1); i++)//R_edge_count - 9
            //      //{
            //      //  if (R_edge[i + dist].row > dist+1)
            //      //  {
            //      //      int32_t ang = (R_edge[i].col - R_edge[i + dist].col) * (R_edge[i + 2 * dist].col - R_edge[i + dist].col) +
            //      //          (R_edge[i].row - R_edge[i + dist].row) * (R_edge[i + 2 * dist].row - R_edge[i + dist].row);
            //      //      //printf("ang=%d ", ang);
            //      //      if ( ang<= 0) //初步确认为锐角或者直角 向量法
            //      //      {
            //      //          R_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //求角度
            //      //          R_corner_angle = 180 - R_corner_angle;
            //      //          if (R_edge[i + 2 * dist].col > R_edge[i + dist].col)    //确定拐角朝向，左拐点没有朝向做的
            //      //          {
            //      //              R_corner_flag = 1;//异常拐点
            //      //              R_corner_row = R_edge[i + dist].row;
            //      //              R_corner_col = R_edge[i + dist].col;
            //      //              break;
            //      //          }
            //      //      }
            //      //  }
            //      //}
            //      int32_t max = 0;
            //      for (int32_t i = 0; i < R_top_corner_start - (2 * dist + 1); i++)//L_edge_count - 9
            //      {
            //          //printf("L_ed2:%d ", L_edge[i].row);
            //          if (R_edge[i + dist].row > dist + 1)
            //          {
            //              R_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //求角度
            //              R_corner_angle = 180 - R_corner_angle;
            //              //printf("R_ang:%d ", R_corner_angle);
            //              if (R_corner_angle > max)
            //              {
            //                  max = R_corner_angle;
            //                  R_corner_row = R_edge[i + dist].row;
            //                  R_corner_col = R_edge[i + dist].col;
            //                  R_beh_i = i + dist;
            //              }
            //          }
            //      }
            //      R_corner_angle = max;
            //  }
            //}
            //if (R_start_lost)
            //{
            //  int32_t min = 999;
            //  //printf("LLL%d", R_edge_count);
            //  //限制在70点以内  有bug需要调
            //  for (int32_t i = 0; i < 70 - (2 * dist + 1); i++)
            //  {
            //      if (R_edge[i + dist].row > dist + 1)
            //      {
            //          R_center_lost_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //求角度
            //          if (R_center_lost_corner_angle < min && R_center_lost_corner_angle>90)
            //          {
            //              min = R_center_lost_corner_angle;
            //              center_lost_corner_row_r = R_edge[i + dist].row;
            //              center_lost_corner_col_r = R_edge[i + dist].col;
            //              R_top_i = i + dist;
            //          }
            //          //int32_t ang = (R_edge[i].col - R_edge[i + dist].col) * (R_edge[i + 2 * dist].col - R_edge[i + dist].col) +
            //          //  (R_edge[i].row - R_edge[i + dist].row) * (R_edge[i + 2 * dist].row - R_edge[i + dist].row);
            //          ////printf("ang2=%d ", ang);
            //          //if (ang <= 0) //初步确认为锐角或者直角 向量法
            //          //{
            //          //  R_center_lost_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //求角度
            //          //  if (R_edge[i + dist].col > R_edge[i + 2 * dist].col && R_center_lost_corner_angle != 180)    //确定拐角朝向，左拐点没有朝向做的
            //          //  {
            //          //      R_corner_flag = 1;//异常拐点
            //          //      center_lost_corner_row_r = R_edge[i + dist].row;
            //          //      center_lost_corner_col_r = R_edge[i + dist].col;
            //          //      break;
            //          //  }
            //          //}
            //      }
            //  }
            //  R_center_lost_corner_angle = min;
            //}
            //if (center_lost_flag_r /*|| R_start_lost*/)
            //{
            //  int32_t min = 999;
            //  for (int32_t i = R_top_corner_start; i < R_edge_count - (2 * dist + 1); i++)
            //  {
            //      if (R_edge[i + dist].row > dist+1)
            //      {
            //          R_center_lost_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //求角度
            //          if (R_center_lost_corner_angle < min)
            //          {
            //              min = R_center_lost_corner_angle;
            //              center_lost_corner_row_r = R_edge[i + dist].row;
            //              center_lost_corner_col_r = R_edge[i + dist].col;
            //              R_top_i = i + dist;
            //          }                                                                                                                                                                     //int32_t ang = (R_edge[i].col - R_edge[i + dist].col) * (R_edge[i + 2 * dist].col - R_edge[i + dist].col) +
            //          //  (R_edge[i].row - R_edge[i + dist].row) * (R_edge[i + 2 * dist].row - R_edge[i + dist].row);
            //          ////printf("ang2=%d ", ang);
            //          //if (ang <= 0) //初步确认为锐角或者直角 向量法
            //          //{
            //          //  R_center_lost_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //求角度
            //          //  if (R_edge[i + dist].col > R_edge[i + 2 * dist].col && R_center_lost_corner_angle != 180)    //确定拐角朝向，左拐点没有朝向做的
            //          //  {
            //          //      R_corner_flag = 1;//异常拐点
            //          //      center_lost_corner_row_r = R_edge[i + dist].row;
            //          //      center_lost_corner_col_r = R_edge[i + dist].col;
            //          //      break;
            //          //  }
            //          //}
            //      }
            //  }
            //  R_center_lost_corner_angle = min;
            //}

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


//  for(int i=0;i<80;i++)
//    {
//        tjrc_st7735_drawPoint(L_edge_use[i].col,80+L_edge_use[i].row,0xF81F);
//    }
//  for(int i=0;i<80;i++)
//    {
//        tjrc_st7735_drawPoint(R_edge_use[i].col,80+R_edge_use[i].row,0xFFE0);
//    }
//  for(int i=0;i<80;i++)
//    {
//        tjrc_st7735_drawPoint((L_edge_use[i].col+R_edge_use[i].col)/2,80+(L_edge_use[i].row+R_edge_use[i].row)/2,0x001F);
//    }

    //gui传递参数
    //显示左右的搜线，需要调整为tft函数
    //for (int32_t i = 0, m = 0; i < 300; i = i + 2, m++)
    //{
    //  sidee[i] = L_edge[m].row;
    //  sidee[i + 1] = L_edge[m].col;
    //  if (sidee[i] > 79) sidee[i] = 79;
    //  if (sidee[i + 1] > 119) sidee[i + 1] = 119;
    //  //if (sidee[i] == 0) sidee[i] = 1;
    //  //if (sidee[i+1] == 0) sidee[i+1] = 1;
    //  //printf("%d ", sidee[i]);
    //}
    //for (int32_t i = 300, m = 0; i < 600; i = i + 2, m++)
    //{
    //  sidee[i] = R_edge[m].row;
    //  sidee[i + 1] = R_edge[m].col;
    //  if (sidee[i] > 79) sidee[i] = 79;
    //  if (sidee[i + 1] > 119) sidee[i + 1] = 119;
    //  //if (sidee[i] == 0) sidee[i] = 1;
    //  //if (sidee[i + 1] == 0) sidee[i + 1] = 1;
    //}

    //显示中线，需要调整为tft函数
    //for (int32_t i = 600, m = 0; i < 900; i = i + 2, m++)
    //{
    //  sidee[i] = Mid_Line[m].row;
    //  sidee[i + 1] = Mid_Line[m].col;
    //  if (sidee[i] > 79) sidee[i] = 79;
    //  if (sidee[i + 1] > 119) sidee[i + 1] = 119;
    //  //if (sidee[i] == 0) sidee[i] = 1;
    //  //if (sidee[i + 1] == 0) sidee[i + 1] = 1;
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
    //  sidee[i] = L_edge_use[m].row;
    //  sidee[i + 1] = L_edge_use[m].col;
    //  if (sidee[i] > 79) sidee[i] = 79;
    //  if (sidee[i + 1] > 119) sidee[i + 1] = 119;
    //  if (sidee[i] < 0) sidee[i] = 0;
    //  if (sidee[i + 1] < 0) sidee[i + 1] = 0;
    //  //printf("hhhh(%d,%d)\n", L_edge_use[m].row, L_edge_use[m].col);
    //}
    //for (int32_t i = 1240, m = 0; i < 1480; i = i + 2, m++)
    //{
    //  sidee[i] = R_edge_use[m].row;
    //  sidee[i + 1] = R_edge_use[m].col;
    //  if (sidee[i] > 79) sidee[i] = 79;
    //  if (sidee[i + 1] > 119) sidee[i + 1] = 119;
    //  if (sidee[i] < 0) sidee[i] = 0;
    //  if (sidee[i + 1] < 0) sidee[i + 1] = 0;
    //}

    //return sidee;

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

//extern uint8_t image[MT9V03X_H][MT9V03X_W];
void average_filter(void)
{
//    uint8_t i, j;
//    uint8_t  c[9];
//    for (i = 1; i < Image_H - 1; i++)
//    {
//        for (j = 1; j < Image_W - 1; j++)
//        {
//            if (i >= 8)
//            {
//                c[0] = image[i - 1][j - 1];
//                c[1] = image[i - 1][j];
//                c[2] = image[i - 1][j + 1];
//                c[3] = image[i][j - 1];
//                c[4] = image[i][j];
//                c[5] = image[i][j + 1];
//                c[6] = image[i + 1][j - 1];
//                c[7] = image[i + 1][j];
//                c[8] = image[i + 1][j + 1];
//                image[i][j] = average_value(c);
//            }
//
//        }
//    }

}

void get_deal_image()
{
//    for (int32_t row = 0; row < Image_H; row++)
//        for (int32_t col = 0; col < Image_W; col++)
//        {
//            image[row][col] = mt9v03x_image[row][col];
//        }
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
  //    int32_t i = 0, j = 0;
  //    int32_t Threshold = 0;
  //    unsigned long  tv = 0;
  //    char txt[16];
  //
  //    if (mode == 0)
  //    {
  //        Threshold = GetOSTU(image_in);//大津法阈值
  //        return Threshold;
  //    }
  //    if (mode == 1)
  //    {
  //        //累加
  //        for (i = 0; i < MT9V03X_H; i++)
  //        {
  //            for (j = 0; j < MT9V03X_W; j++)
  //            {
  //                tv += image_in[i][j];   //累加
  //            }
  //        }
  //        Threshold = tv / MT9V03X_H / MT9V03X_W;        //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
  //        Threshold = Threshold + 20;      //此处阈值设置，根据环境的光线来设定
  //        return Threshold;
  //    }
  //    else if (mode == 2)
  //    {
  //        Threshold = 50;
  //        //手动调节阈值
  //        SobelThreshold(image_in, (uint8_t)Threshold);
  //        return 1;
  //
  //    }
  //    else if (mode == 3)
  //    {
  //        SobelAutoThreshold(image_in);  //动态调节阈值
  //        return 1;
  //    }
  //    else if (mode == 4)
  //    {
  //        Threshold = XLW_otsuThreshold(image_in, MT9V03X_H, MT9V03X_W);  //OTSU
  //        //return Threshold ;
  //    }
  //    /* 二值化 */
  //    for (i = 0; i < MT9V03X_H; i++)
  //    {
  //        for (j = 0; j < MT9V03X_W; j++)
  //        {
  //            if (image_in[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
  //            {
  //                image_in[i][j] = 255;
  //            }
  //            else
  //            {
  //                image_in[i][j] = 1;
  //            }
  //        }
  //    }
  //    return 1;
  //}

  // TODO
/**
 * @brief OTSU大津法二值化计算阈值
 * @param image (uint8_t*)图像起始指针
 * @param col   (uint16_t)列数
 * @param row   (uint16_t)行数
 * @return (uint8_t)计算得出的阈值
 */



///*!
//  * @brief    大津法求阈值大小(龙邱)
//  * @param    tmImage ： 图像数据
//  * @return   阈值
// * @note     Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
//  * @note     1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
//  * @note     2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
//  * @note     3) i表示分类的阈值，也即一个灰度级，从0开始迭代    1
//  * @note     4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像的比例w0，并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背景像素) 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
//  * @note     5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
//  * @note     6) i++；转到4)，直到i为256时结束迭代
//  * @note     7) 将最大g相应的i值作为图像的全局阈值
//  * @note     缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
//  * @note     解决光照不均匀  https://blog.csdn.net/kk55guang2/article/details/78475414
//  * @note     https://blog.csdn.net/kk55guang2/article/details/78490069
//  * @note     https://wenku.baidu.com/view/84e5eb271a37f111f0855b2d.html
//  * @see      GetOSTU(Image_Use);//大津法阈值
//  * @date     2019/6/25 星期二
//  */
//int32_t GetOSTU(uint8_t* tmImage)
//{
//    signed short i, j;
//    unsigned long Amount = 0;
//    unsigned long PixelBack = 0;
//    unsigned long PixelIntegralBack = 0;
//    unsigned long PixelIntegral = 0;
//    signed long PixelIntegralFore = 0;
//    signed long PixelFore = 0;
//    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
//    signed short MinValue, MaxValue;
//    signed short Threshold = 0;
//    unsigned char HistoGram[256];              //
//
//    for (j = 0; j < 256; j++)  HistoGram[j] = 0; //初始化灰度直方图
//
//    for (j = 0; j < MT9V03X_H; j++)
//    {
//        for (i = 0; i < MT9V03X_W; i++)
//        {
//            HistoGram[tmImage[i*MT9V03X_W+j]]++; //统计灰度级中每个像素在整幅图像中的个数
//        }
//    }
//
//    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
//    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值
//
//    if (MaxValue == MinValue)     return MaxValue;         // 图像中只有一个颜色
//    if (MinValue + 1 == MaxValue)  return MinValue;        // 图像中只有二个颜色
//
//    for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  像素总数
//
//    PixelIntegral = 0;
//    for (j = MinValue; j <= MaxValue; j++)
//    {
//        PixelIntegral += HistoGram[j] * j;//灰度值总数
//    }
//    SigmaB = -1;
//    for (j = MinValue; j < MaxValue; j++)
//    {
//        PixelBack = PixelBack + HistoGram[j];   //前景像素点数
//        PixelFore = Amount - PixelBack;         //背景像素点数
//        OmegaBack = (float)PixelBack / Amount;//前景像素百分比
//        OmegaFore = (float)PixelFore / Amount;//背景像素百分比
//        PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
//        PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
//        MicroBack = (float)PixelIntegralBack / PixelBack;   //前景灰度百分比
//        MicroFore = (float)PixelIntegralFore / PixelFore;   //背景灰度百分比
//        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//计算类间方差
//        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
//        {
//            SigmaB = Sigma;
//            Threshold = j;
//        }
//    }
//    return Threshold;                        //返回最佳阈值;
//}


///*--------------------------------------------------------------------------
//* 【函数功能】：大津法求动态阈值
//* 【参    数】：无
//* 【返 回 值】：无
//*--------------------------------------------------------------------------*/
//#if 1
//uint8_t GetOSTUThreshold(uint8_t(*img)[MT9V03X_W], uint16_t start_row, uint16_t end_row, uint16_t start_col, uint16_t end_col)
//{
//    int32_t     threshold1 = 0;
//    int32_t sum_gray = 0;
//    int32_t sum_pix_num = 0;
//    int32_t pl_pix_num = 0;
//    int32_t p2_pix_mum = 0;
//    int32_t p1_sum_gray = 0;
//    float m1 = 0;
//    float m2 = 0;
//    float V = 0;
//    float variance = 0;
//    int32_t i, j, k = 0;
//    uint16_t MinValue = 0, MaxValue = 255;
//    uint16_t DeleGrayClass1 = 30; //高灰度级
//    uint16_t  HistoGram[256] = { 0 };
//
//    for (i = 0; i < 256; i++)
//        HistoGram[i] = 0;
//
//    for (i = start_row; i < end_row; i++)
//    {
//        for (j = start_col; j < end_col; j++)
//        {
//            HistoGram[(int32_t)img[i][j]]++;
//        }
//    }
//
//    //优化--删除灰度级顶部<x个点的灰度级  删除灰度级底部<x个点的灰度级 x==> 10-25之间
//    //for(k=255;k>0;--k) {if(HistoGram[k]<=DeleGrayClass1)  HistoGram[k] = 0; else break;}
//    //for(k=0;k<256;++k) {if(HistoGram[k]<=DeleGrayClass2)  HistoGram[k] = 0; else break;}
//
//    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; ++MinValue);        //获取最小灰度级
//    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MaxValue] == 0; --MaxValue); //获取最大灰度级
//
//    for (k = MinValue; k <= MaxValue; k++)
//    {
//        sum_gray += k * HistoGram[k];
//        sum_pix_num += HistoGram[k];
//    }
//
//    for (k = MinValue; k <= MaxValue; k++)
//    {
//        pl_pix_num += HistoGram[k];
//        p2_pix_mum = sum_pix_num - pl_pix_num;
//        p1_sum_gray += k * HistoGram[k];
//        m1 = (float)p1_sum_gray / pl_pix_num;
//        m2 = (float)(sum_gray - p1_sum_gray) / p2_pix_mum;
//
//        V = pl_pix_num * p2_pix_mum * (m1 - m2) * (m1 - m2);
//
//        if (V > variance)
//        {
//            variance = V;
//            threshold1 = k;
//        }
//    }
//
//#if 1
//    uint8_t t1 = threshold1 / 2;
//    uint8_t t2 = threshold1 + 10;  //拉伸范围
//    float rate = 1.25;          //拉伸比例！！
//    //uint8_t top_y   = 15;            //顶部行域 0-top_y
//    uint8_t side_x = 20;            //侧边列域 0-side_x && (COL_1-side_x)-COL
//
//
//    /* 初始化 */
//    for (i = 255; i > 0; i--)
//        HistoGram[i] = i;
//
//    /* 梯度变换--对比度拉伸 */
//    for (i = t1; i < t2; i++)
//    {
//        HistoGram[i] = (uint16_t)(i * rate);
//        if (HistoGram[i] > t2) HistoGram[i] = t2;
//    }
//
//    //左侧
//    for (i = 0; i < MT9V03X_H; i++)
//    {
//        for (j = 0; j < side_x; j++)
//        {
//            mt9v03x_image[i][j] = HistoGram[mt9v03x_image[i][j]];
//        }
//    }
//
//    //右侧
//    for (i = 0; i < MT9V03X_H; i++)
//    {
//        for (j = MT9V03X_W - 1 - side_x; j < MT9V03X_W; j++)
//        {
//            mt9v03x_image[i][j] = HistoGram[mt9v03x_image[i][j]];
//        }
//    }
//#endif
//    return threshold1;
//}
//#endif

