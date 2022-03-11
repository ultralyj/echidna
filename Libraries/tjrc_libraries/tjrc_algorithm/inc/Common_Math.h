#ifndef __HYPERMATH_H
#define __HYPERMATH_H    

#include "stdint.h"
#include "../../tjrc_hardware_conf/tjrc_hardware.h"
#include "../../tjrc_perpherals/tjrc_peripherals.h"

double d_ArcSin(double x);
double d_ArcTan(double x);
double d_Sqrt(double x_src);
double d_SqrtReciprocal(double x_src);

float f_ArcSin(float x);
float f_ArcTan(float x);
float f_Sqrt(float x_src);
float f_SqrtReciprocal(float x_src);

float f_Abs(float src);
float f_Atan (float y, float x);

float f_Real3PointCurve(int32_t x_1, int32_t x_2, int32_t x_3, int32_t y_1, int32_t y_2, int32_t y_3);

int32_t s32_AmpConstrain(int32_t _MIN, int32_t _MAX, int32_t _VAL);
uint32_t u32_AmpConstrain(uint32_t _MIN, uint32_t _MAX, uint32_t _VAL);

//#define abs(x) ((x)>0? (x):-(x))
#define min(a,b) ((a)<(b)? (a):(b))
#define max(a,b) ((a)>(b)? (a):(b))

/*浮点型参数限幅函数*/
float float_Constrain(float amt,float low,float high);

/*16位有符号整型参数限幅函数*/
int16_t int16_t_Constrain(int16_t amt,int16_t low,int16_t high);

/*16位无符号整型参数限幅函数*/
uint16_t uint16_t_Constrain(uint16_t amt,uint16_t low,uint16_t high);

/*DOUBLE型参数限幅函数*/
double double_Constrain(double amt,double low,double high);


int int_pow(int src, int p);



#endif
