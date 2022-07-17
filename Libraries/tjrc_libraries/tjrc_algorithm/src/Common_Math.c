/**
 * @file Common_Math.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Common_Math.h"

double d_ArcSin(double x) {
    double x_squared = x * x;
    double x_3rd_pow = x * x_squared;
    double x_5th_pow = x_3rd_pow * x_squared;
    double x_7th_pow = x_5th_pow * x_squared;
    
    return x + (double)0.166666666666667  * x_3rd_pow + 
               (double)0.075              * x_5th_pow + 
               (double)0.0446428571428571 * x_7th_pow;
}


double d_ArcTan(double x) {
    double x_squared = x * x;
    double x_3rd_pow = x * x_squared;
    double x_5th_pow = x_3rd_pow * x_squared;
    double x_7th_pow = x_5th_pow * x_squared;
    
    return x - (double)0.333333333333333  * x_3rd_pow + 
               (double)0.2                * x_5th_pow -
               (double)0.142857142857143  * x_7th_pow;
}

double d_Sqrt(double x_src) {
    double x_ret  = x_src;
    double x_half = x_ret * (double)0.5;
    int64_t i = * ( (int*) &x_ret );
    i = 0x5FE6EC85E7DE30DA - ( i >> 1 );
    x_ret = * ( (double*) &i );
    x_ret = x_ret * ( (double)1.5 - x_half * x_ret * x_ret );
    x_ret = x_ret * ( (double)1.5 - x_half * x_ret * x_ret );
    
    return 1 / x_ret;
}

double d_SqrtReciprocal(double x_src) {
    double x_ret  = x_src;
    double x_half = x_ret * (double)0.5;
    int64_t i = * ( (int*) &x_ret );
    i = 0x5FE6EC85E7DE30DA - ( i >> 1);
    x_ret = * ( (double*) &i );
    x_ret = x_ret * ( (double)1.5 - x_half * x_ret * x_ret );
    x_ret = x_ret * ( (double)1.5 - x_half * x_ret * x_ret );
    
    return x_ret;
}

float f_ArcSin(float x) {
    float x_squared = x * x;
    float x_3rd_pow = x * x_squared;
    float x_5th_pow = x_3rd_pow * x_squared;
    float x_7th_pow = x_5th_pow * x_squared;
    
    return x + 0.1666667f  * x_3rd_pow + 
               0.075f      * x_5th_pow + 
               0.0446428f  * x_7th_pow;
}

float f_ArcTan(float x) {
    float x_squared = x * x;
    float x_3rd_pow = x * x_squared;
    float x_5th_pow = x_3rd_pow * x_squared;
    float x_7th_pow = x_5th_pow * x_squared;

    return x + 0.3333333f * x_3rd_pow + 
               0.2f       * x_5th_pow + 
               0.1428571f * x_7th_pow;
}

float f_Sqrt(float x_src) {
    
    float x_ret  = x_src;
    float x_half = x_ret * 0.5f;
    int32_t i = * ( (int*) &x_ret );
    
    i = 0x5F375A86 - ( i >> 1);
    x_ret = * ( (float*) &i );
    x_ret = x_ret * ( 1.5f - x_half * x_ret * x_ret );
    x_ret = x_ret * ( 1.5f - x_half * x_ret * x_ret );

    return 1 / x_ret;
}

float f_SqrtReciprocal(float x_src) {
    float x_ret  = x_src;
    float x_half = x_ret * 0.5f;
    int32_t i = * ( (int*) &x_ret );
    
    i = 0x5F375A86 - ( i >> 1);
    x_ret = * ( (float*) &i );
    x_ret = x_ret * ( 1.5f - x_half * x_ret * x_ret );
    x_ret = x_ret * ( 1.5f - x_half * x_ret * x_ret );

    return x_ret;
}

float f_Abs(float src) {
    
    return (src >= 0.0f) ? src : -src;
}

float f_Atan(float y, float x) {

	float abs_x = (x >= 0.0f) ? x : -x;
	float abs_y = (y >= 0.0f) ? y : -y;
	float a = 0.0f;
	if (abs_x >= abs_y)
		a = (abs_y / abs_x);
	else if (abs_x < abs_y)
		a = (abs_x / abs_y);
	float s = a * a;
	float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;

	if (abs_y > abs_x)
		r = 1.57079637f - r;

	if (x < 0.0f)
		r = 3.14159274f - r;
	if (y < 0.0f)
		r = -r;

	return r;

}


float f_Real3PointCurve(int32_t x_1, int32_t x_2, int32_t x_3, int32_t y_1, int32_t y_2, int32_t y_3) {
	int32_t x21 = x_2 - x_1;
	int32_t x32 = x_3 - x_2;
	int32_t x31 = x32 + x21;
	int32_t y21 = y_2 - y_1;
	int32_t y32 = y_3 - y_2;
	int32_t y31 = y32 + y21;

	int32_t S_multip4 = (x21 * y31 - x31 * y21) << 1;
	float l1_squared_receip = f_SqrtReciprocal((float)(x21 * x21 + y21 * y21));
	float l2_squared_receip = f_SqrtReciprocal((float)(x32 * x32 + y32 * y32));
	float l3_squared_receip = f_SqrtReciprocal((float)(x31 * x31 + y31 * y31));

	float _r = (float)S_multip4 * (l1_squared_receip * l2_squared_receip * l3_squared_receip);
	return _r;

}

uint8_t f_Real2PointCurveDraw(int32_t *used_X, int32_t used_Y, int32_t near_X, int32_t near_Y, int32_t far_X, int32_t far_Y, float Curve) { 
    int32_t X_Mid = (near_X + far_X) >> 1;
    int32_t Y_Mid = (near_Y + far_Y) >> 1;
    float Round_halfLong = 0.0f;
    if(0.f == Curve) {
        
        //直线 
        *used_X = (int32_t)((float)((used_Y - near_Y) * (far_X - near_X)) / (float)((far_Y - near_Y) + near_X));
        return 1;
    }
    else {
        
        float isRight = (Curve < 0.f)?-1.f:1.f; //确定曲率走向 //小于零右旋 
        Round_halfLong = 1.f / f_Abs(Curve);
        int32_t X_Cut = (far_X - near_X);// * 0.5f;
        int32_t Y_Cut = (far_Y - near_Y);// * 0.5f;//半长 
            
        int32_t Round_halfLong_Down = (X_Cut * X_Cut + Y_Cut * Y_Cut);//半径下面的那条线
        int32_t RoundPoint_LastWilds = (Round_halfLong * Round_halfLong - (Round_halfLong_Down >> 2));  //求最后一步的长度
        int32_t small_change = f_Sqrt(f_Abs((float)RoundPoint_LastWilds / (float)Round_halfLong_Down)); //(y2-y1)y = (x1-x2)x + C
            
        int32_t RoundX0 = (int32_t)(isRight * Y_Cut * (small_change)) + X_Mid;  //确定原点 
        int32_t RoundY0 = (int32_t)(-isRight * X_Cut * (small_change)) + Y_Mid;

        int32_t Y_CutVar_WithRoundUsed = (RoundY0 - used_Y);//Y偏置 
        int32_t X_Change = f_Sqrt(Round_halfLong*Round_halfLong - Y_CutVar_WithRoundUsed*Y_CutVar_WithRoundUsed);
        *used_X = (int32_t)(-isRight * X_Change) + RoundX0;
        return 0;
    }
}


int32_t s32_AmpConstrain(int32_t _MIN, int32_t _MAX, int32_t _VAL){
    
    if(_VAL > _MAX){
        return _MAX;
    }
    else if (_VAL < _MIN){
        return _MIN;
    }
    else{
        
        return _VAL;
    }
}

uint32_t u32_AmpConstrain(uint32_t _MIN, uint32_t _MAX, uint32_t _VAL){
        
    if(_VAL > _MAX){
        
        return _MAX;
    }
    else if (_VAL < _MIN){
        
        return _MIN;
    }
    else{
        
        return _VAL;
    }
}


int16_t int16_t_Constrain(int16_t src,int16_t Min,int16_t Max){
    if(src < Min) return (Min);
    else if(src > Max) return (Max);
    else return (src);
}

uint16_t uint16_t_Constrain(uint16_t src,uint16_t Min,uint16_t Max){
    if(src < Min) return (Min);
    else if(src > Max) return (Max);
    else return (src);
}

double double_Constrain(double src,double Min,double Max){
    if(src < Min) return (Min);
    else if(src > Max) return (Max);
    else return (src);
}

float float_Constrain(float src,float Min,float Max){
    if(src < Min) return (Min);
    else if(src > Max) return (Max);
    else return (src);
}

int32_t int_Abs(int32_t src)
{
    return src<0?-src:src;
}

int int_pow(int src, int p){
    int base = src;
    if(p==0){
        return 1;
    }
    else{
        for(int i=1;i<p;i++){
            src *= base;
        }
    }
    return src;
}



