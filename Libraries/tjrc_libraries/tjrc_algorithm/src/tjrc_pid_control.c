/**
 * @file control.c
 * @author YYYDS team (Sangday)
 * @brief 包括两个编码器采集，平衡环，动量轮速度环，驱动轮速度环，舵机转向控制
 * @version 0.1
 * @date 2022-03-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <tjrc_pid_control.h>

float Angle_zero = -0.15f;  //平衡角度

/*******************直立环***********************************/

/**
 * @brief 直立环PID参数
 * 
 */
float angle_kp = 250000;
float angle_kd = 8000;
float angle_ki = 1500;
const float Angle_Integral_Max = 0.5;

/**
 * @brief 直接法PID控制直立环
 * 
 * @param angle_kalman 卡尔曼滤波计算得到的角度
 * @param angle_dot 卡尔曼滤波计算得到的角速度
 * @return int32_t PID输出控制量（PWM占空比）
 */
int32_t tjrc_pid_balance(float angle_kalman, float angle_dot)
{
    float angle_bias;
    static float Angle_Integral = 0;
    int32_t pwm_out = 0;
    
    /* 获取角度偏移量（IMU测得角度与目标角度之差） */
    angle_bias = angle_kalman - Angle_zero;

    /* 计算积分量并限幅 */
    Angle_Integral += angle_bias;
    Angle_Integral = float_Constrain(Angle_Integral, -Angle_Integral_Max, Angle_Integral_Max);
    
    /* 直接法PID */
    pwm_out = ((int)(angle_kp * angle_bias + angle_kd * (angle_dot) + angle_ki * Angle_Integral));
    return pwm_out;
}

/*******************速度环***********************************/

/**
 * @brief 速度环PID参数
 * 
 */
float V_S_Kp = -0.0000055;
float V_S_Ki = 0.025;
float V_S_Kd = 2.8;
const float V_S_Integral_Max = 0.01;

/**
 * @brief 串联的速度环，增量式PID控制，平滑到20个平滑周期
 *
 * @return float PID控制量输出
 */
float tjrc_pid_speedLoop(float _V_S_Bias)
{
    static float V_S_Bias = 0, V_S_Last = 0; // 1400,1200,m:12000
    static float Angle_New = 0, Angle_Diff = 0;
    
    /* 存储微分部分与积分部分的中间变量 */
    float V_S_Diff = 0, V_S_Integral = 0;
    float Angle_Diff_Last = 0, Angle_Old = 0;

    /* 获取编码器数值 */
    V_S_Last = V_S_Bias;
    V_S_Bias = _V_S_Bias;

    /* 中间变量数值更新 */
    Angle_Diff_Last = Angle_Diff;
    Angle_Old = Angle_New;

    /* 返回串级角度 */
    Angle_New = (float)(V_S_Bias * V_S_Kp);

    /* 目标角度变化值 */
    Angle_Diff = Angle_New - Angle_Old; 
    
    /* D:微分 */
    V_S_Diff = (Angle_Diff - Angle_Diff_Last) * V_S_Kd;
    /* I:积分 */
    if (f_Abs(V_S_Bias) > 0)
        V_S_Integral = ((float)Angle_New) * V_S_Ki;

    /* 积分限幅 */
    V_S_Integral = float_Constrain(V_S_Integral, -V_S_Integral_Max, V_S_Integral_Max);
    //    V_S_Integral=0;  //不使用积分
    //    V_S_Diff = 0;//不使用微分

    /* 结果加和并且进行平滑处理 */
    float Angle_delta = (Angle_Diff + V_S_Diff + V_S_Integral) / 20;
    return Angle_delta;
}


/*******************驱动轮速度控制****************************/

/**
 * @brief 目标速度
 * 
 */
float target_speed = 0;

/**
 * @brief 后轮驱动PID参数
 * 
 */
float Dr_kp = 0.02;
float Dr_ki = 0.003;
float Dr_kd = 0.01;
const float Max_Dr_Integral = 2000;

/**
 * @brief 直接法PID控制后轮驱动速度环
 * 
 * @return int32_t PWM输出（-50~50）
 */
int32_t tjrc_pid_drive(float speed)
{
    static float Dr_Bias = 0,last_Dr_Bias = 0;
    static int duty = 0, last_duty = 0;
    static float Dr_Integral = 0;
    /* 获取速度差值 */
    Dr_Bias = target_speed - speed;
    /* 计算积分并限幅 */
    Dr_Integral += Dr_Bias;
    Dr_Integral = float_Constrain(Dr_Integral, -Max_Dr_Integral, Max_Dr_Integral);
    /* 直接法计算PID */
    duty = (int)(Dr_kp * Dr_Bias + Dr_kd * (Dr_Bias - last_Dr_Bias) + Dr_ki * Dr_Integral);
    /* 更新数据 */
    last_duty = duty;
    last_Dr_Bias = Dr_Bias;
    /* 输出占空比 */
    return duty;
}


/***********************************************************
 ** 以下是方向控制
 ** 方向控制每次输出一个横滚角度偏量(Turn_delta)叠加到Target_Angle上
 **
 **********************************************************/

float kkp = -0.00001;
/**
 * @brief 
 * 
 * @return float 舵机控制量输出
 */
float Turn_out(void)
{
    static float Turn_delta = 0;
    static float direct = 0;
    static float last_Turn_delta = 0;
    if (steer_direct == 0 || steer_direct == 1)
    {
        if (steer_direct == 0)
        {
            direct += 5;
        }
        else
        {
            direct -= 5;
        }
        direct = float_Constrain(direct, -18, 18);
        tjrc_servo_setAngle(direct);
        steer_direct = -1;
        Turn_delta = 0;
        // Turn_delta = direct*kkp*drive_encoder;//计算得到此次的压弯角度
    }
    Turn_delta -= last_Turn_delta; //减去上次的设定
    last_Turn_delta = Turn_delta;
    Turn_delta /= 2; //由于平衡环频率是方向环两倍，分两次叠加，降低突变影响？
    return Turn_delta;
}

float enc0_fade_coe = 0.4f;
/**
 * @brief 通过编码器0(GPT12-T2)，获取动量轮的速度信息
 * 
 * @return float 动量轮转速
 */
float tjrc_flyWheel_getSpeed(void)
{
    static float speed_enc0_last = 0;
    float speed_enc0;
    speed_enc0 = (float)tjrc_gpt12_getT2();
    speed_enc0 = ((1.0f-enc0_fade_coe) * speed_enc0_last + enc0_fade_coe * speed_enc0);
    speed_enc0_last = speed_enc0;
    return speed_enc0;
}

/**
 * @brief 通过编码器1(GPT12-T6)，获取驱动轮的速度信息
 * 
 * @return float 驱动轮转速
 */
float tjrc_drive_getSpeed(void)
{
    static float speed_enc1_last = 0;
    float speed_enc1;
    speed_enc1 = (float)tjrc_gpt12_getT6();
    speed_enc1 = (0.8 * speed_enc1_last + 0.2 * speed_enc1);
    speed_enc1_last = speed_enc1;
    return speed_enc1;
}
