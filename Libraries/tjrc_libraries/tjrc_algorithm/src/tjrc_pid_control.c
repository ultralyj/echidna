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
extern uint8_t steer_direct;

/* 速度环使能标志变量 */
const uint8_t speedLoop_enable = 1;
/* 后轮驱动使能标志变量 */
const uint8_t drive_enable = 1;
/* 转向控制使能标志变量 */
const uint8_t turn_enable = 1;

float Angle_zero = -0.074f;  //平衡角度

/*******************直立环***********************************/

/**
 * @brief 直立环PID参数
 * 
 */
float angle_kp = 250000;
float angle_kd = 15000;
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
double V_S_Kp = -0.000016;
double V_S_Ki = 0.010;
double V_S_Kd = 0.15;
const double V_S_Integral_Max = 0.01;

/**
 * @brief 串联的速度环，增量式PID控制，平滑到10个平滑周期
 * 
 * @param _V_S_Bias 动量轮编码器的速度信息
 * @return float PID控制量输出
 */
float tjrc_pid_speedLoop(float _V_S_Bias)
{
    static double V_S_Bias = 0, V_S_Last = 0; // 1400,1200,m:12000
    static double Angle_New = 0, Angle_Diff = 0;
    
    /* 存储微分部分与积分部分的中间变量 */
    double V_S_Diff = 0, V_S_Integral = 0;
    double Angle_Diff_Last = 0, Angle_Old = 0;

    /* 获取编码器数值 */
    V_S_Last = (double)V_S_Bias;
    V_S_Bias = _V_S_Bias;

    /* 中间变量数值更新 */
    Angle_Diff_Last = Angle_Diff;
    Angle_Old = Angle_New;

    /* 返回串级角度 */
    Angle_New = (double)(V_S_Bias * V_S_Kp);

    /* 目标角度变化值 */
    Angle_Diff = Angle_New - Angle_Old; 
    
    /* D:微分 */
    V_S_Diff = (Angle_Diff - Angle_Diff_Last) * V_S_Kd;
    /* I:积分 */
    if (f_Abs(V_S_Bias) > 0)
        V_S_Integral = ((double)Angle_New) * V_S_Ki;

    /* 积分限幅 */
    V_S_Integral = float_Constrain(V_S_Integral, -V_S_Integral_Max, V_S_Integral_Max);

    /* 结果加和并且进行平滑处理 */
    float Angle_delta = (double)(Angle_Diff + V_S_Diff + V_S_Integral) / 20.0;
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
float Dr_kp = 3;
float Dr_ki = 0.2;
float Dr_kd = 0.35;
const float Max_Dr_Integral = 4000;

/**
 * @brief 直接法PID控制后轮驱动速度环
 * 
 * @param speed 后轮驱动电机编码器的速度信息
 * @return int32_t PWM输出（-50~50）
 */
int32_t tjrc_pid_drive(float speed)
{
    static float Dr_Bias = 0,last_Dr_Bias = 0;
    static int16_t duty = 0, last_duty = 0;
    static float Dr_Integral = 0;
    /* 获取速度差值 */
    Dr_Bias = target_speed - speed;
    /* 计算积分并限幅 */
    Dr_Integral += Dr_Bias;
    Dr_Integral = float_Constrain(Dr_Integral, -Max_Dr_Integral, Max_Dr_Integral);
    /* 直接法计算PID */
    duty = (int32_t)(Dr_kp * Dr_Bias + Dr_kd * (Dr_Bias - last_Dr_Bias) + Dr_ki * Dr_Integral);
    /* 更新数据 */
    last_duty = duty;
    last_Dr_Bias = Dr_Bias;
    /* 输出限幅 */
    duty = int16_t_Constrain(duty,-5000,5000);
    /* 输出占空比 */
    return duty;
}


/***********************************************************
 ** 以下是方向控制
 ** 方向控制每次输出一个横滚角度偏量(Turn_delta)叠加到Target_Angle上
 **
 **********************************************************/

float direct_target = 0;

float direct_kp = 200;
float direct_ki = 5;
float direct_kd = 8;
float Direct_Integral_Max = 1;


/**
 * @brief 舵机协同平衡pid
 * 
 * @param angle_kalman 卡尔曼滤波后的姿态角
 * @param angle_dot 姿态角微分
 * @return float 舵机协同平衡的偏置角
 */
float tjrc_pid_servo_balance(float angle_kalman, float angle_dot)
{
    static float Turn_delta = 0;
    static float direct_bias_pid = 0;
    static float last_Turn_delta = 0;
    static float direct_integral = 0;

    /* 获取角度偏移量（IMU测得角度与目标角度之差） */
    float angle_bias = angle_kalman - Angle_zero;
    /* 若角度偏移过大，动量轮难以调节，则使用舵机调节 */
    if(f_Abs(angle_bias) > 0.02f)
    {
        /* 计算积分量并限幅 */
        direct_integral += angle_bias;
        direct_integral = float_Constrain(direct_integral, -Direct_Integral_Max, Direct_Integral_Max);
        if(f_Abs(angle_bias) < 0.02f)
            direct_integral = direct_integral *0.5f;
        /* 直接法PID */
        direct_bias_pid = ((float)(direct_kp * angle_bias + direct_kd * (angle_dot) + direct_ki * direct_integral));
    }
    else
    {
        direct_bias_pid = 0.0f;
    }

    Turn_delta -= last_Turn_delta; //减去上次的设定
    last_Turn_delta = Turn_delta;
    Turn_delta /= 2; //由于平衡环频率是方向环两倍，分两次叠加，降低突变影响？
    return direct_bias_pid;
}

float servo_kp = 2.0f;
float servo_ki = 0.2f;
float servo_kd = 1.0f;
float servo_Integral_Max = 5.0f;
/**
 * @brief 舵机主方向控制pid
 * 
 * @param mid 中线偏移量
 * @param slope 中线斜率
 * @return float 舵机角度
 */
float tjrc_pid_servo_trace(float mid, float slope)
{
    float mid_error;
    static float servo_integral = 0.0f;
    static float servo_last = 0.0f;
    static float mid_error_last = 0;
    float servo_pid = 0.0f;

    mid_error = mid;
    /* 计算积分量并限幅 */
    servo_integral += mid_error;
    servo_integral = float_Constrain(servo_integral, -servo_Integral_Max, servo_Integral_Max);

    /* 直接法PID */
    servo_pid = (float)(servo_kp * mid_error + servo_ki*servo_integral - servo_kd*(mid_error - mid_error_last));
    servo_pid = 0.5f*servo_pid+0.5f*servo_last;
    mid_error_last = mid_error;
    servo_last = servo_pid;
    return servo_pid;
}

float enc0_fade_coe = 0.5f;
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


float enc1_fade_coe = 0.4f;
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
    if(speed_enc1>60000)
        return 0;
    speed_enc1 = ((1 - enc1_fade_coe) * speed_enc1_last + enc1_fade_coe * speed_enc1);
    speed_enc1_last = speed_enc1;
    return speed_enc1;
}


void tjrc_motionControl(void)
{
    extern float Angle_zero;
    static int32_t motor_enable;
    /* 速度环分频变量：1/20，后轮驱动分频：1/5，转向分频：1/2 */
    static uint8_t speedLoop_cnt = 0, drive_cnt = 0, turn_cnt = 0;
    /* 卡尔曼滤波初始化计数器 */
    static int32_t kalman_initCnt = 0;
    /* 函数中间参数传递变量 */
    static float b_angle_delta = 0.0f;
    static float b_angle_kalman = 0.0f , b_angle_dot = 0.0f;
    static float b_turn_delta = 0.0f, b_turn_target = 0.0f;
    static float b_turn_dot = 0.0f, b_turn_raw=0.0f;
    static float b_flyWheel_speed = 0.0f, b_drive_speed = 0.0f;

    /* PWM输出变量 */
    int flywheel_pwmOut = 0, drive_pwmOut = 0;
    /* 特别注意：等待IMU角度稳定后(约200*8ms=1.6s)再开启电机 */
    if (kalman_initCnt > 200 && motor_enable == 0)
    {
        motor_enable = 1;
        IfxPort_setPinHigh(FLYWHEEL_MOTOR_EN_PIN);
        /* sw3: 固定速度调试 */
        extern uint8_t switch_value;
        if(switch_value & SWITCH3)
        {
            printf("fix speed mode!\r\n");
            target_speed=150;
        }
        kalman_initCnt = 0;
    }
    else
    {
        kalman_initCnt++;
    }
    /* 获取IMU信息，进行卡尔曼滤波，再输入到直立环PID得到动量轮PWM输出 */
    tjrc_getAngle_kalman(&b_angle_kalman, &b_angle_dot); //得到计算的初始角度
    flywheel_pwmOut = tjrc_pid_balance(b_angle_kalman, b_angle_dot);

    /* 在电机使能的状态下，平行处理动量轮速度环，后轮驱动和舵机 */
    if (motor_enable==1)
    {
        /* 动量轮速度环（1/20） */
        if (speedLoop_enable)
        {
            if (speedLoop_cnt == 10)
            {
                speedLoop_cnt = 0;
                b_flyWheel_speed = tjrc_flyWheel_getSpeed();
                b_angle_delta = tjrc_pid_speedLoop(b_flyWheel_speed);
                uint8_t str[30];
                sprintf((char*)str,"%.04f,%.04f\n",b_angle_kalman,Angle_zero);
                tjrc_asclin1_sendStr(str);

                //sprintf((char*)str,"A0:%.03f AK:%.03f ",Angle_zero,b_angle_kalman);
                //tjrc_st7735_dispStr612(0,116,(uint8_t*)str,RGB565_BLUE);
            }
            speedLoop_cnt++;
        }

        /* 舵机方向（1/5） */
        if (turn_enable)
        {
            if (turn_cnt == 5)
            {
                turn_cnt = 0;
                /* 更新舵机辅助平衡偏置角 */
                b_turn_delta = tjrc_pid_servo_balance(b_angle_kalman, b_angle_dot);
                /* 更新舵机目标偏置角（摄像头信息） */
                extern float camera_slope;
                extern float camera_pixelError;
                b_turn_raw = -tjrc_pid_servo_trace(camera_pixelError,camera_slope);
                b_turn_dot = float_Constrain(b_turn_raw-b_turn_target, -5,5);
                b_turn_target +=b_turn_dot;
                //printf("[servo]%f,%d\r\n",b_drive_speed,drive_pwmOut);
                /* 远程目标角度调节 */
                if (steer_direct == 0 || steer_direct == 1)
                {
                    if (steer_direct == 0)
                    {
                        direct_target += 5;
                    }
                    else
                    {
                        direct_target -= 5;
                    }
                    steer_direct = -1;
                    // Turn_delta = direct*kkp*drive_encoder;//计算得到此次的压弯角度
                }

                float direct = direct_target + b_turn_target + b_turn_delta;
                direct = float_Constrain(direct, -26, 26);
                tjrc_servo_setAngle(direct);
            }
            turn_cnt++;
        }
        /* 后轮驱动（1/5） */
        if (drive_enable)
        {
            if (drive_cnt == 1)
            {
                drive_cnt = 0;
                b_drive_speed = tjrc_drive_getSpeed();
                extern float target_speed;
                drive_pwmOut = tjrc_pid_drive(b_drive_speed);
                tjrc_driveMotor_pwm(drive_pwmOut);
            }
            drive_cnt++;
        }

        /* 偏移角过大，停转保护 */
        if (b_angle_kalman > 0.35 || b_angle_kalman < -0.6 || fabs(b_flyWheel_speed) > 20000)
        {
            flywheel_pwmOut = 0;
            motor_enable = -1;
            IfxPort_setPinLow(FLYWHEEL_MOTOR_EN_PIN);
        }
        extern float Angle_zero;
        Angle_zero += b_angle_delta;                         //叠加串联输出
        flywheel_pwmOut = s32_AmpConstrain(-9000, 9000, flywheel_pwmOut); //平衡环限幅
        tjrc_flyWheelMotor_pwm(flywheel_pwmOut);
    }

}
