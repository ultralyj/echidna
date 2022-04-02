/**
 * @file tjrc_motor.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_motor.h"

void tjrc_setFlyWheelMotor(void)
{
    IfxPort_setPinModeOutput(FLYWHEEL_MOTOR_DIR_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(FLYWHEEL_MOTOR_EN_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(DRIVE_MOTOR_DIR_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    /* 初始化动量轮PWM(GTM ATOM) */
    tjrc_setGtmAtom_pwm(&FLYWHEEL_MOTOR_GTM_ATOM_CHANNEL, 16000);
    /* 初始化动量后轮PWM(GTM ATOM) */
    tjrc_setGtmAtom_pwm(&DRIVE_MOTOR_GTM_ATOM_CHANNEL, 16000);
    /* 使能电机 */
    IfxPort_setPinHigh(FLYWHEEL_MOTOR_EN_PIN);
}

void tjrc_flyWheelMotor_pwm(int32_t duty)
{
    if(duty >= 0)
    {
        IfxPort_setPinLow(FLYWHEEL_MOTOR_DIR_PIN);
        tjrc_gtmAtom_setDutyCycle(&FLYWHEEL_MOTOR_GTM_ATOM_CHANNEL, duty);
    }
    else
    {
        IfxPort_setPinHigh(FLYWHEEL_MOTOR_DIR_PIN);
        tjrc_gtmAtom_setDutyCycle(&FLYWHEEL_MOTOR_GTM_ATOM_CHANNEL, -duty);
    }
}


void tjrc_driveMotor_pwm(int32_t duty)
{
    duty = -duty;
    if(duty >= 0)
    {
        IfxPort_setPinLow(DRIVE_MOTOR_DIR_PIN);
        tjrc_gtmAtom_setDutyCycle(&DRIVE_MOTOR_GTM_ATOM_CHANNEL, duty);
    }
    else
    {
        IfxPort_setPinHigh(DRIVE_MOTOR_DIR_PIN);
        tjrc_gtmAtom_setDutyCycle(&DRIVE_MOTOR_GTM_ATOM_CHANNEL, -duty);
    }
}

