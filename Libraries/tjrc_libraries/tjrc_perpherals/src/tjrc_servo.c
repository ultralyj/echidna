/**
 * @file tjrc_servo.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_servo.h"

/**
 * @brief 设置舵机角度
 * @param servoAngle (float32)从-90到90度，舵机的旋转角度
 * @return NONE
 */
float32 tjrc_servo_setAngle(float servoAngle)
{
    const float SERVO_OFFSET = 0;
    static float servoAngleRe = 0.00f;
    if (servoAngle >= -90.0f && servoAngle <= 90.0f)
    {
        /* 0.5 - 2.5 250 - 1250 */
        servoAngleRe = servoAngle;
        uint32_t dutyCycle = (uint32_t)(1000.0f * (servoAngle + 90.0f) / 180.0f + 250.0f + SERVO_OFFSET);
        tjrc_gtmAtom_setDutyCycle(SERVO_GTM_ATOM_CHANNEL,dutyCycle);
    }
    return servoAngleRe;
}
