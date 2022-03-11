/*
 * tjrc_motor.c
 *
 *  Created on: 2022年3月10日
 *      Author: 11657
 */

#include "tjrc_motor.h"

void tjrc_setFlyWheelMotor(void)
{
    IfxPort_setPinModeOutput(FLYWHEEL_MOTOR_DIR_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(FLYWHEEL_MOTOR_EN_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    tjrc_setGtmAtom_pwm(&FLYWHEEL_MOTOR_GTM_ATOM_CHANNEL, 16000);

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

