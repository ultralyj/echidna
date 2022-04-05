/**
 * @file Peripheral.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 无线串口接收函数的c文件
 * @version 0.1
 * @date 2022-03-13
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "tjrc_wireless.h"

uint8_t steer_direct = 2, run_direct = 0;

extern rt_sem_t Run_sem;

/**
 * @brief 串口参数调试回调函数（数据格式："(<coe><num_int>)"，如"(p100)"）
 * 
 */
void tjrc_wireless_recCallBack(void)
{
	char buffer[20];
	static uint8_t rx_cnt = 0;
	static uint8_t state = 0;
	float data = 0;
	tjrc_asclin1_receive((uint8_t *)&buffer[rx_cnt]);
	if (buffer[rx_cnt] == '(')
	{ 
		state = 1;
		rx_cnt++;
	}
	else if (state == 1 && ((buffer[rx_cnt] >= 'a' && buffer[rx_cnt] <= 'z') || (buffer[rx_cnt] >= 'A' && buffer[rx_cnt] <= 'Z')))
	{
		rx_cnt++;
		state = 2;
	}
	else if (state == 2 && buffer[rx_cnt] >= '0' && buffer[rx_cnt] <= '9' && buffer[rx_cnt] != ')')
	{
		rx_cnt++;
	}
	else if (buffer[rx_cnt] == ')')
	{
		for (uint8_t i = 0; i + 2 < rx_cnt; i++)
		{
			data += (buffer[2 + i] - 48) * int_pow(10, rx_cnt - 3 - i);
		}
		switch (buffer[1])
		{
		case 'p':
		    Dr_kp = data/100;
			printf("angle_kp:%f", angle_kp);
			break;
		case 'd':
		    Dr_kd = data/100;
			printf("angle_kd=%f", angle_kd);
			break;
		case 'i':
		    Dr_ki = data/100;
			printf("angle_kd=%f", angle_ki);
			break;
		case 'P':
			V_S_Kp = -data / 10000000; // 10
			printf("speed_kp=%f", V_S_Kp);
			break;
		case 'I':
			V_S_Ki = data / 1000; // 1
			printf("speed_i=%f", V_S_Ki);
			break;
		case 'D':
			V_S_Kd = data / 10; // 1
			printf("speed_d=%f", V_S_Kd);
			break;
		case 'R':
			rt_sem_release(Run_sem);
			run_direct = 1;
			break;
		case 'r':
			rt_sem_release(Run_sem);
			run_direct = 0;
			break;
		case 'J':
			// rt_sem_release(Direct_sem);
			steer_direct = 0;
			break;
		case 'j':
			// rt_sem_release(Direct_sem);
			steer_direct = 1;
			break;
		}
		rx_cnt = 0;
		state = 0;
	}
}
