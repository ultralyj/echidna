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
extern float target_speed;
extern float direct_target;
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
		    angle_kp = data*100;
			printf("angle_kp:%f", angle_kp);
			break;
		case 'd':
		    angle_kd = data*100;
			printf("angle_kd=%f", angle_kd);
			break;
		case 'i':
		    angle_ki = data*100;
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
		case 'O':
		    target_speed = data;
            run_direct = 1;
            break;
        case 'o':
            target_speed = -data;
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
		case 'C':
            // rt_sem_release(Direct_sem);
		    direct_target = data;
            break;
        case 'c':
            // rt_sem_release(Direct_sem);
            steer_direct = -data;
            break;
		case 'X':
		    target_speed = 0;
		    break;
		case 'Y':
		    direct_target = 0;
		    break;
		case 'B':
		    IfxPort_togglePin(BEEP_PIN);
		    break;
		}
		rx_cnt = 0;
		state = 0;
	}
}

/**
 * @brief 
 * 
 * @param image 
 * @param threshold 
 */
void tjrc_wireless_sendImage(uint8_t* image, uint8_t threshold)
{
    uint8_t imggg[120];
    uint8_t imggg2[120];
    uint8_t imggg3[120];
    int32_t mmm=0;

   for(int j=0;j<120;j++)
   {
       if(image[mmm*120+j]>threshold) imggg[j] = 0;
       else imggg[j] = 1;
       for (mmm = 1; mmm < 31; mmm++)
       {
           if(image[mmm*120+j]>threshold) imggg[j] = (imggg[j] << 1) + 0;
           else imggg[j] = (imggg[j] << 1) + 1;
       }
       if(mmm==31)mmm=0;
       printf("%x ",imggg[j]);
   }
   mmm=31;
   for(int j=0;j<120;j++)
   {
       if(image[mmm*120+j]>threshold) imggg2[j] = 0;
       else imggg2[j] = 1;
       for (mmm = 32; mmm < 62; mmm++)
       {
           if(image[mmm*120+j]>threshold) imggg2[j] = (imggg2[j] << 1) + 0;
           else imggg2[j] = (imggg2[j] << 1) + 1;
       }
       if(mmm==62)mmm=31;
       printf("%x ",imggg2[j]);
   }
    mmm=62;
   for(int j=0;j<120;j++)
   {
       if(image[mmm*120+j]>threshold) imggg3[j] = 0;
       else imggg3[j] = 1;
       for (mmm = 63; mmm < 80; mmm++)
       {
           if(image[mmm*120+j]>threshold) imggg3[j] = (imggg3[j] << 1) + 0;
           else imggg3[j] = (imggg3[j] << 1) + 1;
       }
       if(mmm==80)mmm=62;
       printf("%x ",imggg3[j]);
   }
   printf("\r\n");
}
