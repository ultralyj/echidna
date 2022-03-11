#include "Peripheral.h"


char legal_cmd[][10] = { "p", "d", "i", "P", "D", "I", "R", "tf", "tr"};
uint8_t steer_direct = 0, run_direct = 0;

void receive_data(void)
{
//    char buffer[20];
//	static uint8_t num=0;
//	static uint8_t state=0;
//	float data=0;
//	//uart_query(uartn, (uint8*)&buffer[num]);
//	if(buffer[num]=='('){//???��
//		state=1;
//		num++;
//	}
//	else if(state==1 && ((buffer[num]>=97 && buffer[num]<=122) || (buffer[num]>=65 && buffer[num]<=90))){
//		num++;
//		state=2;
//	}
//	else if(state==2 && buffer[num]>=48 && buffer[num]<=57 && buffer[num] != ')'){
//		num++;
//	}
//	else if(buffer[num] == ')'){
//		for (uint8 i=0;i+2<num;i++){
//			data += (buffer[2+i]-48)*int_pow(10, num-3-i);
//		}
//		switch(buffer[1]){
//			case 'p':
//			    angle_kp = data;
//				printf("angle_kp:%f",angle_kp);
//				break;
//			case 'd':
//			    angle_kd = data;
//				printf("angle_kd=%f", angle_kd);
//				break;
//			case 'i':
//                angle_ki = data;
//                printf("angle_kd=%f", angle_ki);
//                break;
//			case 'P':
//			    V_S_Kp = -data/10000000;//10
//			    printf("speed_kp=%f", V_S_Kp);
//				break;
//			case 'I':
//			    V_S_Ki = data/1000;//1
//                printf("speed_i=%f", V_S_Ki);
//                break;
//            case 'D':
//                V_S_Kd = data/10;//1
//                printf("speed_d=%f", V_S_Kd);
//                break;
//            case 'R':
//                rt_sem_release(Run_sem);
//                run_direct = 1;
//                break;
//            case 'r':
//                rt_sem_release(Run_sem);
//                run_direct = 0;
//                break;
//            case 'J':
//                //rt_sem_release(Direct_sem);
//                steer_direct = 0;
//                break;
//            case 'j':
//                //rt_sem_release(Direct_sem);
//                steer_direct = 1;
//                break;
//		}
//		num=0;
//		state=0;
//	}
}
