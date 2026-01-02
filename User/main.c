#include "stm32f10x.h"  // Device header

#include "Delay.h"
#include "Timer.h"

#include "LED.h"
#include "Serial.h"
#include "Motor.h"
#include "Encoder.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>




//PID相关全局变量
volatile float Target_1 = 0, Actual_1 = 0, Out_1 = 0;			//目标值，实际值，输出值
volatile float Kp_1 = 0, Ki_1 = 0, Kd_1 = 0;	//比例项，积分项，微分项的权重
volatile float Error0_1 = 0, Error1_1 = 0, IntOut_1 = 0;		//本次误差，上次误差，误差积分

volatile float Target_2 = 0, Actual_2 = 0, Out_2 = 0;			//目标值，实际值，输出值
volatile float Kp_2 = 2, Ki_2 = 0, Kd_2 = 0;		//比例项，积分项，微分项的权重
volatile float Error0_2 = 0, Error1_2 = 0, IntOut_2 = 0;		//本次误差，上次误差，误差积分

volatile uint16_t TimeTick = 0;						//计时器（应用在定时器定时中断）

int main()
{	
	//模块初始化调用
	LED_Init();
	Serial_Init();
	Motor_Init();
	Encoder_Init();
	
	Serial_Printf("[display-clear]");
	Serial_Printf("[display,0,0,Tar1   Act1   Out1]");
	Serial_Printf("[display,0,40,Tar2   Act2   Out2]");
	
	Serial_RxFlag = 0;
	
	while(1)
	{
		
		
		
		
		/* ==================== [START] 蓝牙接收数据处理模块 [START] ==================== */
		if (Serial_RxFlag == 1)
		{			
			//字符串分割
			char * Tag = strtok(Serial_RxPacket, ",");
			
//			if (strcmp(Tag, "key") == 0)
//			{
//				//NULL表示为后续分割
//				char * Name = strtok(NULL, ",");
//				//如果没有新的子串，函数会返回空指针
//				char * Action = strtok(NULL, ",");
//				
//				if (strcmp(Name, "1") == 0 && strcmp(Action, "up") == 0)
//				{
//					printf("Key,1,up\r\n");
//				}
//				else if (strcmp(Name, "2") == 0 && strcmp(Action, "down") == 0)
//				{
//					printf("Key,1,down\r\n");
//				}
//			}
//			else 
			if (strcmp(Tag, "slider") == 0)
			{
				//NULL表示为后续分割
				char * Name = strtok(NULL, ",");
				//如果没有新的子串，函数会返回空指针
				char * Value = strtok(NULL, ",");
				
				if (strcmp(Name, "Kp1") == 0)
				{
					//字符串转换为int
					float FloatValue = atof(Value);
					Kp_1 = FloatValue;
				}
				else if (strcmp(Name, "Ki1") == 0)
				{
					//字符串转换为double
					float FloatValue = atof(Value);
					Ki_1 = FloatValue;
				}
				else if (strcmp(Name, "Kd1") == 0)
				{
					//字符串转换为double
					float FloatValue = atof(Value);
					Kd_1 = FloatValue;
				}
				else if (strcmp(Name, "Ta1") == 0)
				{
					//字符串转换为double
					float FloatValue = atof(Value);
					Target_1 = FloatValue;
				}
				else if (strcmp(Name, "Kp2") == 0)
				{
					//字符串转换为int
					float FloatValue = atof(Value);
					Kp_2 = FloatValue;
				}
				else if (strcmp(Name, "Ki2") == 0)
				{
					//字符串转换为double
					float FloatValue = atof(Value);
					Ki_2 = FloatValue;
				}
				else if (strcmp(Name, "Kd2") == 0)
				{
					//字符串转换为double
					float FloatValue = atof(Value);
					Kd_2 = FloatValue;
				}
				else if (strcmp(Name, "Ta2") == 0)
				{
					//字符串转换为double
					float FloatValue = atof(Value);
					Target_2 = FloatValue;
				}
			}
//			else if (strcmp(Tag, "joystick") == 0)
//			{
//				//左摇杆横向值
//				int8_t LH = atoi(strtok(NULL, ","));
//				//左摇杆纵向值
//				int8_t LV = atoi(strtok(NULL, ","));
//				//右摇杆横向值
//				int8_t RH = atoi(strtok(NULL, ","));
//				//右摇杆纵向值
//				int8_t RV = atoi(strtok(NULL, ","));
//				
//				printf("joystick, %d, %d, %d, %d\r\n", LH, LV, RH, RV);
//			}
							
			Serial_RxFlag = 0;
		}
		/* ==================== [END] 蓝牙接收数据处理模块 [END] ==================== */
		
		
		
		
		/* ==================== [START] 蓝牙回传数据模块 [START] ==================== */
		Serial_Printf("[display,0,20,%03.1f  %03.1f  %03.1f]", Target_1, Actual_1, Out_1);
		Serial_Printf("[display,0,60,%03.1f  %03.1f  %03.1f]", Target_2, Actual_2, Out_2);
		/* ==================== [END] 蓝牙回传数据模块 [END] ==================== */
		
		
		
		
	}
}

//1ms定时器
void TIM1_UP_IRQHandler(void)
{
	//检查标志位
	if (TIM_GetITStatus(TIM1,TIM_IT_Update) == SET )
	{
		TimeTick ++;
		if (TimeTick >= 40)
		{
			TimeTick = 0;
			
			//编码器数据读取与初步处理
			Actual_1 = Encoder1_Get();
			Actual_2 = Encoder2_Get();
			
			Error1_1 = Error0_1;
			Error0_1 = Target_1 - Actual_1;		
			
			Error1_2 = Error0_2;
			Error0_2 = Target_2 - Actual_2;
			
			//积分限幅
			IntOut_1 += Ki_1 * Error0_1;
			if (IntOut_1 >  99){IntOut_1 =  99;}
			if (IntOut_1 < -99){IntOut_1 = -99;}
			
			IntOut_2 += Ki_2 * Error0_2;
			if (IntOut_2 >  99){IntOut_2 =  99;}
			if (IntOut_2 < -99){IntOut_2 = -99;}
			
			//输出限幅
			Out_1 = Kp_1 * Error0_1 + IntOut_1 + Kd_1 * (Error0_1 - Error1_1);
			if (Out_1 >  99){Out_1 =  99;}
			if (Out_1 < -99){Out_1 = -99;}
			
			Out_2 = Kp_1 * Error0_2 + IntOut_2 + Kd_2 * (Error0_2 - Error1_2);
			if (Out_2 >  99){Out_2 =  99;}
			if (Out_2 < -99){Out_2 = -99;}
			
			//输出
			Motor_SetPWM1(Out_1);
			Motor_SetPWM2(Out_2);
			
		}
		
		
		//清除标志位
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	}
}
