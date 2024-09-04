#include "motor.h"
#include "tim.h"
#include "pid.h"

#define MAX_PSPEED	3.2

float speed = 0.8;
uint8_t flag = 1;

//以占空比设置转速
void Motor_Set(int Motor1,int Motor2){
	//方向
	Motor1 > 0 ? BIN1_SET : BIN1_RESET;
	Motor2 > 0 ? AIN1_SET : AIN1_RESET;
	
	//防溢出，pwm占空比设置速度
	if (Motor1 < 0){
		if (Motor1 < -99){
			Motor1 = -99;
		}
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,-Motor1);
	}
	else{
		if (Motor1 > 99){
			Motor1 = 99;
		}
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,100 - Motor1);
	}
	if (Motor2 < 0){
		if (Motor2 < -99){
			Motor2 = -99;
		}
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,-Motor2);
	}
	else{
		if (Motor2 > 99){
			Motor2 = 99;
		}
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,100 - Motor2);
	}
}

//通过pid设置转速
//	  Motor_SetSpeed(1,2);	//右
//	  Motor_SetSpeed(2,1);	//左
//	  Motor_SetSpeed(1,1);	//前
//	  Motor_SetSpeed(-1,-1);//后
//	  Motor_SetSpeed(-1,1);//顺时针
//	  Motor_SetSpeed(1,-1);//逆时针
void Motor_SetSpeed(float motor1, float motor2){
	//设置pid目标转速，以圈为单位
	pidMotor1Speed.target_val=motor1;
	pidMotor2Speed.target_val=motor2;
	
	//立即设置，保证实时性
	Motor_Set(PID_Realize(&pidMotor1Speed,Motor1_Speed),PID_Realize(&pidMotor2Speed,Motor2_Speed));
}

void Motor_AddSpeed(void){
	if (speed < MAX_PSPEED) speed += 0.8;
	
	if (flag){Motor_SetSpeed(speed,speed);}
	else {Motor_SetSpeed(-speed,-speed);}
}

void Motor_MinusSpeed(void){
	if (speed >= 0.8) speed -= 0.8;
	
	if (flag){Motor_SetSpeed(speed,speed);}
	else {Motor_SetSpeed(-speed,-speed);}
}
