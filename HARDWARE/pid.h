#ifndef __PID_H__
#define __PID_H__

//声明一个结构体类型
typedef struct 
{
	float target_val;//目标值
	float actual_val;//实际值
	float err;//当前偏差
	float err_last;//上次偏差
	float err_sum;//误差累计值
	float Kp,Ki,Kd;//比例，积分，微分系数
	
} tPid;

extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern tPid pidHW_Tracking;
extern tPid pidFollow;
extern tPid pidMPU6050YawMovement;

//声明函数
float P_Realize(tPid * pid,float actual_val);
void PID_Init(void);
float PI_Realize(tPid * pid,float actual_val);
float PID_Realize(tPid * pid,float actual_val);

#endif
