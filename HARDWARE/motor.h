#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

#define AIN1_SET	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET)
#define AIN1_RESET	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET)
#define BIN1_SET	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET)
#define BIN1_RESET	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET)

extern float Motor1_Speed;
extern float Motor2_Speed;
extern float speed;
extern uint8_t flag;

void Motor_Set(int Motor1,int Motor2);	//先右后左	-15~5X
void Motor_SetSpeed(float motor1, float motor2);
void Motor_AddSpeed(void);
void Motor_MinusSpeed(void);

#endif
