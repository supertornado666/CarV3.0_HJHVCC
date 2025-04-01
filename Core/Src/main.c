/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "motor.h"
#include "niming.h"
#include "pid.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include "HC_SR04.h"

#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
cJSON *cJsonData ,*cJsonVlaue;	//上位机需要的参数
float p,i,d,a,b;				//发�?�到上位机的pid参数
extern float Mileage;			//里程
uint8_t OLEDString[50];			//OLED显示的字符串
uint8_t Usart3String[50];		//串口3显示的字符串
uint8_t g_ucaHW_Read[4] = {0};	//保存红外对管电平的数�?
int8_t g_cThisState = 0;		//这次状�??
int8_t g_cLastState = 0; 		//上次状�??
float g_fHW_PID_Out;			//红外对管PID计算输出速度
float g_fHW_PID_Out1;			//电机1的最后循迹PID控制速度
float g_fHW_PID_Out2;			//电机2的最后循迹PID控制速度
float g_fHC_SR04_Read;			//跟随功能实际距离
float g_fFollow_PID_Out;		//跟随功能应设速度
uint8_t g_ucUsart3ReceiveData;  //保存串口三接收的数据
uint8_t g_ucUsart2ReceiveData;  //保存串口二接收的数据
extern int g_lHW_State;//帮助视觉调试 用于表示红外对管或者视觉摄像头识别状态
uint8_t Car_Mode = 0;			//模式

float pitch,roll,yaw; //俯仰�? 翻滚�? 航向�?
float  g_fMPU6050YawMovePidOut = 0.00f; //姿�?�PID运算输出
float  g_fMPU6050YawMovePidOut1 = 0.00f; //第一个电机控制输�?
float  g_fMPU6050YawMovePidOut2 = 0.00f; //第一个电机控制输�?
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	OLED_Clear();
	PID_Init();

	HAL_Delay(500);//延时0.5�? 6050上电稳定后初始化
	MPU_Init();//初始化MPU6050
	while(MPU_Init()!=0);
	while(mpu_dmp_init()!=0);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//�?启定时器1 通道1 PWM输出
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//�?启定时器1 通道4 PWM输出
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);	//�?启定时器2
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);	//�?启定时器4
	HAL_TIM_Base_Start_IT(&htim2);					//�?启定时器2 中断
	HAL_TIM_Base_Start_IT(&htim4);                	//�?启定时器4 中断
	HAL_TIM_Base_Start_IT(&htim1);                //�?启定时器1 中断
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);	//�?启串�?1接收中断
	HAL_UART_Receive_IT(&huart3,&g_ucUsart3ReceiveData,1);  //串口三接收数�?
	HAL_UART_Receive_IT(&huart2,&g_ucUsart2ReceiveData,1);  //串口三接收数捿
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);// 启用UART2的错误中断功能

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
/**************************固定显示当前模式*****************************************************/
	sprintf((char *)OLEDString,"Mode:%d",Car_Mode);//模式
	OLED_ShowString(0,7,OLEDString,12);

	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char *)Usart3String),50);//阻塞式发送鿚过串口三输出字笿 strlen:计算字符串大�?
	sprintf((char *)Usart3String,"Mode:%d\r\n",Car_Mode);//显示当前模式
/**************************固定显示当前模式*****************************************************/

	if (Car_Mode == 0){
		Motor_SetSpeed(0,0);
/**************************OLED显示信息*****************************************************/
		sprintf((char *)OLEDString,"v1:%.2f v2:%.2f",Motor1_Speed,Motor2_Speed);//字符串格式化，两电机速度
		OLED_ShowString(0,0,OLEDString,12);
		sprintf((char *)OLEDString,"Mileage:%.2f",Mileage);//里程
		OLED_ShowString(0,1,OLEDString,12);
		sprintf((char *)OLEDString,"Battery:%.2fV",adcGetBatteryVoltage());//电池电压
		OLED_ShowString(0,2,OLEDString,12);
		sprintf((char *)OLEDString,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//障碍物距�?
		OLED_ShowString(0,3,OLEDString,12);
		sprintf((char *)OLEDString,"p:%.2f r:%.2f \r\n",pitch,roll);//俯仰�? 翻滚�?
		OLED_ShowString(0,4,OLEDString,12);
		sprintf((char *)OLEDString,"y:%.2f  \r\n",yaw);//航向�?
		OLED_ShowString(0,5,OLEDString,12);
/**************************OLED显示信息*****************************************************/
	  
/**************************串口3/蓝牙显示信息*****************************************************/  
		sprintf((char *)Usart3String,"v1:%.2fv2:%.2f\r\n",Motor1_Speed,Motor2_Speed);//显示两个电机转鿿 单位：转/�?
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char *)Usart3String),50);//阻塞式发送鿚过串口三输出字笿 strlen:计算字符串大�?
		sprintf((char *)Usart3String,"Mileage%.2fcm\r\n",Mileage);//计算小车里程 单位cm
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char *)Usart3String),50);//阻塞式发送鿚过串口三输出字笿 strlen:计算字符串大�?
		sprintf((char *)Usart3String,"Battery:%.2fV\r\n",adcGetBatteryVoltage());//显示电池电压
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char *)Usart3String),50);//阻塞式发送鿚过串口三输出字笿 strlen:计算字符串大�?
		sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//显示超声波数�?
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char *)Usart3String),50);//通过串口三输出字�? strlen:计算字符串大�?	
		
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0);  //这个可以解决经常读不出数据的问题
		sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//显示6050数据
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char *)Usart3String),50);//通过串口三输出字�? strlen:计算字符串大�?	
/**************************串口3/蓝牙显示信息*****************************************************/ 	  
	}

	if (Car_Mode == 1){
/**************************红外循迹功能*****************************************************/
		g_ucaHW_Read[0] = READ_HW_OUT_1;//读取红外对管状�?��?�这样相比于写在if里面更高�?
		g_ucaHW_Read[1] = READ_HW_OUT_2;
		g_ucaHW_Read[2] = READ_HW_OUT_3;
		g_ucaHW_Read[3] = READ_HW_OUT_4;

		if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
		{
			g_cThisState = 0;//直行
		}
		else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
		{
			g_cThisState = -1;//应该右偏
		}
		else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
		{
			g_cThisState = -2;//快�?�右�?
		}
		else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
		{
			g_cThisState = -3;//右转�?
		}
		else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
		{
			g_cThisState = 1;//应该左偏	
		}
		else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
		{
			g_cThisState = 2;//快�?�左�?
		}
		else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1 )
		{
			g_cThisState = 3;//左转�?
		}
		g_fHW_PID_Out = PID_Realize(&pidHW_Tracking,g_cThisState);//PID计算输出目标速度 这个速度，会和基�?速度加减

		g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;//电机1速度=基础速度+循迹PID输出速度
		g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;//电机2速度=基础速度-循迹PID输出速度，因为负值右转正值左�?
		if(g_fHW_PID_Out1 > 5) g_fHW_PID_Out1 = 5;//进行限幅 限幅速度�?0-5之间
		if(g_fHW_PID_Out1 < 0) g_fHW_PID_Out1 = 0;
		if(g_fHW_PID_Out2 > 5) g_fHW_PID_Out2 = 5;
		if(g_fHW_PID_Out2 < 0) g_fHW_PID_Out2 = 0;
		if(g_cThisState != g_cLastState)//如果这次状�?�不等于上次状�?��?�就进行改变目标速度和控制电机�?�在定时器中依旧定时控制电机
		{
			Motor_SetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);//通过计算的�?�度控制电机
		}

		g_cLastState = g_cThisState;//保存上次红外对管状�??
/**************************红外循迹功能*****************************************************/
	}
	
	if (Car_Mode == 2){
/****************************遥控模式*****************************************************/
					/*在stm32f1xx_it.c中执�?*/
/****************************遥控模式*****************************************************/
	}
	
	if (Car_Mode == 3){
/****************************避障功能*****************************************************/
		if (HC_SR04_Read() > 25){//若障碍物大于25cm，直�?100ms
			Motor_SetSpeed(1,1);
			HAL_Delay(100);
		}
		else {//否则，右�?400ms
			Motor_SetSpeed(-1,1);
			HAL_Delay(400);
			if (HC_SR04_Read() > 25){//若障碍物大于25cm，直�?100ms
				Motor_SetSpeed(1,1);
				HAL_Delay(100);
			}
			else {//否则，左�?800ms
				Motor_SetSpeed(1,-1);
				HAL_Delay(800);
				if (HC_SR04_Read() > 25){//若障碍物大于25cm，直�?100ms
					Motor_SetSpeed(1,1);
					HAL_Delay(100);
				}
				else {//否则，后�?1000ms，再右转200ms
					Motor_SetSpeed(-1,-1);
					HAL_Delay(1000);
					Motor_SetSpeed(-1,1);
					HAL_Delay(200);
				}
			}
		}
/****************************避障功能*****************************************************/
	}
	
	if (Car_Mode == 4){
/****************************跟随功能*****************************************************/
		g_fHC_SR04_Read = HC_SR04_Read();//读取当前距离
		
		if (g_fHC_SR04_Read < 60){//有跟随物�?
			g_fFollow_PID_Out = PID_Realize(&pidFollow,g_fHC_SR04_Read);//PID运算
			if (g_fFollow_PID_Out > 5){g_fFollow_PID_Out = 5;}//限幅
			if (g_fFollow_PID_Out < -5){g_fFollow_PID_Out = -5;}
			if (g_fFollow_PID_Out < 1.5 && g_fFollow_PID_Out > -1.5){g_fFollow_PID_Out = 0;}
			Motor_SetSpeed(g_fFollow_PID_Out,g_fFollow_PID_Out);
		}
		else {Motor_SetSpeed(0,0);}//无跟随物时停�?
		HAL_Delay(10);
/****************************跟随功能*****************************************************/
	}
	
	if (Car_Mode == 5){
/***********************MPU6050航向角PID控制*****************************************************/
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //这个可以解决经常读不出数据的问题
			
		g_fMPU6050YawMovePidOut = PID_Realize(&pidMPU6050YawMovement,yaw);//PID计算输出目标速度 这个速度，会和基�?速度加减

		g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;//基础速度加减PID输出速度
		g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
		if(g_fMPU6050YawMovePidOut1 > 3.5) g_fMPU6050YawMovePidOut1 = 3.5;//进行限幅
		if(g_fMPU6050YawMovePidOut1 < 0) g_fMPU6050YawMovePidOut1 = 0;
		if(g_fMPU6050YawMovePidOut2 > 3.5) g_fMPU6050YawMovePidOut2 = 3.5;
		if(g_fMPU6050YawMovePidOut2 < 0) g_fMPU6050YawMovePidOut2 = 0;
		Motor_SetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);
/***********************MPU6050航向角PID控制*****************************************************/
	}
	
	if(Car_Mode == 6){
		sprintf((char*)OLEDString, "lHW:%d  ", g_lHW_State);//视觉识别结果
		OLED_ShowString(0,6,OLEDString,12);
	
		g_fHW_PID_Out = PID_Realize(&pidOpenmv_Tracking,g_cThisState);//PID计算输出目标速度 这个速度，会和基础速度加减

		g_fHW_PID_Out1 = 0.5 + g_fHW_PID_Out;//电机1速度=基础速度+循迹PID输出速度
		g_fHW_PID_Out2 = 0.5 - g_fHW_PID_Out;//电机1速度=基础速度-循迹PID输出速度
		if(g_fHW_PID_Out1 >1.2) g_fHW_PID_Out1 =1.2;//进行限幅 限幅速度在0-1.2之间
		if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
		if(g_fHW_PID_Out2 >1.2) g_fHW_PID_Out2 =1.2;//进行限幅 限幅速度在0-1.2之间
		if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
		if(g_cThisState != g_cLastState)//如何这次状态不等于上次状态、就进行改变目标速度和控制电机、在定时器中依旧定时控制电机
		{
			Motor_SetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);//通过计算的速度控制电机
		}
		
		g_cLastState = g_cThisState;//保存上次红外对管状态
	}

/***************************发�?�上位机******************************************************/	  
//	ANO_DT_Send_F2(Motor1_Speed*100, 3.0*100,Motor2_Speed*100,3.0*100);	//F2帧发送到上位�?
//	if(Usart_WaitReasFinish() == 0)//是否接收完毕
//	{
//		cJsonData  = cJSON_Parse((const char *)Usart1_ReadBuf);
//		if(cJSON_GetObjectItem(cJsonData,"p") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"p");	
//			p = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Kp = p;
//			pidMotor2Speed.Kp = p;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"i") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"i");	
//			i = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Ki = i;
//			pidMotor2Speed.Ki = i;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"d") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"d");	
//			d = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Kd = d;
//			pidMotor2Speed.Kd = d;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"a") !=NULL)
//		{
//		
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"a");	
//			a = cJsonVlaue->valuedouble;
//			pidMotor1Speed.target_val =a;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"b") !=NULL)
//		{
//		
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"b");	
//			b = cJsonVlaue->valuedouble;
//			pidMotor2Speed.target_val =b;
//		}
//		if(cJsonData != NULL){
//		  cJSON_Delete(cJsonData);//释放空间、但是不能删除cJsonVlaue不然�? 出现异常错误
//		}
//		memset(Usart1_ReadBuf,0,255);//清空接收buf，注意这里不能使用strlen	
//	}
//	printf("P:%.1f I:%.1f D:%.1f A:%.1f B:%.1f\r\n",p,i,d,a,b);
/***************************发�?�上位机******************************************************/
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
