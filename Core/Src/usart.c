/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
typedef struct __FILE FILE;
extern uint8_t Usart1_ReadBuf[256];	//串口1 缓冲数组
extern uint8_t Usart1_ReadCount;	//串口1 接收字节计数
extern int8_t g_cThisState;
int g_lHW_State = 0;
uint8_t g_ucaUsart2ReceiveBuffer[10];
uint8_t g_ucUsart2ReceivCounter=0;
extern uint8_t g_ucUsart2ReceiveData;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

//重定向，使printf()具有串口输出的作�?
int fputc(int ch,FILE *stream)
{
	HAL_UART_Transmit(&huart1,( uint8_t *)&ch,1,0xFFFF);
	return ch;
}

//判断否接收完�?帧数�?
uint8_t Usart_WaitReasFinish(void)
{
	static uint16_t Usart_LastReadCount = 0;//记录上次的计数�??
	if(Usart1_ReadCount == 0)
	{
		Usart_LastReadCount = 0;
		return 1;//表示没有在接收数�?
	}
	if(Usart1_ReadCount == Usart_LastReadCount)//如果这次计数值等于上次计数�??
	{
		Usart1_ReadCount = 0;
		Usart_LastReadCount = 0;
		return 0;//已经接收完成�?
	}
	Usart_LastReadCount = Usart1_ReadCount;
	return 2;//表示正在接受�?
}

/*******************
*  @brief  摄像头串口协议解析函数 可以连接K210或openmv等
*  @param  data:串口接收到的每个字节
*  @return  
*
*******************/
void usartCamera_Receive_Data(uint8_t data)
{
	static uint8_t state = 0;//定义静态static 变量
	
	if(state==0&&data==0xA5) //判断第一个是不是帧头0xA5
	{
		state=1;//是帧头0xA5 赋值state=1 表示接收下一个数据
		//数据存储在数组中 "g_ucUsart2ReceivCounter++",这里是先用后加，比如g_ucUsart2ReceivCounter 初值为0，执行这个是先g_ucaUsart2ReceiveBuffer[0]=data，然后g_ucUsart2ReceivCounter++，即后g_ucUsart2ReceivCounter = 1的
		g_ucaUsart2ReceiveBuffer[g_ucUsart2ReceivCounter++] = data;
	}
	else if(state==1&&data==0xA6) //第二个是不是帧头0xA6
	{
		state=2;//如果第二个是帧头0xA6 赋值state=2 表示接收下一个数据
		g_ucaUsart2ReceiveBuffer[g_ucUsart2ReceivCounter++] = data;//保存数据
	}
	else if(state==2)//然后确定开头是0XA5 0XA6 就开始接收
	{
		g_ucaUsart2ReceiveBuffer[g_ucUsart2ReceivCounter++]=data;
		if(g_ucUsart2ReceivCounter>9||data==0x5B) state=3;  //接收大于9个或者接收到帧尾0X5B 就置位状态三
	}
	else if(state==3) //状态三
	{
		if(g_ucaUsart2ReceiveBuffer[g_ucUsart2ReceivCounter-1] == 0x5B)  //确定 最后一个是不是0x5B帧尾 是帧尾0x5B 就认为通信正确 处理数据
		{
			state = 0;					//这里就可以处理数据了、处理完记得清空数组和重置标志位与计数值
			g_ucUsart2ReceivCounter = 0;//清零计数值
			//比如根据数据设置红外旋转偏移状态

			//1.设置快速 慢速右边 左边 数字存储的变量意义: [0]和[1]:帧头、[2]:摄像头左边数第一个感兴趣区域、[3]:左边第二个、[4]:左边第三个、[5]:左边第四个、[6]:左边第五个、[7]:帧尾
			if(g_ucaUsart2ReceiveBuffer[6]==0&&g_ucaUsart2ReceiveBuffer[5]==0&&g_ucaUsart2ReceiveBuffer[3]==0&&g_ucaUsart2ReceiveBuffer[2]==0){
				g_cThisState=0;//前进
				g_lHW_State=22222;//设置这个显示在OLED上方便调试 五个值 以此从左向右表示 从左向右的五个区域
			}
			if(g_ucaUsart2ReceiveBuffer[6]==0&&g_ucaUsart2ReceiveBuffer[5]==1&&g_ucaUsart2ReceiveBuffer[3]==0&&g_ucaUsart2ReceiveBuffer[2]==0){
				g_cThisState=-1;//应该右转
				g_lHW_State=22212;	//表示右数第二个 识别到线
			}
			if(g_ucaUsart2ReceiveBuffer[6]==1&&g_ucaUsart2ReceiveBuffer[5]==0&&g_ucaUsart2ReceiveBuffer[3]==0&&g_ucaUsart2ReceiveBuffer[2]==0){
				g_cThisState=-2;//快速右转
				g_lHW_State=22221;
			}
			if(g_ucaUsart2ReceiveBuffer[6]==1&&g_ucaUsart2ReceiveBuffer[5]==1&&g_ucaUsart2ReceiveBuffer[3]==0&&g_ucaUsart2ReceiveBuffer[2]==0){
				g_cThisState=-3;//快速右转
				g_lHW_State=22211;
			}
			if(g_ucaUsart2ReceiveBuffer[6]==0&&g_ucaUsart2ReceiveBuffer[5]==0&&g_ucaUsart2ReceiveBuffer[3]==1&&g_ucaUsart2ReceiveBuffer[2]==0){
				g_cThisState=1;//应该左转
				g_lHW_State=21222;
			}
            if(g_ucaUsart2ReceiveBuffer[6]==0&&g_ucaUsart2ReceiveBuffer[5]==0&&g_ucaUsart2ReceiveBuffer[3]==0&&g_ucaUsart2ReceiveBuffer[2]==1){
				g_cThisState=2;//快速左转
				g_lHW_State=12222;
			}
            if(g_ucaUsart2ReceiveBuffer[6]==0&&g_ucaUsart2ReceiveBuffer[5]==0&&g_ucaUsart2ReceiveBuffer[3]==1&&g_ucaUsart2ReceiveBuffer[2]==1){
			 	g_cThisState=3;//快速左转
				g_lHW_State=11222;
			}

			//2.然后清空数组
			for(int i=0;i<10;i++) g_ucaUsart2ReceiveBuffer[i]=0x00;//清空数组
				
		}
		else //不是帧尾说明通信错误重新开始接收
		{
			state=0;
			g_ucUsart2ReceivCounter =0;
			for(int i=0;i<10;i++) g_ucaUsart2ReceiveBuffer[i]=0x00;//清空数组
		}
	}
	else
	{	//其他异常清空
		state=0;
		g_ucUsart2ReceivCounter =0;
		for(int i=0;i<10;i++) g_ucaUsart2ReceiveBuffer[i]=0x00;//清空数组
	}
}

/* UART 错误回调函数 处理串口错误 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) != RESET) //使用__HAL_UART_GET_FLAG宏检查UART的overrun错误标志位是否被置位。如果返回值不等于RESET，表示overrun错误标志位被置位，即发生了overrun错误
    {
        __HAL_UART_CLEAR_OREFLAG(huart);//使用__HAL_UART_CLEAR_OREFLAG宏清除UART的overrun错误标志位
        HAL_UART_Receive_IT(&huart2,&g_ucUsart2ReceiveData,1);  //使用HAL库函数启动UART2接收中断，并设置接收缓冲区的大小为1字节
    }
}

/* USER CODE END 1 */
