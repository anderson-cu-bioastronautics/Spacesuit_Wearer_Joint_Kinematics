/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#define RX_BUFFER_SIZE  	 100
#define nIMUs			 	 0x06

void Error_Handler(void);

void MX_SPI1_Init(void);
void MX_SPI1_Init_8BIT(void);
void spi_write(uint16_t *txadddata, GPIO_TypeDef* Port, uint16_t Pin);
void spi_RX(uint16_t *txdata, uint8_t *rxdata,GPIO_TypeDef* Port, uint16_t Pin);
void spi_DMA_RX(uint8_t *txdata, uint8_t *rxdata,GPIO_TypeDef* Port, uint16_t Pin, int byteNumber);


void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_DMA_Init(void);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);

struct inputStruct
{

};

struct RxBuffer
{
	int16_t RxBuffer_OMX[RX_BUFFER_SIZE];
	int16_t RxBuffer_OMY[RX_BUFFER_SIZE];
	int16_t RxBuffer_OMZ[RX_BUFFER_SIZE];
	int16_t RxBuffer_AX[RX_BUFFER_SIZE];
	int16_t RxBuffer_AY[RX_BUFFER_SIZE];
	int16_t RxBuffer_AZ[RX_BUFFER_SIZE];

};
//Must be called after RxBuffer and Indices declaration
void DMA_FSM_BURST6(int *csCount,struct RxBuffer *My,int I[nIMUs]);
void DMA_FSM_BURST2(int *csCount,struct RxBuffer *My,int I[nIMUs]);
void data_polling(int *csCount,struct RxBuffer *My,int I[nIMUs],int *c);

//Break int16_t to bytes
union tx_float
{
	int16_t t;
	uint8_t tbytes[2];
};

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
