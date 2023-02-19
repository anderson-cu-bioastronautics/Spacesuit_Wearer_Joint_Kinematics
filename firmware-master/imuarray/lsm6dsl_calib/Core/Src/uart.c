/*
 * uart.c
 *
 *  Created on: Mar 2, 2020
 *      Author: Jiabin Lin
 */

#ifndef SRC_UART_C_
#define SRC_UART_C_



#endif
#include "main.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/*USART1 Initialization Function
 *USART1 GPIO
 *PA9     ------> USART1_TX
 *PA10    ------> USART1_RX
 *UART1 is connected to the Hirose Connector in PCB
*/

void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
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
}


/*UART2 Initialization Function
 *USART2 GPIO
 *PA2     ------> USART2_TX
 *PA3     ------> USART2_RX
 *UART2 is connected to USB in STM32
*/
void MX_USART2_UART_Init(void)
{
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
}

/*
 * Enable DMA controller clock
 */
void MX_DMA_Init(void)
{

  // DMA controller clock enable
  __HAL_RCC_DMA2_CLK_ENABLE();
  /* UART DMA
     DMA interrupt init
     DMA2_Stream2_IRQn interrupt configuration
     DMA2_Stream7_IRQn interrupt configuration
   */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

  /*SPI DMA
    DMA interrupt init
    DMA2_Stream0_IRQn interrupt configuration
    DMA2_Stream3_IRQn interrupt configuration
   */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}
