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

#define NWORDS 9 + 3*nIMUs					// Number of floats per UART message
#define NBYTES (NWORDS)*4						// Number of bytes per UART message

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern union tx_float gam[3];					    //union is used to break float to bytes
extern union tx_float phi[3];
extern union tx_float acc[3*nIMUs];
extern struct OMEbar OMEbarOJ;
extern int sendIndex[nIMUs];

union tx_float tx_bytes[9+3*nIMUs];
union tx_float tx_bytesBuffer[100][NWORDS];
int iTx_bytesBuffer = 0;
int iTx_bytesBufferRollover = 0;

/*USART1 Initialization Function
 *USART1 GPIO
 *PA9     ------> USART1_TX
 *PA10    ------> USART1_RX
 *UART1 is connected to the Hirose Connector in PCB
*/

void MX_USART1_UART_Init(void)
{
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

void TransmitData(struct CalibBuffer *calibData)
{
	for (int i = 0; i < 3; i++)
	{
		tx_bytes[i].t = phi[i].t;
		tx_bytes[i+3].t = gam[i].t;								//Combine ga & phi together
		tx_bytes[i+6].t = OMEbarOJ.OME[i][0];
	}
//		tx_bytes[9].t = 0.0;										//Placeholder for temperature
	for (int k = 0; k < nIMUs; k++)
	{
		tx_bytes[k*3 + 9].t = calibData[k].RxBuffer_AX[sendIndex[k]];
		tx_bytes[k*3 + 10].t = calibData[k].RxBuffer_AY[sendIndex[k]];
		tx_bytes[k*3 + 11].t = calibData[k].RxBuffer_AZ[sendIndex[k]];
	}
//	for (int i = 0; i < 27; i++)
//	{
//		for (int j = 0; j < 4; j++)
//		{
//			tx_bytes[i].tbytes[j] = (uint8_t)(j + i*4);
//		}
//	}
	//UART
	HAL_UART_DMAResume(&huart1);								//Start UART
	HAL_UART_Transmit_DMA(&huart1, tx_bytes[0].tbytes, NBYTES);			//Transmit bytes with DMA
	for (int i = 0; i < NWORDS; i++)
	{
		tx_bytesBuffer[iTx_bytesBuffer][i] = tx_bytes[i];
	}
	if (iTx_bytesBuffer >= 99)
	{
		iTx_bytesBuffer = 0;
		iTx_bytesBufferRollover++;
	}
	else
	{
		iTx_bytesBuffer++;
	}
//		if (iTx_bytesBufferRollover > 0)
//	if (iTx_bytesBuffer > 4)
//	{
//		__NOP();
//	}
}

// UART DMA handler
// Being call after done with the transmission
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_DMAPause(&huart1);									  //Pause UART
}
