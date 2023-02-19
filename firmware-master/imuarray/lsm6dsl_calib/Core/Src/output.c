#include "main.h"
#include "lsm6dsl.h"
#include <string.h>

#define NWORDS 6*nIMUs					// Number of floats per UART message
#define NBYTES (NWORDS+1)*2						// Number of bytes per UART message

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

union tx_float tx_bytes[NWORDS+1];						// Pad with extra word as workaround to first byte data corruption when communicating with Teensy
union tx_float tx_bytesBuffer[100][NWORDS];
int iTx_bytesBuffer = 0;
int iTx_bytesBufferRollover = 0;
int nUARTCplt = 0;

void SendData(struct RxBuffer *rawData, int index)
{
	tx_bytes[0].t = 0;
	for (int k = 0; k < nIMUs; k++)
	{
		tx_bytes[k*6 + 1].t = (int16_t)rawData[k].RxBuffer_OMX[index];
		tx_bytes[k*6 + 2].t = (int16_t)rawData[k].RxBuffer_OMY[index];
		tx_bytes[k*6 + 3].t = (int16_t)rawData[k].RxBuffer_OMZ[index];
		tx_bytes[k*6 + 4].t = (int16_t)rawData[k].RxBuffer_AX[index];
		tx_bytes[k*6 + 5].t = (int16_t)rawData[k].RxBuffer_AY[index];
		tx_bytes[k*6 + 6].t = (int16_t)rawData[k].RxBuffer_AZ[index];
	}

	//UART
	HAL_UART_DMAResume(&huart1);								//Start UART
	HAL_UART_Transmit_DMA(&huart1, tx_bytes[0].tbytes, NBYTES);			//Transmit bytes with DMA

	for (int i = 1; i <= NWORDS; i++)
	{
		tx_bytesBuffer[iTx_bytesBuffer][i - 1] = tx_bytes[i];
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
	if (iTx_bytesBuffer > 1)
	{
		__NOP();
	}
}

// UART DMA handler
// Being call after done with the transmission
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_DMAPause(&huart1);									  //Pause UART
	nUARTCplt++;
	if (nUARTCplt > 99)
	{
		__NOP();
	}
}
