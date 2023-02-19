#include "main.h"
#include "lsm6dsl.h"
extern SPI_HandleTypeDef hspi1;
extern GPIO_TypeDef* Ports;
extern uint16_t Pins;
extern uint8_t EDMA[3];
extern uint8_t FIFO_S2[2];
extern int TEMPIndex;
extern int RxBuffer_TEMP[RX_BUFFER_SIZE];
extern int DP_FLAG;

uint8_t FIFO_ADD6[13] = {(READ|FIFO_DATA_OUT_L),0,0,0,0,0,0,0,0,0,0,0,0};	//FIFO address for burst 6
uint8_t TEMP6[25] = {0x00};													//Temporarily holder for result

uint8_t FIFO_ADD2[3] = {(READ|FIFO_DATA_OUT_L),0,0};						//FIFO address for burst 2
uint8_t TEMP2[5] = {0};														//Temporarily holder for result

uint8_t TEMP_ADD[2] = {(READ|TEMP_OUT_L),0};
uint8_t RTEMP[5] = {0};
volatile int state = 0;														// SPI FSM state for DMA
extern volatile int FSM_CHECKED_FLAG;										// Ensure FSM is checked only once per callback
volatile int BUFFER_ORDER = 0;												// Initialize buffer store order
volatile int fFIFO_DATA_AVAIL = 0;												// Initialize FIFO data available flag


/*
 *SPI DMA handler, being called after completed one transmit&receive
 *PULL up SPI CS pin after done with transmission, disable SPI DMA
 *Check if data in FIFO, Set flag accordingly for the FSM
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	while( hspi1.State == HAL_SPI_STATE_BUSY );
	HAL_GPIO_WritePin(Ports, Pins, GPIO_PIN_SET );
	HAL_SPI_DMAPause(&hspi1);
//	if(((EDMA[2]&isEmptyDMA)==isEmptyDMA))									// Check if FIFO is empty (logical AND between FIFO_S2 register value and isEmpty bit)
//	{
//		 state = 0;															// If empty,
//		 BUFFER_ORDER = 0;
//	}
//	else state = 1;
	FSM_CHECKED_FLAG = 0;													// Set flag to check FSM on next main program iteration

}

/*
 * DMA Burst 6 (Most efficiency)
 * Input: csCount,My and I (pass by pointer so can change the variable inside the local scope)
 * Output: None
 * Use Burst read to read 12 bytes of data at once
 * state 0: change IMU when the FIFO is empty
 * state 1: read execution
 * 			BUFFER_ORDER：  6 read execution
 * 			BUFFER_ORDER: 0 data storage
 */
void DMA_FSM_BURST6(int *csCount, struct RxBuffer *My, int I[nIMUs])
{
	FSM_CHECKED_FLAG = 1;
	if(((EDMA[2]&isEmptyDMA)==isEmptyDMA))													// Check if FIFO is empty (logical AND between FIFO_S2 register value and isEmpty bit)
	{																						// If empty, reset FIFO data available flag, change to next IMU in array, and read its FIFO status register
		fFIFO_DATA_AVAIL = 0;
		*csCount = *csCount ==(nIMUs-1)? 0 : *csCount+1;
		chipSelection(*csCount);
		spi_DMA_RX(FIFO_S2, EDMA, Ports, Pins,2);
	}
	else
	{																						// If not empty, initiate a FIFO read
		switch (fFIFO_DATA_AVAIL)
		{
			case 0:																			// If read was not initiated, initiate read
				fFIFO_DATA_AVAIL = 1;														// Set FIFO data available flag to indicate read was initiated
				spi_DMA_RX(&FIFO_ADD6[0], TEMP6, Ports, Pins, 13);								// Initiate read
				break;
			case 1:																			// If read was initiated, store data from FIFO into RxBuffer
				My[*csCount].RxBuffer_OMX[I[*csCount]] = (int16_t)((TEMP6[4]<<8)|(TEMP6[2]));
				My[*csCount].RxBuffer_OMY[I[*csCount]] =  (int16_t)((TEMP6[8]<<8)|(TEMP6[6]));
				My[*csCount].RxBuffer_OMZ[I[*csCount]] = (int16_t)((TEMP6[12]<<8)|(TEMP6[10]));
				My[*csCount].RxBuffer_AX[I[*csCount]] =  (int16_t)((TEMP6[16]<<8)|(TEMP6[14]));
				My[*csCount].RxBuffer_AY[I[*csCount]] = (int16_t)((TEMP6[20]<<8)|(TEMP6[18]));
				My[*csCount].RxBuffer_AZ[I[*csCount]] = (int16_t)((TEMP6[24]<<8)|(TEMP6[22]));
				DP_FLAG = 1;
				fFIFO_DATA_AVAIL = 0;
				spi_DMA_RX(FIFO_S2, EDMA, Ports, Pins, 2);
				break;
		}
	}
}

/*
 * DMA Burst 2 (relatively efficiency)
 * Input: csCount,My and I (pass by pointer so can change the variable inside the local scope)
 * Output: None
 * Use Burst read to read 2 bytes of data at once FIFO_DATA_HIGH & FIFO_DATA_LOW
 * state 0: change IMU when the FIFO is empty
 * state 1: read and store
 */
void DMA_FSM_BURST2(int *csCount,struct RxBuffer *My,int I[nIMUs])
{
	FSM_CHECKED_FLAG = 1;
	switch (state)
		{
			case 0:
				*csCount = *csCount ==(nIMUs-1)? 0 : *csCount+1;
				chipSelection(*csCount);
				spi_DMA_RX(FIFO_S2, EDMA, Ports, Pins,2);
				break;
			case 1:
				switch (BUFFER_ORDER)
				{
					case 6:
						BUFFER_ORDER = 0;
						spi_DMA_RX(&FIFO_ADD2[0], TEMP2,Ports,Pins,3);
						break;
					case 0:
						My[*csCount].RxBuffer_OMX[I[*csCount]] = (int16_t)((TEMP2[4]<<8)|(TEMP2[2]));
						BUFFER_ORDER = 1;
						spi_DMA_RX(&FIFO_ADD2[0], TEMP2,Ports,Pins,3);
						break;
					case 1:
						My[*csCount].RxBuffer_OMY[I[*csCount]] =  (int16_t)((TEMP2[4]<<8)|(TEMP2[2]));
						BUFFER_ORDER = 2;
						spi_DMA_RX(&FIFO_ADD2[0], TEMP2,Ports,Pins,3);
						break;
					case 2:
						My[*csCount].RxBuffer_OMZ[I[*csCount]] = (int16_t)((TEMP2[4]<<8)|(TEMP2[2]));
						BUFFER_ORDER = 3;
						spi_DMA_RX(&FIFO_ADD2[0], TEMP2,Ports,Pins,3);
						break;
					case 3:
						My[*csCount].RxBuffer_AX[I[*csCount]] =  (int16_t)((TEMP2[4]<<8)|(TEMP2[2]));
						BUFFER_ORDER = 4;
						spi_DMA_RX(&FIFO_ADD2[0], TEMP2,Ports,Pins,3);
						break;
					case 4:
						My[*csCount].RxBuffer_AY[I[*csCount]] = (int16_t)((TEMP2[4]<<8)|(TEMP2[2]));
						BUFFER_ORDER = 5;
						spi_DMA_RX(&FIFO_ADD2[0], TEMP2,Ports,Pins,3);
						break;
					case 5:
						My[*csCount].RxBuffer_AZ[I[*csCount]] = (int16_t)((TEMP2[4]<<8)|(TEMP2[2]));
						I[*csCount] = I[*csCount] < RX_BUFFER_SIZE - 1? I[*csCount]+1 : 0;
						DP_FLAG = 1;
						BUFFER_ORDER = 6;
						spi_DMA_RX(FIFO_S2, EDMA,Ports,Pins,2);
						break;
				}
				break;
		}
}
