/*
 * spi_polling.c
 *
 *  Created on: Aug 6, 2020
 *      Author: Jiabin Lin
 */

#include "main.h"
#include "lsm6dsl.h"
extern GPIO_TypeDef* Ports;
extern uint16_t Pins;
/*
 * Poll data from IMU FIFO (Least effective)
 * Input: csCount, My,I,c pass by pointer so can change the value in local scope
 *
 */
void data_polling(int *csCount,struct RxBuffer *My,int I[nIMUs],int *c)
{
	uint16_t N = FIFO_unRead(1);
	uint16_t E = FIFO_unRead(0);
	if(N==0 && ((E&isEmpty)==isEmpty))
	{
	  *csCount = *csCount ==(nIMUs-1)? 0 : *csCount+1;
	  chipSelection(*csCount);
	}
	else
	{
		My[*csCount].RxBuffer_OMX[I[*csCount]] = read_FIFO(Ports, Pins);
		My[*csCount].RxBuffer_OMY[I[*csCount]] = read_FIFO(Ports, Pins);
		My[*csCount].RxBuffer_OMZ[I[*csCount]] = read_FIFO(Ports, Pins);
		My[*csCount].RxBuffer_AX[I[*csCount]] = read_FIFO(Ports, Pins);
		My[*csCount].RxBuffer_AY[I[*csCount]] = read_FIFO(Ports, Pins);
		My[*csCount].RxBuffer_AZ[I[*csCount]] = read_FIFO(Ports, Pins);
		I[*csCount] = I[*csCount] < RX_BUFFER_SIZE-1? I[*csCount] + 1 : 0;
		if(c[*csCount]%(16*1) == 0)
		{
		  read_TEMP(Ports, Pins);
		}
		c[*csCount]++;
	}
}

