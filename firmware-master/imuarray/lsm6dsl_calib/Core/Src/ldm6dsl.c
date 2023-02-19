#include <string.h>
#include "lsm6dsl.h"
#include "main.h"


int TEMPIndex = 0;
int TMIndex = 0;
int16_t RxBuffer_TEMP[RX_BUFFER_SIZE] = {0};
uint32_t RxBuffer_TM[RX_BUFFER_SIZE] = {0};
extern struct RxBuffer My[nIMUs];
GPIO_TypeDef* Ports;
uint16_t Pins;





//Set struct My to zero after declaration
//Input My, pass by pointer
//Output NONE
void Buffer_Init(struct RxBuffer My[])
{
	for(int i = 0; i<nIMUs; i++)
	{
		memset (My[i].RxBuffer_OMX, 0, sizeof(My[i].RxBuffer_OMX));
		memset (My[i].RxBuffer_OMY, 0, sizeof(My[i].RxBuffer_OMY));
		memset (My[i].RxBuffer_OMZ, 0, sizeof(My[i].RxBuffer_OMZ));
		memset (My[i].RxBuffer_AX,  0, sizeof(My[i].RxBuffer_AX));
		memset (My[i].RxBuffer_AY,  0, sizeof(My[i].RxBuffer_AY));
		memset (My[i].RxBuffer_AZ,  0, sizeof(My[i].RxBuffer_AZ));
	}
}

/*
 * Check If FIFO is Empty and the number of data in FIFO
 * Method Polling
 * Input: num
 * Return number of data in the FIFO if num is true
 * else return if FIFO is empty
 */
uint16_t FIFO_unRead(int num)
{
	uint16_t FIFO_S1 = (READ|FIFO_STATUS1)<<8;
	uint16_t FIFO_S2 = (READ|FIFO_STATUS2)<<8;
	uint8_t temp[2] = {0};
	spi_RX(&FIFO_S1, &temp[0],Ports,Pins);
	spi_RX(&FIFO_S2, &temp[1],Ports,Pins);
	if(num)
	{
		return (uint8_t)((temp[1])&0x07)<<8 | (uint8_t)(temp[0]);
	}
	return (uint8_t)((temp[1]))<<8 | (uint8_t)(temp[0]);
}

//SPI chip selection
//Output: change CS GPIO globally
void chipSelection(int cs)
{
	switch(cs)
	{
	  case 0:
		  Ports = GPIOB;
		  Pins = GPIO_PIN_15;
		  HAL_GPIO_WritePin(Ports, Pins, GPIO_PIN_SET );
		  break;
	  case 1:
		  Ports = GPIOB;
		  Pins = GPIO_PIN_14;
		  HAL_GPIO_WritePin(Ports, Pins, GPIO_PIN_SET );
		  break;
	  case 2:
		  Ports = GPIOB;
		  Pins = GPIO_PIN_13;
		  HAL_GPIO_WritePin(Ports, Pins, GPIO_PIN_SET );
		  break;
	  case 3:
		  Ports = GPIOB;
		  Pins = GPIO_PIN_3;
		  HAL_GPIO_WritePin(Ports, Pins, GPIO_PIN_SET );
		  break;
	  case 4:
		  Ports = GPIOB;
		  Pins = GPIO_PIN_4;
		  HAL_GPIO_WritePin(Ports, Pins, GPIO_PIN_SET );
		  break;
	  case 5:
		  Ports = GPIOB;
		  Pins = GPIO_PIN_5;
		  HAL_GPIO_WritePin(Ports, Pins, GPIO_PIN_SET );
		  break;
	}
}
//Reset configuration for all IMUs on the board (not just ones in use)
//Wait 5ms for proper reset
void resetAll()
{
	//for(int i =0; i<nIMUs;i++)
//	for(int i = nIMUs-1; i>=0; i--)
	for(int i = 5; i>=0; i--)
	{
		chipSelection(i);
		IMU_reset(Ports, Pins);
		HAL_Delay(5);
	}
}
//Configure all IMUs on the board (not just ones in use)
void setupAll()
{
	//for (int i = 0; i<nIMUs; i++)
//	for(int i = nIMUs-1; i>=0; i--)
	for(int i = 5; i>=0; i--)
	{
		chipSelection(i);
		IMU_config(Ports, Pins);
		PEDO_config(Ports, Pins);
		TimeStamp_config(Ports, Pins);
		INTTrigger(Ports, Pins);											//Sync ODR with PWM
		FIFO_config(Ports, Pins);
	}
}

//Read Temperature from IMU not in IMU FIFO
//Method Polling,2 bytes per transmission
//Input CS
void read_TEMP(GPIO_TypeDef* Port, uint16_t Pin)
{
	uint16_t temp_add[2] = {(0x80|TEMP_OUT_L)<<8,(0x80|TEMP_OUT_H)<<8};
	uint8_t temp[2] = {0};
	spi_RX(&temp_add[0], &temp[0],Port,Pin);
	spi_RX(&temp_add[1], &temp[1],Port,Pin);
	RxBuffer_TEMP[TEMPIndex++] = (uint8_t)(temp[1]&0x00FF)<<8 | (uint8_t)(temp[0]&0x00FF);
	if(RX_BUFFER_SIZE==TEMPIndex)
	{
		TEMPIndex = 0;
	}
}


//Read FIFO
//Method Polling
//Input: CS
int16_t read_FIFO(GPIO_TypeDef *Port, uint16_t Pin)
{
	uint16_t FIFO_LOW = (READ|FIFO_DATA_OUT_L)<<8;
	uint16_t FIFO_HIGH = (READ|FIFO_DATA_OUT_H)<<8;
	uint8_t temp[2] = {0};
	spi_RX(&FIFO_LOW, &temp[0],Port,Pin);
	spi_RX(&FIFO_HIGH, &temp[1],Port,Pin);
	return (int16_t)((uint8_t)(temp[1])<<8 | (uint8_t)(temp[0]));
}


void PEDO_config(GPIO_TypeDef* Port, uint16_t Pin)
{
	uint16_t add[2] = {(0x80|CTRL10_C<<8),(0x80|INT1_CTRL<<8)};
	uint8_t r1[1] = {0};
	spi_RX(&add[0],r1,Port,Pin);
	uint8_t r2[1] = {0};
	spi_RX(&add[1],r2,Port,Pin);
	uint16_t pedo_config[6]  	= {
									(FUNC_CFG_ACCESS<<8|0x80),
									(CONFIG_PEDO_THS_MIN<<8|0x8E),
									(FUNC_CFG_ACCESS<<8|0x00),
									(CTRL1_XL<<8|accelcfg16k),///changed
									(CTRL10_C<<8|0b00110100),
									(INT1_CTRL<<8|0b10111011)
									};
	uint16_t pedo_configcheck[6] = {
									((0x80|FUNC_CFG_ACCESS)<<8),
									((0x80|CONFIG_PEDO_THS_MIN)<<8),
									((0x80|FUNC_CFG_ACCESS)<<8),
									((0x80|CTRL1_XL)<<8),
								    ((0x80|CTRL10_C)<<8),
									((0x80|INT1_CTRL)<<8)
									};

	uint8_t shouldbe[6] = {0};											//The actual configuration we write to register
	uint8_t actual_16[6] = {0};											//The actual configuration we read from register

	for(int i = 0; i < 6; i ++)
	{
		spi_write(&pedo_config[i],Port,Pin);
		spi_RX(&pedo_configcheck[i], &actual_16[i],Port,Pin);
		shouldbe[i] = pedo_config[i];
	}
}



void IMU_config(GPIO_TypeDef* Port, uint16_t Pin)
{
	uint16_t IMU_configbuffer[9] 	  = {(INT1_CTRL<<8|intctrl),
											(CTRL7_G<<8|ctrl7cfg),
											(CTRL10_C<<8|ctrl10cfg),
											(CTRL4_C<<8|ctrl4),
											(CTRL6_C<<8|ctrl6),
											(CTRL3_C<<8|ctrl3),
											(CTRL5_C<<8|ctrl5),
											(CTRL8_XL<<8|ctrl8cfg),
											(MASTER_CONFIG<<8|MASTER_SENSOR)};
	uint16_t IMU_configbuffercheck[9] = {((0x80|INT1_CTRL)<<8),
											((0x80|CTRL7_G)<<8),
											((0x80|CTRL10_C)<<8),
											((0x80|CTRL4_C)<<8),
											((0x80|CTRL6_C)<<8),
											((0x80|CTRL3_C)<<8),
											((0x80|CTRL5_C)<<8),
											((0x80|CTRL8_XL)<<8),
											((0x80|MASTER_CONFIG)<<8)};

	uint8_t shouldbe[9] = {0};											//The actual configuration we write to register
	uint8_t actual_16[9] = {0};										    //The actual configuration we read from register
	for (int i =0; i<9;i++)
	{
		spi_write(&IMU_configbuffer[i],Port,Pin);
		spi_RX(&IMU_configbuffercheck[i], &actual_16[i],Port,Pin);
		shouldbe[i] = IMU_configbuffer[i];
	}
}


void IMU_reset(GPIO_TypeDef* Port, uint16_t Pin)
{
	uint16_t reset_buffer[3] = {(CTRL3_C<<8|0b10000001),
								(CTRL10_C<<8|ctrl10_pedo_reset),
								(TIMESTAMP2_REG<<8|timerreset),

								};
	uint16_t reset_buffercheck[3] = {
									((0x80|CTRL3_C)<<8),
									((0x80|CTRL10_C)<<8),
									((0x80|TIMESTAMP2_REG)<<8),

									};
	uint8_t shouldbe[3] = {0,0,0};									//The actual configuration we write to register
	uint8_t actual_16[3] = {0,0,0};									//The actual configuration we read from register

	for (int i =0; i<3;i++)
	{
		spi_write(&reset_buffer[i],Port,Pin);
		spi_RX(&reset_buffercheck[i], &actual_16[i],Port,Pin);
		shouldbe[i] = reset_buffer[i];
	}
}



void TimeStamp_config(GPIO_TypeDef* Port, uint16_t Pin)
{
	uint16_t timestamp_buffer[4] 	=  {(CTRL1_XL<<8|0b11111100),
										(WAKE_UP_DUR<<8|wakeupcfg),
										(CTRL10_C<<8|0b110100),
										(MD1_CFG<<8|mdcfg)};

	uint16_t timestamp_buffercheck[4] = {((0x80|CTRL1_XL)<<8),
										((0x80|WAKE_UP_DUR)<<8),
										((0x80|CTRL10_C)<<8),
										((0x80|MD1_CFG)<<8)};
	uint8_t  shouldbe[4] = {0};											//The actual configuration we write to register
	uint8_t actual_16[4] = {0};											//The actual configuration we read from register
	for(int i = 0; i < 4; i ++)
	{
		spi_write(&timestamp_buffer[i],Port,Pin);
		spi_RX(&timestamp_buffercheck[i], &actual_16[i],Port,Pin);
		shouldbe[i] = timestamp_buffer[i];
	}
}
//Sync ODR with PWM
void INTTrigger(GPIO_TypeDef* Port, uint16_t Pin)
{
	uint16_t INTTrigger_buffer[2] = {
			(CTRL6_C<<8|ctrl6edge),
			(CTRL4_C<<8|ctrl4)
				};
	uint16_t INTTrigger_checkbuffer[2] = {
				((READ|CTRL6_C)<<8),
				((READ|CTRL4_C)<<8)
					};
	uint8_t  shouldbe[2] = {0};											//The actual configuration we write to register
	uint8_t actual_16[2] = {0};											//The actual configuration we read from register
	for(int i = 0; i <2;i++)
	{
		spi_write(&INTTrigger_buffer[i],Port,Pin);
		spi_RX(&INTTrigger_checkbuffer[i], &actual_16[i],Port,Pin);
		shouldbe[i] = INTTrigger_buffer[i];
	}


}



void FIFO_config(GPIO_TypeDef* Port, uint16_t Pin)
{
	uint16_t FIFO_config[9]      =  {(FIFO_CTRL5<<8|fiforeset),
									(CTRL2_G<<8|gyrocfg16k),//ODR
									(CTRL1_XL<<8|accelcfg16k),//ODR
									(FIFO_CTRL5<<8|0b00111000),
									(FIFO_CTRL3<<8|deccfgno),//deccfg833
									(FIFO_CTRL4<<8|notimestamp),//fifo4cfg),////notimestamp
									(FIFO_CTRL1<<8|0b11111011),
									(FIFO_CTRL2<<8|fifo2cfgNOts),
									(FIFO_CTRL5<<8|fifo16k)};
	uint16_t FIFO_configcheck[9] = {((0x80|FIFO_CTRL5)<<8),
									((0x80|CTRL2_G)<<8),
									((0x80|CTRL1_XL)<<8),
									((0x80|FIFO_CTRL5)<<8),
									((0x80|FIFO_CTRL3)<<8),
									((0x80|FIFO_CTRL4)<<8),
									((0x80|FIFO_CTRL1)<<8),
									((0x80|FIFO_CTRL2)<<8),
									((0x80|FIFO_CTRL5)<<8)};
	uint8_t  shouldbe[9] = {0}; 									//The actual configuration we write to register
	uint8_t actual_16[9] = {0};										//The actual configuration we read from register
	for (int i =0; i<9;i++)
	{
		spi_write(&FIFO_config[i],Port,Pin);
		spi_RX(&FIFO_configcheck[i], &actual_16[i],Port,Pin);

		shouldbe[i] = FIFO_config[i];
	}
}

int signshort(int x)
{
  if (x > 0x8000)
        return x - 0x10000;
  return x;
}
