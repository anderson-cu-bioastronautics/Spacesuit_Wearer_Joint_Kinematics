#include <stdio.h>
#include <string.h>
#include "lsm6dsl.h"
#include "main.h"
#include "calibration.h"
//#define LOCAL_PROCESS_TIMEOUT 10000										//Define number to run the program at a specific amount of time(ms)
//#define POLLING															//Define POLLING for spi polling
#define SPIDMA																//Define SPIDMA for burst read
																			//Warning: switch data reading method need to power cycle for proper performance
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
extern int TEMPIndex;
extern int TMIndex;
extern GPIO_TypeDef* Ports;
extern uint16_t Pins;
extern struct OMEbar OMEbarOJ;
extern int sets;

int c[nIMUs]={0};														// c is counting the number of times each specific IMU is read

#ifdef SPIDMA
uint8_t EDMA[3] = {0x00,0x00,0x00};											// Register for FIFO status
uint8_t FIFO_S2[2] = {(READ|FIFO_STATUS2),0};								// FIFO status
volatile int FSM_CHECKED_FLAG = 0;											// Ensure FSM is checked only once per callback
int DP_FLAG = 0;															// Start Data Processing Flag
#endif

int main(void)
{
	HAL_Init();    															// Reset of all peripherals, Initializes the Flash interface and the Systick.
	SystemClock_Config();    												// Configure the system clock
	MX_GPIO_Init();   														// Initialize all configured peripherals
	MX_DMA_Init();															// Initialize DMA UART and SPI,placed before protocol configuration
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	//MX_TIM3_Init();														// PWM timer initialization
	MX_TIM4_Init();															// PWM timer initialization
	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);									// Initialize PWM to be stopped
	MX_SPI1_Init();
	struct RxBuffer rawData[nIMUs];											// Input buffer for raw data
	struct CalibBuffer calibData[nIMUs];										// Input buffer for calibrated data
	Buffer_Init(rawData);													// Initialize buffer contents to zeros
	Calib_Buffer_Init(calibData);
	int I[nIMUs] = {0};																// Declaration of buffer indices for each IMU
//	Indices_Init(&I);
	int rpNumbers_[nIMUs] = {0};											// rpNumber is to keep track the number of data left in the buffer(Mismatch)
	int csCount=0;															// csCount is the ID of each IMU
	int fPWMStarted = 0;
	uint32_t startTime = 0;

	resetAll();																// Reset all IMU
	setupAll();																// IMU configuration
	CalibrationSetup();														// Set up calibration parameters
	#ifdef SPIDMA
	MX_SPI1_Init_8BIT();													// Initialize SPI with 8 bit per transmission for burst read
	spi_DMA_RX(FIFO_S2, EDMA,Ports,Pins,2);									// Read FIFO_S2 into EDMA to initialize FSM
	#endif

	#ifdef LOCAL_PROCESS_TIMEOUT
	uint32_t timeout = HAL_GetTick() + (uint32_t)LOCAL_PROCESS_TIMEOUT;		//Time out program, change LOCAL_PROCESS_TIMEOUT to change the program duration
	#endif
	//HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);								//PWM(GPIO C6) start, use to synchronize IMUs FIFO rate
//	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15));								//Check if the Board Synchronization pin is pull up(Active high)
	//HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);								//PWM(GPIO B6) start, use to synchronize IMUs FIFO rate

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);									// Turn on red LED

	while (1)
	{
		//Define SPIDMA to use SPI burst read
		//Two Option: SPI burst2(one data) and SPI burst6(one entire set)
		//SPI burst6 FIFO Max speed 1200-1300 Hz with current code


		#ifdef SPIDMA
		#ifdef LOCAL_PROCESS_TIMEOUT
		if(HAL_GetTick()<=timeout)
		{
		#endif
//			switch(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15))					// Dedicated start/stop pin
			switch(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10))					// USART1 RX as start/stop pin
			{
				case 1:
					if (!fPWMStarted)
					{
						startTime = HAL_GetTick();										// Pause to let DHU UART ports initialize
						while ((HAL_GetTick() - startTime) < 100);
						HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);						//PWM(GPIO B6) start, use to synchronize IMUs FIFO rate
						fPWMStarted = 1;
						//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);			//Turn on green LED
					}

					if (!FSM_CHECKED_FLAG) DMA_FSM_BURST6(&csCount, rawData, I);			//Option:DMA_FSM_BURST6(&csCount,rawData,&I); DMA_FSM_BURST2(&csCount,rawData,&I)
																					//Notice: Make sure to change PWM rate at a right speed when switch to BURST2
																					//Otherwise: program is likely to enter hard fault interrupt because of buffer overflow
					if(DP_FLAG)														//Data retrieval completed (one set)
					{																//Start to process data
						DP_FLAG = 0;
						Calibrate(rawData, calibData, I, csCount);					// Calibrate raw data and store in calibData
						OMEmean(calibData, I, rpNumbers_, csCount);					// Take sum of gyro measurements from each IMU and average when all IMUs have produced data
						IntegrateOmBar(calibData);									// Integrate gyro measurements according to high speed algorithm from Savage (1998a)
						if(sets == 4)												// Once gyro measurements have been integrated over 8 time steps
						{
							TransmitData(calibData);											// Send data over UART
							ResetIntegration();										// Reset integration
						}
						I[csCount] = I[csCount] < RX_BUFFER_SIZE - 1? I[csCount]+1 : 0;	// Increment buffer index
						c[csCount]++;												//Count total number of frames read for each IMU
//						if (c[csCount] > 1000)
//							__NOP();
					}

//					if(!(HAL_GetTick() % 500))										// Flash Green LED
//					{
//						if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1))
//						{
//							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET );
//						}
//						else
//						{
//							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET );
//						}
//					}
					break;
				case 0:
					if (fPWMStarted)
					{
						HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
						fPWMStarted = 0;
						//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET );		// Turn off green LED
					}
					break;

			}
		#ifdef LOCAL_PROCESS_TIMEOUT
		}
		else __NOP();														//Set breakpoint for timeout if needed
		#endif
		#endif

		//Define POLLING to use SPI polling
		//SPI transmit one byte at a time (Max FIFO rate 790Hz with current code)

		#ifdef POLLING
		#ifdef LOCAL_PROCESS_TIMEOUT
		if(HAL_GetTick()<=timeout)
		{
		#endif
			switch(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15))
			{
				case 1:
					HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);						//PWM(GPIO B6) start, use to synchronize IMUs FIFO rate
					data_polling(&csCount,rawData,&I,c);
					OMEmean(rawData,I,rpNumbers_,csCount);
					IntegrateOmBar();
					break;
				case 0:
					HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
					break;
			}
		#ifdef LOCAL_PROCESS_TIMEOUT
		}
		else __NOP();														//Set breakpoint for timeout if needed
		#endif
		#endif
	}
}





void Error_Handler(void)
{
	while(1);
}


