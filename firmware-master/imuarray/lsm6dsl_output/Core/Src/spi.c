/*
 * spi.c
 *
 *  Created on: Mar 2, 2020
 *      Author: Jiabin Lin
 */


/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
#include "main.h"
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
//SPI MCU support package (Msp) configuration in stm32f4xx_hal_msp.c file
/*SPI1 GPIO
   *PA5     ------> SPI1_SCK
   *PA6     ------> SPI1_MISO
   *PA7     ------> SPI1_MOSI
   */
void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;							//SPI Mode 3 configuration
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;						//SPI Mode 3 configuration
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;							//SPI Mode 3 configuration
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;			//SPI clock rate: system clock/pre-scaler
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_SPI1_Init_8BIT(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;						//SPI Mode 3 configuration for LSM6DSL SPI requirement
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;							//SPI Mode 3 configuration for LSM6DSL SPI requirement
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;			//SPI clock rate: system clock/pre-scaler
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

//SPI DMA function
//Input : tx/rx data address, Port and Pin for CS and byte to transmit and receive
//Output: None
void spi_DMA_RX(uint8_t *txdata, uint8_t *rxdata,GPIO_TypeDef* Port, uint16_t Pin, int byteNumber)
{
	 HAL_SPI_DMAResume(&hspi1);
	 HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET );
	 HAL_SPI_TransmitReceive_DMA( &hspi1, txdata, (uint8_t*)rxdata, byteNumber);
}

//SPI polling code, one byte per transmission
//Better use with 16 bits clock cycle confirguation
void spi_RX(uint16_t *txdata, uint8_t *rxdata,GPIO_TypeDef* Port, uint16_t Pin)
{
	 HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET );
	 HAL_SPI_TransmitReceive( &hspi1, (uint8_t*)txdata, rxdata, 1, 100);
	 while( hspi1.State == HAL_SPI_STATE_BUSY );
	 HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET );
}

//SPI write without reading it back
//Use in configuration
void spi_write(uint16_t *txadddata, GPIO_TypeDef* Port, uint16_t Pin)
{
	 HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET );
	 HAL_SPI_Transmit( &hspi1, (uint8_t*)txadddata, 1, 100);
	 while( hspi1.State == HAL_SPI_STATE_BUSY );
	 HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET );
}
