/*
 * lsm6dsl.h
 *
 *  Created on: Jan 27, 2020
 *      Author: jiabinlin
 */

#include "main.h"

#define GYRO_XOUT_H       		0x23       				//X-gyro high bits
#define GYRO_XOUT_L       		0x22       				//X-gyro low bits
#define GYRO_YOUT_H       		0x25       				//Y-gyro high bits
#define GYRO_YOUT_L      		0x24       				//Y-gyro low bits
#define GYRO_ZOUT_H       		0x27       				//Z-gyro high bits
#define GYRO_ZOUT_L       		0x26       				//Z-gyro low bits
#define TEMP_OUT_H        		0x21
#define TEMP_OUT_L        		0x20
#define ACCEL_XOUT_H      		0x29       				//X-accel high bits
#define ACCEL_XOUT_L      		0x28       				//X-accel low bits
#define ACCEL_YOUT_H     		0x2B       				//Y-accel high bits
#define ACCEL_YOUT_L      		0x2A       				//Y-accel low bits
#define ACCEL_ZOUT_H      		0x2D       				//Z-accel high bits
#define ACCEL_ZOUT_L      		0x2C       				//Z-accel low bits
#define FIFO_DATA_OUT_L   		0x3E       				//FIFO low
#define FIFO_DATA_OUT_H   		0x3F       				//FIFO high

#define WHO_AM_I          		0x0F
#define TIMESTAMP2_REG    		0x42       				//register to reset timer
#define WAKE_UP_DUR       		0x5C       				//timer resolution
#define MD1_CFG           		0x5E       				//interrupts
#define MASTER_CONFIG    		0x1A
#define FUNC_CFG_ACCESS   		0x01
#define INT1_CTRL         		0x0D
#define FUNC_CFG_ACCESS   		0x01
#define CONFIG_PEDO_THS_MIN   	0x0F
#define FUNC_SRC1		  		0x53

#define FIFO_CTRL1   			0x06            		//FIFO threshold
#define FIFO_CTRL2   			0x07            		//FIFO to include temp
#define FIFO_CTRL3   			0x08            		//decimation rate
#define FIFO_CTRL4   			0x09            		//FIFO temp control
#define FIFO_CTRL5   			0xA             		//FIFO

#define CTRL1_XL  				0x10
#define CTRL2_G   				0x11
#define CTRL3_C   				0x12
#define CTRL4_C   				0x13
#define CTRL5_C   				0x14
#define CTRL6_C    				0x15
#define CTRL7_G    				0x16
#define CTRL8_XL   				0x17           			//Accel filter
#define CTRL10_C   				0x19           			//Cntrl register, to enable timestamps




#define FIFO_STATUS1   			0x3A          			//FIFO status
#define FIFO_STATUS2   			0x3B
#define FIFO_STATUS3   			0x3C
#define FIFO_STATUS4   			0x3D

#define READ   					0x80					//Write & Write Command
#define WRITE   				0x00


//IMU configuration
#define fifo1cfg   				0b11111011         		//threshold defalult 0
#define fifo2cfg   				0b10000111         		//set bit 3 to 1 for temp
#define fifo2cfgNOts			0b00000111
#define deccfg   				0b00001001
#define fifo5cfg   				0b00111001         		//data rate 833Hz
#define fifo4cfg   				0b10001000         		//only enable 1st & 2nd
#define notimestamp				0b10000000
#define fiforeset   			0b00000000
#define timerreset   			0xAA

#define wakeupcfg   			0b00010000         		//timestampresolution
#define mdcfg    				0b00000001
#define MASTER_SENSOR   		0b00010000
#define func_cfg   				0x0                 	//not using any function from bank A and B #was 0x80

#define gyrocfg833   			0b01111000           	//ODR with 833Hz, and full scale 1000dps
#define gyrocfg416   			0b01101000           	//ODR with 416Hz, and full scale 1000dps
#define gyrocfg33k				0b10011000
#define gyrocfg66k				0b10101000
#define gyrocfg16k				0b10001000				// ODR 1660 Hz, FS 1000 dps
#define accelcfg16k				0b10001100				// ODR 1660 Hz, FS 1000 dps
#define accelcfg833   			0b01111100          	//ODR with 833Hz, with 8g scale
#define accelcfg33k				0b10011100
#define accelcfg416				0b01101100
#define accelcfg66k				0b10101100
#define fifo466     			0b00110110			    //FIFOCTRL5
#define fifo833     			0b00111110
#define fifo16k     			0b01000110
#define fifo33k     			0b01001110
#define fifo66k					0b01010110
#define deccfg833				0b00001101 				//FIFOCTRL3
#define deccfg33k     			0b00001111
#define deccfgno				0b00001001

#define ctrl4   				0b10110100
#define ctrl5   				0b01100000           	//need to be change to 11000.. later
#define ctrl6   				0b01000010		    	 //~~???
#define ctrl3   				0b01000100
#define ctrl6edge				0b10000010           	//intctrl   0b00100011 /p
#define intctrl   				0b00111011           	//p
#define ctrl7cfg   				0b00000100
#define ctrl8cfg   				0b0
#define ctrl10cfg   			0b00110100
#define ctrl10_pedo_reset   	0b00000100   			//p reset



//check flag
#define fifo2_sample   			0b00000111        		//check if threshold reached
#define fifo_empty    			0b00010000        		//check if the fifo buffer is empty
#define totaldata      			0
#define isEmpty 		    	0b1000000000000
#define isEmptyDMA				0b10000



void resetAll();
void setupAll();
void INTTrigger  (GPIO_TypeDef* Port, uint16_t Pin);
void IMU_config  (GPIO_TypeDef* Port, uint16_t Pin);
void IMU_reset   (GPIO_TypeDef* Port, uint16_t Pin);
void FIFO_config (GPIO_TypeDef* Port, uint16_t Pin);
void PEDO_config (GPIO_TypeDef* Port, uint16_t Pin);
void TimeStamp_config(GPIO_TypeDef* Port, uint16_t Pin);
void read_TEMP   (GPIO_TypeDef* Port, uint16_t Pin);
int16_t read_FIFO(GPIO_TypeDef *Port, uint16_t Pin);
void chipSelection(int cs);
void Buffer_Init(struct RxBuffer My[]);
void Calib_Buffer_Init(struct CalibBuffer My[]);
//void Indices_Init(struct Indices *I);
uint16_t FIFO_unRead(int num);
void OMEmean(struct CalibBuffer *My, int I[nIMUs], int *rpNumbers, int csCount);
void IntegrateOmBar(struct CalibBuffer *calibData);
void ResetIntegration();
int signshort(int x);


