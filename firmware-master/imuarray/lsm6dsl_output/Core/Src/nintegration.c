#include "main.h"
#include "lsm6dsl.h"
#include <string.h>

float Sum_OMX[RX_BUFFER_SIZE] = {0};
float Sum_OMY[RX_BUFFER_SIZE] = {0};
float Sum_OMZ[RX_BUFFER_SIZE] = {0};
int sumIndex[nIMUs][RX_BUFFER_SIZE] = {0};			// Indices of gyro measurements used to compute sum
int sendIndex[nIMUs] = {0};							// Index of accelerometer measurements to transmit, corresponding to transmitted average gyro measurement and phi
struct OMEbar OMEbarOJ =
{
		{
			{0},
			{0},
			{0}
		}
};

int OMEIndex = 0;
int cc = 0;
int LenArr = 0;

#define FS 800.0								//IMU Sampling Frequency IMPORTANT: Change Frequency Value after change PWM rate
double dt = (2.0/(FS));
float al[3][3]	   = {{0,0,0},{0,0,0},{0,0,0}};
float gd[3][3] 	   = {{0,0,0},{0,0,0},{0,0,0}};
float lastal[3] = {0, 0, 0};
float bd[3][3]     = {{0},{0},{0}};
float be[3] 	   = {0,0,0};
int alf = 0;
int sets = 0;
union tx_float gam[3];					    //union is used to break float to bytes
union tx_float phi[3];
//union tx_float ombar[3];
union tx_float acc[3*nIMUs];
union tx_float therm;

/*
 * Purpose: Average data after receive nIMUs IMUs' data
 * 			Store it the OMEbarOJ
 * rpNumbers: repeated numbers
 * 			 shows the number of data hasn't yet being processed
 * 			 Usually shows 0,1,2,3
 * 			 When rpNumebers shows hundreds, FIFO speed(PWM) is too fast
 */
void OMEmean(struct CalibBuffer *calibData, int I[nIMUs], int *rpNumbers, int csCount)
{
	int nFlag = 0;

	Sum_OMX[rpNumbers[csCount]] += calibData[csCount].RxBuffer_OMX[I[csCount]];
	Sum_OMY[rpNumbers[csCount]] += calibData[csCount].RxBuffer_OMY[I[csCount]];
	Sum_OMZ[rpNumbers[csCount]] += calibData[csCount].RxBuffer_OMZ[I[csCount]];
	sumIndex[csCount][rpNumbers[csCount]] = I[csCount];
	rpNumbers[csCount]++;

	if(LenArr < rpNumbers[csCount])
	{
		LenArr = rpNumbers[csCount];
	}

	for(int i = 0; i < nIMUs; i++)
	{
		if(rpNumbers[i]>0)
		{
			nFlag++;
		}
	}

	if(nFlag == nIMUs)
	{
		OMEbarOJ.OME[0][OMEIndex] =  (float) ((float)Sum_OMX[0]/(float)nIMUs);
		OMEbarOJ.OME[1][OMEIndex] =  (float) ((float)Sum_OMY[0]/(float)nIMUs);
		OMEbarOJ.OME[2][OMEIndex] =  (float) ((float)Sum_OMZ[0]/(float)nIMUs);
		OMEIndex = OMEIndex < RX_BUFFER_SIZE? OMEIndex+1 : 0;

		for (int k = 0; k < nIMUs; k++)	sendIndex[k] = sumIndex[k][0];

		for(int j = 1; j<(LenArr+1); j++)											// Shift sum arrays left by 1
		{
			Sum_OMX[j-1] = Sum_OMX[j];
			Sum_OMY[j-1] = Sum_OMY[j];
			Sum_OMZ[j-1] = Sum_OMZ[j];

			for (int k = 0; k < nIMUs; k++)	sumIndex[k][j - 1] = sumIndex[k][j];
		}

		for (int z = 0; z < nIMUs; z++)
		{
			if(z < nIMUs)
			{
				rpNumbers[z]--;
			}
		}
	}
}

void ArrShifter()
{
	for(int j = 2; j < OMEIndex+2; j++)
	{
		OMEbarOJ.OME[0][j-2] = OMEbarOJ.OME[0][j];
		OMEbarOJ.OME[1][j-2] = OMEbarOJ.OME[1][j];
		OMEbarOJ.OME[2][j-2] = OMEbarOJ.OME[2][j];
	}
}

//Purpose: numerical integration algorithm
//tx_bytes structure: phi (bytes 0-11), gamma (bytes 12-23), ombar, accelerometer
//Input: None
void IntegrateOmBar(struct CalibBuffer *calibData)
{
	if(OMEIndex%3 == 0 && OMEIndex > 0)
	{
		sets++;
		for(int i = 0; i<3;i++)
		{
			if(alf == 1)
			{
				al[0][i] = lastal[i];
			}
			alf = 1;
			al[1][i] = al[0][i] + 0.25 * dt  * (OMEbarOJ.OME[i][0] + OMEbarOJ.OME[i][1]);					// Trapezoidal rule to calculate intermediate alpha
			al[2][i] = al[0][i] + dt/6 * (OMEbarOJ.OME[i][0] + 4*OMEbarOJ.OME[i][1] + OMEbarOJ.OME[i][2]);
			lastal[i] = al[2][i];
		}

		for(int i = 0; i<3;i++)
		{
			bd[i][0] = 0.5*((al[i][1]*OMEbarOJ.OME[2][i]) - (al[i][2]*OMEbarOJ.OME[1][i]));
			bd[i][1] = 0.5*((al[i][2]*OMEbarOJ.OME[0][i]) - (al[i][0]*OMEbarOJ.OME[2][i]));
			bd[i][2] = 0.5*((al[i][0]*OMEbarOJ.OME[1][i]) - (al[i][1]*OMEbarOJ.OME[0][i]));
			for(int j =0;j<3;j++)
			{
				gd[i][j] = 0.5* (al[i][j] - (((sets-1)*2+i)*(dt/2) * OMEbarOJ.OME[j][i]));
			}
		}

		for(int i = 0; i<3; i++)
		{
			be[i] += (dt/6)*((bd[0][i]) + 4*(bd[1][i]) + (bd[2][i]));
			gam[i].t += (dt/6)*(gd[0][i] + 4*gd[1][i] + gd[2][i]);
			phi[i].t = al[2][i]+be[i];
		}
//		if (sets > 2)
//		{
//			__NOP();
//		}
		ArrShifter();
		OMEIndex = OMEIndex - 2;
	}
}

void ResetIntegration()
{
	for(int i = 0; i < 3; i++)
	{
		lastal[i] = 0;
		be[i] = 0;
		gam[i].t = 0;
		phi[i].t = 0;
		for(int j = 0; j < 3; j++)
		{
			al[j][i] = 0;
			bd[j][i] = 0;
			gd[j][i] = 0;
		}
	}
	sets=0;
}
