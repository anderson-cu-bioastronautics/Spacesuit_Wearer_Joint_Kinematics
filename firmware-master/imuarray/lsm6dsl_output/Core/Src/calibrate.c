/*
 * calibrate.c
 *
 * Calibrate gyros onboard in order to obtain angular velocity measurements in radians for two-speed integration
 *
 *  Created on: Nov 6, 2020
 *      Author: Young-Young
 */

#include "main.h"
#include "lsm6dsl.h"
#include "calibration.h"
#include "calibration_params1.h"
#include "gsl/gsl_vector.h"
#include "gsl/gsl_blas.h"

gsl_matrix *TKom[6];	// Gyro nonorthogonality*scale factor matrix

//void Init_3by3_Matrix(gsl_matrix *A, float init_values[3][3])
//{
//	int i, j;
//
//	for (i = 0; i < 3; i++)
//	{
//		for (j = 0; j < 3; j++)
//		{
//			gsl_matrix_set(A, i, j, init_values[i][j]);
//		}
//	}
//
//	return;
//}

void CalibrationSetup()		// Set up calibration parameter matrices
{
	gsl_matrix_view Kom;
	gsl_matrix_view Tom;

	for (int i = 0; i < 6; i++)
	{
		Kom = gsl_matrix_view_array(&Kom_init[i][0][0], 3, 3);	// Gyro scale factor matrix
		Tom = gsl_matrix_view_array(&Tom_init[i][0][0], 3, 3);	// Gyro nonorthogonality matrix
		TKom[i] = gsl_matrix_alloc(3, 3);				// Product of scale factor and nonorthogonality matrices

		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &Tom.matrix, &Kom.matrix, 0.0, TKom[i]);			// Compute product of T and K
	}

	return;
}

float PolyVal(float x, double poly[4])		// Evaluate polynomial using Horner's Method
{
	return poly[0] + x*(poly[1] + x*(poly[2] + x*poly[3])); // y = poly[0] + poly[1]*x + poly[2]*x^2 + poly[3]*x^3
}

void Calibrate(struct RxBuffer *rawData, struct CalibBuffer *calibData, int I[nIMUs], int csCount)
{
	double omLin[3];
	double omCalib[3] = {0.0};

	// Apply nonlinearity correction to gyro measurements
	omLin[0] = (double)PolyVal((float)rawData[csCount].RxBuffer_OMX[I[csCount]], &pnlom[csCount][0][0]);
	omLin[1] = (double)PolyVal((float)rawData[csCount].RxBuffer_OMY[I[csCount]], &pnlom[csCount][1][0]);
	omLin[2] = (double)PolyVal((float)rawData[csCount].RxBuffer_OMZ[I[csCount]], &pnlom[csCount][2][0]);
	gsl_vector_view omLinVec = gsl_vector_view_array(omLin, 3);
	gsl_vector_view omCalibVec = gsl_vector_view_array(omCalib, 3);

	// Apply nonorthogonality and scale factor corrections to gyro measurements
	gsl_blas_dgemv(CblasNoTrans, 1.0, TKom[csCount], &omLinVec.vector, 0.0, &omCalibVec.vector);

	// Write calibrated gyro measurements to calibData
	calibData[csCount].RxBuffer_OMX[I[csCount]] = (float)omCalib[0];
	calibData[csCount].RxBuffer_OMY[I[csCount]] = (float)omCalib[1];
	calibData[csCount].RxBuffer_OMZ[I[csCount]] = (float)omCalib[2];

	// Pass uncalibrated accelerometer measurements directly to calibData (to be calibrated offline)
	calibData[csCount].RxBuffer_AX[I[csCount]] = rawData[csCount].RxBuffer_AX[I[csCount]];
	calibData[csCount].RxBuffer_AY[I[csCount]] = rawData[csCount].RxBuffer_AY[I[csCount]];
	calibData[csCount].RxBuffer_AZ[I[csCount]] = rawData[csCount].RxBuffer_AZ[I[csCount]];

//		if (csCount == 5)
//			__NOP();

	return;
}
