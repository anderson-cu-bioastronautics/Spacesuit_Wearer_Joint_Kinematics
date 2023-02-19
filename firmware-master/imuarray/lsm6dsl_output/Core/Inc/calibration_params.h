/*
 * calibration_params.h
 *
 *  Created on: Mar 19, 2021
 *      Author: Young-Young
 */

#ifndef CALIBRATION_PARAMS_H_
#define CALIBRATION_PARAMS_H_



#endif /* CALIBRATION_PARAMS_H_ */

#include "main.h"

// Define calibration coefficients
// Convention for all polynomials y = poly[0] + poly[1]*x + poly[2]*x^2 + poly[3]*x^3 NOTE: Opposite from MATLAB

//float ptom[3][4];							// Gyro thermal calibration coeffs
float pnlom[6][3][4] = {{{0, 1.0, 0.1, 0.01},		// Gyro nonlinearity polynomial coeffs IMU0
		{0, 1.0, 0.1, -0.01},
		{0, 1.0, -0.1, 0.01}},

		{{0, 1.0, 0.1, 0.01},		// Gyro nonlinearity polynomial coeffs IMU1
		{0, 1.0, 0.1, -0.01},
		{0, 1.0, -0.1, 0.01}},

		{{0, 1.0, 0.1, 0.01},		// Gyro nonlinearity polynomial coeffs IMU2
		{0, 1.0, 0.1, -0.01},
		{0, 1.0, -0.1, 0.01}},

		{{0, 1.0, 0.1, 0.01},		// Gyro nonlinearity polynomial coeffs IMU3
		{0, 1.0, 0.1, -0.01},
		{0, 1.0, -0.1, 0.01}},

		{{0, 1.0, 0.1, 0.01},		// Gyro nonlinearity polynomial coeffs IMU4
		{0, 1.0, 0.1, -0.01},
		{0, 1.0, -0.1, 0.01}},

		{{0, 1.0, 0.1, 0.01},		// Gyro nonlinearity polynomial coeffs IMU5
		{0, 1.0, 0.1, -0.01},
		{0, 1.0, -0.1, 0.01}}};

double Kom_init[6][3][3] = {{{0.0175*M_PI/180.0, 0, 0},		// Scale factor matrix IMU0
		{0, 0.0175*M_PI/180.0, 0},
		{0, 0, 0.0175*M_PI/180.0}},

		{{0.0175*M_PI/180.0, 0, 0},		// Scale factor matrix IMU1
		{0, 0.0175*M_PI/180.0, 0},
		{0, 0, 0.0175*M_PI/180.0}},

		{{0.0175*M_PI/180.0, 0, 0},		// Scale factor matrix IMU2
		{0, 0.0175*M_PI/180.0, 0},
		{0, 0, 0.0175*M_PI/180.0}},

		{{0.0175*M_PI/180.0, 0, 0},		// Scale factor matrix IMU3
		{0, 0.0175*M_PI/180.0, 0},
		{0, 0, 0.0175*M_PI/180.0}},

		{{0.0175*M_PI/180.0, 0, 0},		// Scale factor matrix IMU4
		{0, 0.0175*M_PI/180.0, 0},
		{0, 0, 0.0175*M_PI/180.0}},

		{{0.0180*M_PI/180.0, 0, 0},		// Scale factor matrix IMU5
		{0, 0.0180*M_PI/180.0, 0},
		{0, 0, 0.0180*M_PI/180.0}}};

double Tom_init[6][3][3] = {{{1.0, -0.001, 0.001},				// Nonorthogonality matrix IMU0
		{0.002, 1.0, -0.002},
		{-0.003, 0.003, 1.0}},

		{{1.0, -0.001, 0.001},				// Nonorthogonality matrix IMU1
		{0.002, 1.0, -0.002},
		{-0.003, 0.003, 1.0}},

		{{1.0, -0.001, 0.001},				// Nonorthogonality matrix IMU2
		{0.002, 1.0, -0.002},
		{-0.003, 0.003, 1.0}},

		{{1.0, -0.001, 0.001},				// Nonorthogonality matrix IMU3
		{0.002, 1.0, -0.002},
		{-0.003, 0.003, 1.0}},

		{{1.0, -0.001, 0.001},				// Nonorthogonality matrix IMU4
		{0.002, 1.0, -0.002},
		{-0.003, 0.003, 1.0}},

		{{1.0, -0.001, 0.001},				// Nonorthogonality matrix IMU5
		{0.002, 1.0, -0.002},
		{-0.003, 0.003, 1.0}}};
