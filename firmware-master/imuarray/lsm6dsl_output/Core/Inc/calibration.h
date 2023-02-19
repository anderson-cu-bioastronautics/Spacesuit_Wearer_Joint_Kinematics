/*
 * calibration.h
 *
 *  Created on: Sep 29, 2020
 *      Author: Young-Young Shen
 */

#include <math.h>
#include "gsl/gsl_vector.h"
#include "gsl/gsl_matrix.h"

#ifndef SRC_CALIBRATION_H_
#define SRC_CALIBRATION_H_

//void Init_3by3_Matrix(gsl_matrix *A, float init_values[3][3]);
void CalibrationSetup(void);
void Calibrate(struct RxBuffer *rawData, struct CalibBuffer *calibData, int I[nIMUs], int csCount);

#endif /* SRC_CALIBRATION_H_ */
