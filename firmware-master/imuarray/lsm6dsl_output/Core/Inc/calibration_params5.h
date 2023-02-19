/*
 * calibration_params5.h
 *
 *  Created on: Jul 18, 2021
 *      Automatically generated using generate_calibration_params_h.m
 */

#ifndef CALIBRATION_PARAMS_H_
#define CALIBRATION_PARAMS_H_



#endif /* CALIBRATION_PARAMS_H_ */

#include "main.h"

// Define calibration coefficients
// Convention for all polynomials y = poly[0] + poly[1]*x + poly[2]*x^2 + poly[3]*x^3 NOTE: Opposite from MATLAB

double pnlom[6][3][4] = {{{-10.6340297695085, 1.00000000000000, -2.49004614809381E-10, -7.08845834331602E-11},		// Gyro nonlinearity polynomial coeffs IMU0
{39.0428181242183, 1.00000000000000, -1.88458330997478E-08, -1.33869306563328E-11},
{37.7379850464030, 1.00000000000000, -8.22909332800278E-08, -1.53170837804788E-11}},

{{-33.7617970052862, 1.00000000000000, -1.33592091785146E-08, -6.50729685515517E-11},		// Gyro nonlinearity polynomial coeffs IMU1
{74.4355431992742, 1.00000000000000, -3.62150016677459E-08, -1.11833399395700E-11},
{6.86392798853889, 1.00000000000000, -6.63420817046716E-08, -2.11498546895227E-11}},

{{-33.0253411820080, 1.00000000000000, -9.34573906427743E-09, -6.59565392758028E-11},		// Gyro nonlinearity polynomial coeffs IMU2
{80.1168664157362, 1.00000000000000, -1.79822088813644E-08, -1.71549756553818E-11},
{19.4984682518576, 1.00000000000000, -7.43178728995010E-08, -8.37613960957590E-12}},

{{-12.6372223151664, 1.00000000000000, -9.47035252561578E-09, -6.53979713605126E-11},		// Gyro nonlinearity polynomial coeffs IMU3
{44.6893972718411, 1.00000000000000, -4.56629636938099E-08, -1.09847324484052E-11},
{42.7397886332036, 1.00000000000000, -7.41902850035651E-08, 4.78242868219985E-13}},

{{-15.1007431677261, 1.00000000000000, -2.74594845333919E-08, -6.49332847297089E-11},		// Gyro nonlinearity polynomial coeffs IMU4
{52.5378642211596, 1.00000000000000, -5.18592248147898E-08, -1.24445109918167E-11},
{4.52308399669754, 1.00000000000000, -5.35808826519616E-08, -1.61822090966671E-11}},

{{-74.9524782320342, 1.00000000000000, 3.50300587770034E-09, -6.52098606362860E-11},		// Gyro nonlinearity polynomial coeffs IMU5
{86.7220926548752, 1.00000000000000, -3.31678883920968E-08, -9.15956270771416E-12},
{-29.4603398383553, 1.00000000000000, -6.18069801944203E-08, -2.80038578005767E-11}}};

double Kom_init[6][3][3] = {{{0.000622728308326465, 0.00000000000000, 0.00000000000000},		// Gyro scale factor matrix IMU0
{0.00000000000000, 0.000614408542796438, 0.00000000000000},
{0.00000000000000, 0.00000000000000, 0.000607507741698692}},

{{0.000618898475016765, 0.00000000000000, 0.00000000000000},		// Gyro scale factor matrix IMU1
{0.00000000000000, 0.000615696335228306, 0.00000000000000},
{0.00000000000000, 0.00000000000000, 0.000604671634749289}},

{{0.000610742985150295, 0.00000000000000, 0.00000000000000},		// Gyro scale factor matrix IMU2
{0.00000000000000, 0.000609687768106398, 0.00000000000000},
{0.00000000000000, 0.00000000000000, 0.000609180005898769}},

{{0.000616376972090038, 0.00000000000000, 0.00000000000000},		// Gyro scale factor matrix IMU3
{0.00000000000000, 0.000616787829538290, 0.00000000000000},
{0.00000000000000, 0.00000000000000, 0.000609814346726548}},

{{0.000617869368697955, 0.00000000000000, 0.00000000000000},		// Gyro scale factor matrix IMU4
{0.00000000000000, 0.000614318077076839, 0.00000000000000},
{0.00000000000000, 0.00000000000000, 0.000608231264312412}},

{{0.000619443282322872, 0.00000000000000, 0.00000000000000},		// Gyro scale factor matrix IMU5
{0.00000000000000, 0.000614625779174761, 0.00000000000000},
{0.00000000000000, 0.00000000000000, 0.000608382660844367}}};

double Tom_init[6][3][3] = {{{1.00000000000000, 0.0105674048195537, 0.0225473895984926},		// Gyro nonorthogonality matrix IMU0
{0.00430347084549616, 1.00000000000000, 0.00224370579538755},
{0.0209608201812470, 0.00182352204396173, 1.00000000000000}},

{{1.00000000000000, 0.0196821998422952, 0.00129953449663112},		// Gyro nonorthogonality matrix IMU1
{-0.00962595898227424, 1.00000000000000, -0.00902051063296526},
{0.00719347584165680, -0.0178396945066075, 1.00000000000000}},

{{1.00000000000000, 0.0102711669705929, 0.0102113146632884},		// Gyro nonorthogonality matrix IMU2
{0.00802629378362452, 1.00000000000000, 0.00125172992520850},
{0.000746665971025122, 0.0105006254429632, 1.00000000000000}},

{{1.00000000000000, 0.00394095079445726, 0.0210774203817036},		// Gyro nonorthogonality matrix IMU3
{0.0154940663572503, 1.00000000000000, -0.00384824469393137},
{0.0277308062997455, 0.00320417755036181, 1.00000000000000}},

{{1.00000000000000, 0.0198324979075956, 0.00773588555584939},		// Gyro nonorthogonality matrix IMU4
{-0.00337990800117085, 1.00000000000000, -0.00734163203534777},
{0.00784196503835981, -0.0131872786488954, 1.00000000000000}},

{{1.00000000000000, 0.00920009754201228, 0.0117894550486008},		// Gyro nonorthogonality matrix IMU5
{0.000447082422987213, 1.00000000000000, -0.0141332428687468},
{0.00924652118762191, -0.0255112166540751, 1.00000000000000}}};
