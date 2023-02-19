/*
 * calibration_params6.h
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

double pnlom[6][3][4] = {{{18.5431985213284, 1.00000000000000, -9.20323942096022E-08, -2.98073929128549E-11},		// Gyro nonlinearity polynomial coeffs IMU0
{75.5551424890524, 1.00000000000000, -3.79645358829942E-08, 6.17916827145200E-11},
{11.1577518626772, 1.00000000000000, 1.02930331832865E-07, 3.90848894781431E-11}},

{{-6.60161373941928, 1.00000000000000, -5.18185754032608E-08, -1.79113933427910E-11},		// Gyro nonlinearity polynomial coeffs IMU1
{45.7172013408328, 1.00000000000000, -6.08233548136483E-08, 6.56695782911608E-11},
{18.3182024703249, 1.00000000000000, 1.27180735094548E-07, 6.21965748100638E-11}},

{{-0.608687048296554, 1.00000000000000, -6.87518596848894E-08, -2.54682612416104E-11},		// Gyro nonlinearity polynomial coeffs IMU2
{41.7909882754090, 1.00000000000000, -5.17490116460632E-08, 5.74548164177933E-11},
{40.3355570064065, 1.00000000000000, 1.35199094383118E-07, 5.96751439493072E-11}},

{{-9.31084148533170, 1.00000000000000, -6.23706679654976E-08, -2.67245309241111E-11},		// Gyro nonlinearity polynomial coeffs IMU3
{65.6730031470042, 1.00000000000000, -2.77265966376727E-08, 7.40395265885941E-11},
{27.8781404523173, 1.00000000000000, 1.39878009246590E-07, 5.71162153666019E-11}},

{{-21.7880393361845, 1.00000000000000, -6.64921980951759E-08, -1.61216808611046E-11},		// Gyro nonlinearity polynomial coeffs IMU4
{153.784694685780, 1.00000000000000, -5.82562562647142E-09, 7.53439441037465E-11},
{65.2552144208828, 1.00000000000000, 1.26601034546693E-07, 4.92374576505059E-11}},

{{-36.8061497561244, 1.00000000000000, -5.45960982390406E-08, -7.85098343072362E-12},		// Gyro nonlinearity polynomial coeffs IMU5
{111.938073990608, 1.00000000000000, -2.63274087067509E-08, 6.25770269238790E-11},
{-3.28450485129275, 1.00000000000000, 1.24592572846127E-07, 4.37593951204682E-11}}};

double Kom_init[6][3][3] = {{{0.000625484379715042, 0.00000000000000, 0.00000000000000},		// Gyro scale factor matrix IMU0
{0.00000000000000, 0.000608647093751291, 0.00000000000000},
{0.00000000000000, 0.00000000000000, 0.000604667151306223}},

{{0.000620281621832832, 0.00000000000000, 0.00000000000000},		// Gyro scale factor matrix IMU1
{0.00000000000000, 0.000612911347282507, 0.00000000000000},
{0.00000000000000, 0.00000000000000, 0.000609584354525695}},

{{0.000618226344483770, 0.00000000000000, 0.00000000000000},		// Gyro scale factor matrix IMU2
{0.00000000000000, 0.000612225448491994, 0.00000000000000},
{0.00000000000000, 0.00000000000000, 0.000604919325532247}},

{{0.000622597085576256, 0.00000000000000, 0.00000000000000},		// Gyro scale factor matrix IMU3
{0.00000000000000, 0.000602916340834508, 0.00000000000000},
{0.00000000000000, 0.00000000000000, 0.000606051453630380}},

{{0.000621712937959161, 0.00000000000000, 0.00000000000000},		// Gyro scale factor matrix IMU4
{0.00000000000000, 0.000615032326921154, 0.00000000000000},
{0.00000000000000, 0.00000000000000, 0.000607895927391313}},

{{0.000610462061576895, 0.00000000000000, 0.00000000000000},		// Gyro scale factor matrix IMU5
{0.00000000000000, 0.000613264939211128, 0.00000000000000},
{0.00000000000000, 0.00000000000000, 0.000607011054940142}}};

double Tom_init[6][3][3] = {{{1.00000000000000, 0.0219099889531646, 0.00191753917999446},		// Gyro nonorthogonality matrix IMU0
{-0.00225382000047782, 1.00000000000000, -0.00194978093395885},
{-0.00981604373689681, 0.00706543412550289, 1.00000000000000}},

{{1.00000000000000, 0.00661596839568939, 0.0200596810917505},		// Gyro nonorthogonality matrix IMU1
{0.0153780855815010, 1.00000000000000, -0.000704886160050019},
{0.0205595769655983, 0.00257335677392275, 1.00000000000000}},

{{1.00000000000000, 0.00503244728911666, 0.0252011567355860},		// Gyro nonorthogonality matrix IMU2
{0.0113634763581412, 1.00000000000000, -0.00421128871295429},
{0.0160375710017144, -0.00422492004681714, 1.00000000000000}},

{{1.00000000000000, 0.0101143936629411, 0.0181405136515863},		// Gyro nonorthogonality matrix IMU3
{0.0141516400956478, 1.00000000000000, -0.00394072450473111},
{0.0160118895894607, -0.00707012355193582, 1.00000000000000}},

{{1.00000000000000, 0.00381469506509322, -0.0113929180514675},		// Gyro nonorthogonality matrix IMU4
{0.0203537486342721, 1.00000000000000, 0.00470463535306432},
{-0.0323407266760085, -0.00102976114651704, 1.00000000000000}},

{{1.00000000000000, 0.0151200822204765, 0.00637449516879277},		// Gyro nonorthogonality matrix IMU5
{0.000529632607913490, 1.00000000000000, -0.00277878076897936},
{0.000746631656073805, -0.00196941885947850, 1.00000000000000}}};

