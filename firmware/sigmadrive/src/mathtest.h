/*
 * mathtest.h
 *
 *  Created on: Feb 8, 2019
 *      Author: mstoilov
 */

#ifndef MATHTEST_H_
#define MATHTEST_H_

#include <complex>

double mul_double(double a, double b);

float mul_float(float a, float b);

float div_float(float a, float b);

int mul_int(int a, int b);

float dot(const std::complex<float>& a, const std::complex<float>& b);


#endif /* MATHTEST_H_ */
