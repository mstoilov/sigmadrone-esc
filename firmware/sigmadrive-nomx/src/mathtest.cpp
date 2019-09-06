/*
 * mathtest.cpp
 *
 *  Created on: Feb 8, 2019
 *      Author: mstoilov
 */


#include "mathtest.h"

double mul_double(double a, double b)
{
	return a * b;
}


float mul_float(float a, float b)
{
	return a * b;
}

float div_float(float a, float b)
{
	return a / b;
}

int mul_int(int a, int b)
{
	return a * b;
}


float dot(const std::complex<float>& a, const std::complex<float>& b)
{
	return a.real() * b.real() + a.imag() * b.imag();
}
