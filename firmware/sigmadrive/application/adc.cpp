/*
 * adc.cpp
 *
 *  Created on: Sep 8, 2019
 *      Author: mstoilov
 */

#include "adc.h"

Adc::Adc()
{

}

Adc::~Adc()
{

}

void Adc::Attach(ADC_HandleTypeDef* hadc)
{
	hadc_ = hadc;

}


