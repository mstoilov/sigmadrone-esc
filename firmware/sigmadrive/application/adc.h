/*
 * adc.h
 *
 *  Created on: Sep 8, 2019
 *      Author: mstoilov
 */

#ifndef APPLICATION_ADC_H_
#define APPLICATION_ADC_H_

#include <map>
#include <functional>
#include "stm32f745xx.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal_adc.h"


class Adc {
public:
	Adc();
	virtual ~Adc();
	void Attach(ADC_HandleTypeDef* hadc);


public:
	ADC_HandleTypeDef* hadc_;
};

#endif /* APPLICATION_ADC_H_ */
