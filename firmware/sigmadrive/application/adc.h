/*
 * adc.h
 *
 *  Created on: Sep 8, 2019
 *      Author: mstoilov
 */

#ifndef APPLICATION_ADC_H_
#define APPLICATION_ADC_H_

#define USE_HAL_ADC_REGISTER_CALLBACKS 1

#include <map>
#include <functional>
#include "stm32f745xx.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal_adc.h"
#include "stm32f7xx_ll_adc.h"


class Adc {
public:
	static const uint32_t ADC_HISTORY_SIZE = 20;

	Adc();
	virtual ~Adc();
	void Attach(ADC_HandleTypeDef* hadc);

	void InjectedSwTrig();
	void StartRegularConversions();
	void SyncDataDMA();

public:
	ADC_HandleTypeDef* hadc_;
	int32_t regdata_[16];
};

#endif /* APPLICATION_ADC_H_ */
