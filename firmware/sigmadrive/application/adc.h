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
	using handle_map_type = std::map<ADC_HandleTypeDef*, Adc*>;
	static const uint32_t ADC_HISTORY_SIZE = 20;

	Adc();
	virtual ~Adc();
	void Attach(ADC_HandleTypeDef* hadc);

	void InjectedConvCpltCallback();
	void InjectedSwTrig();
	void StartRegularConversions();
	static handle_map_type handle_map_;

public:
	ADC_HandleTypeDef* hadc_;
	uint32_t injhistory_[ADC_HISTORY_SIZE][3];
	uint32_t dir_ = 0;
	int32_t injdata_[4];
	int32_t bias_[3];
	int32_t regdata_[16];
	int32_t regvolt_[16];
	uint32_t injdata_counter_ = 0;
	uint32_t regdata_counter_ = 0;

};

#endif /* APPLICATION_ADC_H_ */
