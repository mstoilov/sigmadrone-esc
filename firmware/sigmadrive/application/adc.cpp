/*
 * adc.cpp
 *
 *  Created on: Sep 8, 2019
 *      Author: mstoilov
 */

#include <assert.h>
#include <string.h>
#include "adc.h"

Adc::handle_map_type Adc::handle_map_;

extern "C"
void AdcInjectedConvCpltCallback(struct __ADC_HandleTypeDef *hadc)
{
	Adc *adc = Adc::handle_map_[hadc];
	if (adc)
		adc->InjectedConvCpltCallback();
}

Adc::Adc()
	: hadc_(nullptr)
{
	memset(injhistory_, 0, sizeof(injhistory_));
}

Adc::~Adc()
{

}

void Adc::Attach(ADC_HandleTypeDef* hadc)
{
	hadc_ = hadc;
	assert(handle_map_.find(hadc_) == handle_map_.end());
	handle_map_[hadc_] = this;
	hadc_->InjectedConvCpltCallback = AdcInjectedConvCpltCallback;
//	LL_ADC_EnableIT_JEOS(hadc_->Instance);
//	LL_ADC_Enable(hadc_->Instance);

	__HAL_ADC_ENABLE(hadc_);
	__HAL_ADC_ENABLE_IT(hadc_, ADC_IT_JEOC);
}

void Adc::InjectedSwTrig()
{
	LL_ADC_INJ_StartConversionSWStart(hadc_->Instance);
}

void Adc::InjectedConvCpltCallback()
{
	injdata_[0] = injhistory_[injdata_counter_][0] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(hadc_->Instance, LL_ADC_INJ_RANK_1), LL_ADC_RESOLUTION_12B);
	injdata_[1] = injhistory_[injdata_counter_][1] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(hadc_->Instance, LL_ADC_INJ_RANK_2), LL_ADC_RESOLUTION_12B);
	injdata_[2] = injhistory_[injdata_counter_][2] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(hadc_->Instance, LL_ADC_INJ_RANK_3), LL_ADC_RESOLUTION_12B);

//	injdata_[injdata_counter_][0] = LL_ADC_INJ_ReadConversionData12(hadc_->Instance, LL_ADC_INJ_RANK_1);
//	injdata_[injdata_counter_][1] = LL_ADC_INJ_ReadConversionData12(hadc_->Instance, LL_ADC_INJ_RANK_2);
//	injdata_[injdata_counter_][2] = LL_ADC_INJ_ReadConversionData12(hadc_->Instance, LL_ADC_INJ_RANK_3);

	injdata_counter_ = (injdata_counter_ + 1) % ADC_HISTORY_SIZE;
}
