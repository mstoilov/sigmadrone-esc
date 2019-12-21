/*
 * adc.cpp
 *
 *  Created on: Sep 8, 2019
 *      Author: mstoilov
 */

#include <assert.h>
#include <string.h>
#include "adc.h"
#include "main.h"
#include "dcache.h"


Adc::Adc()
	: hadc_(nullptr)
{
	memset(regdata_, 0, sizeof(regdata_));
}

Adc::~Adc()
{

}

void Adc::Attach(ADC_HandleTypeDef* hadc)
{
	hadc_ = hadc;
	__HAL_ADC_ENABLE(hadc_);
	if (hadc == &hadc1) {
		__HAL_ADC_ENABLE_IT(hadc_, ADC_IT_JEOC);
		if (HAL_ADC_Start_DMA(hadc_, (uint32_t*) regdata_, 5) != HAL_OK) {
			/* Start Conversation Error */
			printf("Failed to start regular conversions\n");
		}
	}

}

void Adc::StartRegularConversions()
{
}

void Adc::InjectedSwTrig()
{
	LL_ADC_INJ_StartConversionSWStart(hadc_->Instance);
}
