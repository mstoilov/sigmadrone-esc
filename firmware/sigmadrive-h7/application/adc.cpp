/*
 * adc.cpp
 *
 *  Created on: Sep 8, 2019
 *      Author: mstoilov
 */

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "adc.h"
#include "main.h"
#include "dcache.h"



Adc::Adc() : hadc_(nullptr)
{
	memset(regdata_, 0, sizeof(regdata_));
}

Adc::~Adc()
{
}

/** Attach this object to the hardware ADC device
 *
 * This method attaches to the hardware level device and
 * starts the DMA data acquisition for the configured channels.
 * It also enables the ADC_IT_JEOC IRQ that signals the completion
 * of sampling of the phase current. This is a very important IRQ
 * because it drives the @ref MotorDrive::IrqUpdateCallback and thus
 * the whole servo drive operation.
 *
 * @param hadc handle to the hardware level device
 */
void Adc::Attach(ADC_HandleTypeDef *hadc, uint32_t n_regranks, bool enable_irq)
{
	hadc_ = hadc;
	// __HAL_ADC_ENABLE(hadc_);
	LL_ADC_Enable(hadc_->Instance);
	if (enable_irq)
		__HAL_ADC_ENABLE_IT(hadc_, ADC_IT_JEOC);
	if (HAL_ADC_Start_DMA(hadc_, (uint32_t*) regdata_, n_regranks) != HAL_OK) {
		/* Start Conversation Error */
		printf("Failed to start regular conversions for ADC\n");
	}
}

/** Trigger the injected channels conversion
 *
 * This method triggers the ADC data acquisition for the
 * injected channels.
 *
 */
void Adc::InjectedSwTrig()
{
	// LL_ADC_INJ_StartConversionSWStart(hadc_->Instance);
	LL_ADC_INJ_SetTriggerSource(hadc_->Instance, LL_ADC_INJ_TRIG_SOFTWARE);
	LL_ADC_INJ_StartConversion(hadc_->Instance);
}

/** Read injected channel conversion data
 *
 * @param  Rank This parameter can be one of the following values:
 *         @arg @ref LL_ADC_INJ_RANK_1
 *         @arg @ref LL_ADC_INJ_RANK_2
 *         @arg @ref LL_ADC_INJ_RANK_3
 *         @arg @ref LL_ADC_INJ_RANK_4
 * @return The ADC data for the specified rank
 */
uint32_t Adc::InjReadConversionData(uint32_t rank)
{
	return LL_ADC_INJ_ReadConversionData12(hadc_->Instance, rank);
}


uint32_t Adc::RegReadConversionData(uint32_t rank_index)
{
	return regdata_[rank_index];
}
