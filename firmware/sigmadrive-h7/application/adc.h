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
#include "stm32h743xx.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal_adc.h"
#include "stm32h7xx_ll_adc.h"

class Adc {
public:
	Adc();
	virtual ~Adc();
	void Attach(ADC_HandleTypeDef *hadc, uint32_t n_regranks, bool enable_irq);

	void InjectedSwTrig();
	uint32_t InjReadConversionData(uint32_t rank);
	uint32_t RegReadConversionData(uint32_t rank_index);


public:
	ADC_HandleTypeDef *hadc_;       /**< Handle to the hardware device structure */
	int32_t regdata_[16];           /**< Buffer for storing the DMA transfered data for the regular channels */
};

#endif /* APPLICATION_ADC_H_ */
