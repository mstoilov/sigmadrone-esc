#ifndef _BLDC_ADC_H_
#define _BLDC_ADC_H_

#include "bldc-include.h"


#ifdef __cplusplus
extern "C" {
#endif

void bldc_adc_init(ADC_TypeDef *ADCx);
void bldc_adc_start(ADC_TypeDef *ADCx);
void bldc_adc_stop(ADC_TypeDef *ADCx);


#ifdef __cplusplus
}
#endif

#endif
