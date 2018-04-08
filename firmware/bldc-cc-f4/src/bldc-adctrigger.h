#ifndef _BLDC_ADCTRIGGER_H_
#define _BLDC_ADCTRIGGER_H_

#include "bldc-include.h"
#include "bldc-timer.h"


#ifdef __cplusplus
extern "C" {
#endif

void bldc_adc_trigger_init(TIM_TypeDef *TIMx, uint32_t timer_hz, uint32_t system_hz, uint32_t max_counter, uint32_t pwm_mode);
void bldc_adc_trigger_start(TIM_TypeDef *TIMx);
void bldc_adc_trigger_stop(TIM_TypeDef *TIMx);


#ifdef __cplusplus
}
#endif

#endif
