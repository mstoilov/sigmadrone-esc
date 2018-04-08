#ifndef _BLDC_TIMER_H_
#define _BLDC_TIMER_H_

#include "bldc-include.h"

#ifdef __cplusplus
extern "C" {
#endif

void bldc_timer_init(TIM_TypeDef *TIMx, uint32_t timer_hz, uint32_t system_hz, uint32_t max_counter);
void bldc_timer_start(TIM_TypeDef *TIMx);
void bldc_timer_stop(TIM_TypeDef *TIMx);

void bldc_timer_init_pwm_decoder(TIM_TypeDef *TIMx, uint32_t timer_hz, uint32_t system_hz, uint32_t max_counter);
void bldc_timer_pwm_decoder_start(TIM_TypeDef *TIMx);
void bldc_timer_pwm_decoder_stop(TIM_TypeDef *TIMx);
uint32_t bldc_timer_get_clock_frequency(TIM_TypeDef *TIMx, uint32_t system_clock);

#ifdef __cplusplus
}
#endif

#endif
