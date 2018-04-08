#ifndef _BLDC_6STEP_H_
#define _BLDC_6STEP_H_

#include "bldc-include.h"
#include "bldc-timer.h"


#ifdef __cplusplus
extern "C" {
#endif

void bldc_6step_init(TIM_TypeDef *TIMx, uint32_t timer_hz, uint32_t system_hz, uint32_t max_counter,
		uint32_t pwm_mode, uint32_t polarity, uint32_t npolarity, uint32_t deadtime_ns, uint32_t repetions);
void bldc_6step_start(TIM_TypeDef *TIMx);
void bldc_6step_stop(TIM_TypeDef *TIMx);
void bldc_6step_toggle(TIM_TypeDef *TIMx);
void bldc_6step_next(TIM_TypeDef *TIMx, uint32_t state);
uint32_t bldc_6step_switching_frequency(TIM_TypeDef *TIMx, uint32_t system_clock);
void bldc_6step_set_throttle(TIM_TypeDef *TIMx, uint32_t percent /* 0 .. 100 */ );
void bldc_6step_updown_throttle(TIM_TypeDef *TIMx, uint32_t percent /* 0 .. 100 */ );

#ifdef __cplusplus
}
#endif

#endif
