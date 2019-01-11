#ifndef TRIGGER_H_
#define TRIGGER_H_

#include <stdint.h>
#include <vector>
#include <functional>
#include "timer.h"
#include "gpiopin.h"


class Trigger: public Timer {
public:
	using base = Timer;

	Trigger(
			TIM_TypeDef *TIMx,
			const TimeSpan& trigger_period,
			const Frequency& system_clock = Frequency::from_hertz(SystemCoreClock),
			SlaveMode slave_mode = SlaveTrigger,
			TriggerInput trigger_input = TriggerInternal0,
			TriggerOutput trigger_output = TrigOC1REF,
			Channel ch = CH1,
			OCMode pwm_mode = PWM1);
	virtual ~Trigger();
};

#endif
