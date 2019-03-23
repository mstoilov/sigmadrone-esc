/*
 * PWMDecoder.h
 *
 *  Created on: May 4, 2017
 *      Author: mstoilov
 */

#ifndef PWMDECODER_H_
#define PWMDECODER_H_

#include <stdint.h>
#include <vector>
#include <functional>
#include "timer.h"
#include "gpiopin.h"


class PWMDecoder: public Timer {
public:
	using base = Timer;

	PWMDecoder(
			const std::vector<GPIOPin>& output_pins,
			TIM_TypeDef *TIMx,
			const TimeSpan& pwm_period,
			const Frequency& system_clock = Frequency::from_hertz(SystemCoreClock),
			uint32_t irq_priority = 0
	);
	virtual ~PWMDecoder();
	virtual void HandleCC1() override;
	virtual void Start() override;
	virtual void Stop() override;
	uint32_t GetPWMPulse();
	uint32_t GetPWMPeriod();
	TimeSpan GetPulseLength();

	/** Attach a member function to call when a rising edge occurs on the input
	 *
	 *  @param tptr pointer to the object to call the member function on
	 *  @param mptr pointer to the member function to be called
	 */
	template<typename T>
	void Callback_PWMCC(T* tptr, void (T::*mptr)(uint32_t, uint32_t))
	{
		callback_PWMCC_ = [=](uint32_t pulse, uint32_t period){(tptr->*mptr)(pulse, period);};
	}

	void Callback_PWMCC(const std::function<void(uint32_t, uint32_t)>& callback)
	{
		callback_PWMCC_ = callback;
	}

protected:
	uint32_t pwm_pulse_;
	uint32_t pwm_period_;
	std::function<void(uint32_t, uint32_t)> callback_PWMCC_ = [](uint32_t, uint32_t){};
};


#endif /* PWMDECODER_H_ */
