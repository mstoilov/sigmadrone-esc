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
#include "functionptr.h"


class PWMDecoder: public Timer {
public:
	PWMDecoder();
	using base = Timer;

	PWMDecoder(
			TIM_TypeDef *TIMx,
			const TimeSpan& pwm_period,
			const Frequency& system_clock = Frequency::from_hertz(SystemCoreClock),
			const std::vector<GPIOPin>& output_pins = {}
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
	void callback(T* tptr, void (T::*mptr)(void))
	{
		if (tptr && mptr)
			callback_ = [=](void){(tptr->*mptr)();};
	}

	void callback(const std::function<void(void)>& callback)
	{
		callback_ = callback;
	}

protected:
	uint32_t pwm_pulse_;
	uint32_t pwm_period_;
	std::function<void(void)> callback_ = [](){};
};


#endif /* PWMDECODER_H_ */
