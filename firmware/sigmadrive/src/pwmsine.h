/*
 *  Sigmadrone
 *  Copyright (c) 2013-2015 The Sigmadrone Developers
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Martin Stoilov <martin@sigmadrone.org>
 *  Svetoslav Vassilev <svassilev@sigmadrone.org>
 */

#ifndef _PWMSINE_H_
#define _PWMSINE_H_

#include <stdint.h>
#include <vector>
#include <complex>
#include <functional>
#include "timer.h"
#include "digitalout.h"

struct RunState {
	RunState(const std::function<bool(void)>& function)
	: rs_function(function)
	{
	}

	std::function<bool(void)> rs_function;
};


//#define PANASONIC_MOTOR
#ifdef PANASONIC_MOTOR
#define MOTOR_POLES_NUMBER 8
#else
#define MOTOR_POLES_NUMBER 14
#endif

class PWMSine : public Timer
{
public:
	using base = Timer;

	static constexpr unsigned int NUMBER_OF_POLES = MOTOR_POLES_NUMBER;
	static constexpr unsigned int M2E_RATIO = NUMBER_OF_POLES / 2;
	static constexpr unsigned int SINE_SAMPLES = 1024;
	static constexpr unsigned int CURRENT_SAMPLES = 3;
	static constexpr unsigned int ADC_GAIN = 10;
	static constexpr unsigned int SHUNT_RESISTOR = 10; /* mA */
	static constexpr float MAX_THROTTLE = 0.35;
	static constexpr float MIN_THROTTLE = 0.00;

	PWMSine(const std::vector<GPIOPin>& output_pins,
			TIM_TypeDef *TIMx,
			const Frequency& switching_freq,
			const Frequency& system_clock = Frequency::from_hertz(SystemCoreClock),
			TriggerOutput trigger_output = TrigUpdate,
			OCMode pwm_mode = PWM1,
			OCPolarity polarity = High,
			OCPolarity npolarity = Low,
			uint32_t deadtime = 30, 		/* ns */
			uint32_t irq_priority = 0);
	virtual ~PWMSine();
	virtual void HandleUpdate() override;
	virtual void Start() override;
	virtual void Stop() override;
	virtual void SetOCValue(Channel ch, uint32_t value) override;
	void HandleCurrentJEOS(int32_t *injdata, size_t idx, size_t size);
	void SineDriving();
	float GetDutyCycle();
	void SetThrottle(float percent);
	void SetElectricalRotationsPerSecond(const Frequency& f);
	Frequency GetSwitchingFrequency();
	void Toggle();
	void SetOpAmpBias(const std::vector<int32_t> bias);
	static float GetEncoderTheta();

public:
	int32_t adc_data_[CURRENT_SAMPLES];
	Frequency switching_freq_;
	uint32_t update_counter_ = 0;
	TimeSpan duty_;
	uint32_t duty_oc_ = 0;
	OCMode pwm_mode_;
	OCPolarity polarity_;
	OCPolarity npolarity_;
	uint32_t SINE_STEPS = 1024;
	std::vector<int32_t> opamp_bias_;
	std::complex<float> p1;
	std::complex<float> p2;
	std::complex<float> p3;
	std::complex<float> r;
	std::complex<float> v;

	uint32_t run_index_ = 0;
	std::vector<std::function<bool(void)>> run_stack_;
};

#endif

