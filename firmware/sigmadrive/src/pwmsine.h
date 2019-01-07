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
#include "timer.h"
#include "digitalout.h"

class PWMSine : public Timer
{
public:
	using base = Timer;

	static constexpr unsigned int SINE_STATES = 1200;

	PWMSine(
			TIM_TypeDef *TIMx,
			const Frequency& switching_freq,
			const Frequency& system_clock = Frequency::from_hertz(SystemCoreClock),
			const std::vector<GPIOPin>& output_pins = {}
	);
	virtual ~PWMSine();
	virtual void HandleUpdate() override;
	virtual void Start() override;
	virtual void Stop() override;
	void SineDriving();
	float GetDutyCycle();
	void SetDutyCycle(float percent);
	void SetDutyPeriod(const TimeSpan& period);
	void SetRotationsPerSecond(const Frequency& f);
	Frequency GetSwitchingFrequency();
	void Toggle();

public:
	Frequency switching_freq_;
	uint32_t state_;
	TimeSpan duty_;
	TimeSpan sine_duty_[SINE_STATES];
	uint32_t counter_ = 0;
};

#endif

