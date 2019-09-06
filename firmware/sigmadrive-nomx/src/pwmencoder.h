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

#ifndef _PWMENCODER_H_
#define _PWMENCODER_H_

#include <stdint.h>
#include <vector>
#include "timer.h"
#include "digitalout.h"

class PWMEncoder : public Timer
{
public:
	using base = Timer;

	PWMEncoder(
			const std::vector<GPIOPin>& output_pins,
			TIM_TypeDef *TIMx,
			const TimeSpan& pwm_period,
			const Frequency& system_clock = Frequency::from_hertz(SystemCoreClock)
	);
	virtual ~PWMEncoder();
	virtual void HandleUpdate() override;
	virtual void Start() override;
	virtual void Stop() override;
	void DumbDriving();
	void SetDutyCycle(float duty);
	void SetDutyPeriod(const TimeSpan& period);
	void Toggle();

protected:
	TimeSpan period_;
	uint32_t state_;
	DigitalOut UL_;
	DigitalOut VL_;
	DigitalOut WL_;
	DigitalOut TG_;
};

#endif

