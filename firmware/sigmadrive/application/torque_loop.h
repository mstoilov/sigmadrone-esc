/*
 * torque_loop.h
 *
 *  Created on: Oct 13, 2019
 *      Author: mstoilov
 */

#ifndef _TORQUELOOP_H_
#define _TORQUELOOP_H_

#include <stdint.h>
#include <stddef.h>
#include <complex>
#include "pwm_generator.h"

class PwmGenerator;

class TorqueLoop {
public:
	TorqueLoop();
	~TorqueLoop();

	void SetThrottle(float throttle)	{ throttle_ = std::min(std::max(0.0f, throttle), 1.0f); }
	float GetThrottle() const			{ return throttle_; }
	void GetTimings(float throttle, const std::complex<float>& vec, uint32_t period, uint32_t *timings, size_t timings_size);

protected:
	float throttle_;
	std::complex<float> p1_;
	std::complex<float> p2_;
	std::complex<float> p3_;
};


#endif /* _TORQUELOOP_H_ */
