/*
 * torque_loop.cpp
 *
 *  Created on: Oct 13, 2019
 *      Author: mstoilov
 */


#include <math.h>
#include "torque_loop.h"

TorqueLoop::TorqueLoop()
	: throttle_(0.0f)
{
	p1_ = std::polar<float>(1.0f, 0.0f);
	p2_ = std::polar<float>(1.0f, M_PI * 4.0f / 3.0f);
	p3_ = std::polar<float>(1.0f, M_PI * 2.0f / 3.0f);

	props_= RpcPropertyMap({
		{"throttle", RpcProperty(&throttle_)},
//		{"i", RpcProperty(&i, RpcObjectAccess::readwrite
//				[](const rexjson::value& v){},
//				[](void *ctx)->void {std::cout << "i was modified" << std::endl;},
//				nullptr
//				),
//		},
//		{"f", RpcProperty(&f,
//				RpcObjectAccess::readwrite,
//				[](const rexjson::value& v)->void{if (v.get_real() > 10.0f) throw std::range_error("value is too big");},
//				[](void *ctx)->void {std::cout << "f was modified" << std::endl;},
//				nullptr),
//		},
//		{"s", RpcProperty(&s)},
//		{"b", RpcProperty(&b,
//				RpcObjectAccess::readwrite,
//				[](const rexjson::value& v)->void{throw std::runtime_error("Can't set");})},
//		{"e", RpcProperty(&e)},
	});
}

TorqueLoop::~TorqueLoop()
{
	// TODO Auto-generated destructor stub
}

void TorqueLoop::GetTimings(float throttle, const std::complex<float>& vec, uint32_t period, uint32_t *timings, size_t timings_size)
{
	uint32_t half_pwm = period / 2;
	uint32_t throttle_duty = half_pwm * throttle;

	if (timings_size > 0)
		timings[0] = half_pwm + throttle_duty * ((vec.real()*p1_.real() + vec.imag()*p1_.imag()));
	if (timings_size > 1)
		timings[1] = half_pwm + throttle_duty * ((vec.real()*p2_.real() + vec.imag()*p2_.imag()));
	if (timings_size > 2)
		timings[2] = half_pwm + throttle_duty * ((vec.real()*p3_.real() + vec.imag()*p3_.imag()));
}

