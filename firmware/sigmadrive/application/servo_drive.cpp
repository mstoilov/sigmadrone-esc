/*
 * servo_drive.cpp
 *
 *  Created on: Oct 15, 2019
 *      Author: mstoilov
 */

#include "servo_drive.h"
#include "quadrature_encoder.h"

ServoDrive::ServoDrive(IEncoder* encoder, IPwmGenerator *pwm)
{
	encoder_ = encoder;
	pwm_ = pwm;
	props_= RpcPropertyMap({
		{"torque_loop", tql_.props_},
		{"encoder", encoder->GetProperties()},
	});

}

ServoDrive::~ServoDrive()
{
	// TODO Auto-generated destructor stub
}

void ServoDrive::Start()
{
	update_counter_ = 0;
	period_ = GetPwmGenerator()->GetPeriod();
	GetPwmGenerator()->Start();
}

void ServoDrive::Stop()
{
	GetPwmGenerator()->Stop();
}

bool ServoDrive::IsStarted()
{
	return GetPwmGenerator()->IsStarted();
}

void ServoDrive::PeriodElapsedCallback()
{
	float throttle = 0.05;

	if (update_counter_ < (SystemCoreClock / period_) / 4) {
		/*
		 * Reset the rotor for 1/4 Second
		 */

		tql_.GetTimings(throttle, std::polar<float>(1.0, 0.0), period_, timings_, sizeof(timings_)/sizeof(timings_[0]));
		GetPwmGenerator()->SetTimings(timings_, sizeof(timings_)/sizeof(timings_[0]));
		GetEncoder()->ResetCounter(0);
	} else {
		float rot = GetEncoder()->GetElectricPosition();
		std::complex<float> rotor = std::polar(1.0f, rot);
		rotor = rotor * std::complex<float>(0.0f, 1.0f);
		tql_.GetTimings(throttle, rotor, period_, timings_, sizeof(timings_)/sizeof(timings_[0]));
		GetPwmGenerator()->SetTimings(timings_, sizeof(timings_)/sizeof(timings_[0]));
	}
	update_counter_++;

}
