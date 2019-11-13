/*
 * servo_drive.h
 *
 *  Created on: Oct 15, 2019
 *      Author: mstoilov
 */

#ifndef _SERVO_DRIVE_H_
#define _SERVO_DRIVE_H_

#include "iservodrive.h"
#include "lowpassfilter.h"
#include "property.h"
#include "torque_loop.h"
#include "pidcontroller.h"


class ServoDrive : public IServoDrive {
public:
	ServoDrive(IEncoder* encoder, IPwmGenerator *pwm);
	virtual ~ServoDrive();
	virtual IEncoder* GetEncoder() const override { return encoder_; }
	virtual IPwmGenerator* GetPwmGenerator() const override { return pwm_; }
	virtual void Start() override;
	virtual void Stop() override;
	virtual bool IsStarted() override;

	void SetCallbackFrequency(uint32_t callback_hz);
	void PeriodElapsedCallback();
	void UpdateSpeed();

public:
	rexjson::property props_;

public:
	uint32_t callback_hz_;
	float csa_gain_ = 10.0f;
	float Rsense_ = 0.010f;
	float position_temp_ = 0.0;
	float theta_old_ = 0;
	float theta_cur_ = 0;
	float speed_ = 0.0;		// counts per time slice
	LowPassFilter<decltype(speed_), float> lpf_speed_;
	LowPassFilter<std::complex<float>, float> lpf_R;
	LowPassFilter<std::complex<float>, float> lpf_I;
	LowPassFilter<std::complex<float>, float> lpf_Ipwm;
	float throttle_ = 0.05;
	uint32_t update_counter_ = 0;
	uint32_t period_ = 0;
	TorqueLoop tql_;
	uint32_t timings_[3];
	IEncoder *encoder_ = nullptr;
	IPwmGenerator *pwm_ = nullptr;

};

#endif /* _SERVO_DRIVE_H_ */
