/*
 * servo_drive.cpp
 *
 *  Created on: Oct 15, 2019
 *      Author: mstoilov
 */

#include "servo_drive.h"
#include "quadrature_encoder.h"
#include "adc.h"
#include "drv8323.h"

extern TIM_HandleTypeDef htim1;
extern Adc adc1;
extern Adc adc2;
extern Adc adc3;
extern Drv8323 drv1;

ServoDrive::ServoDrive(IEncoder* encoder, IPwmGenerator *pwm, uint32_t update_hz)
	: update_hz_(update_hz)
	, lpf_speed_(0.00035f)
	, lpf_R(0.5)
	, lpf_I(0.6)
	, lpf_Ipwm(0.8)
	, lpf_bias_a(bias_alpha_)
	, lpf_bias_b(bias_alpha_)
	, lpf_bias_c(bias_alpha_)
	, lpf_i_a(i_alpha_)
	, lpf_i_b(i_alpha_)
	, lpf_i_c(i_alpha_)
	, lpf_i_abs(1.0/update_hz, 0.1)
{
	lpf_bias_a.Reset(1 << 11);
	lpf_bias_b.Reset(1 << 11);
	lpf_bias_c.Reset(1 << 11);
	encoder_ = encoder;
	pwm_ = pwm;
	props_= rexjson::property_map({
		{"speed", rexjson::property(&speed_, rexjson::property_access::readonly)},
		{"throttle", rexjson::property(
				&throttle_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 0.25) throw std::range_error("Invalid value");}) },
		{"bias_alpha", rexjson::property(
				&bias_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_bias_a.SetAlpha(bias_alpha_);
					lpf_bias_b.SetAlpha(bias_alpha_);
					lpf_bias_c.SetAlpha(bias_alpha_);
				})},
		{"i_alpha", rexjson::property(
				&i_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_i_a.SetAlpha(i_alpha_);
					lpf_i_b.SetAlpha(i_alpha_);
					lpf_i_c.SetAlpha(i_alpha_);
				})},
		{"ri_angle", &ri_angle},
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
	osDelay(20);
	update_counter_ = 0;
	period_ = GetPwmGenerator()->GetPeriod();
	GetPwmGenerator()->Start();
}

void ServoDrive::Stop()
{
	GetPwmGenerator()->Stop();
	osDelay(20);

}

bool ServoDrive::IsStarted()
{
	return GetPwmGenerator()->IsStarted();
}

void ServoDrive::UpdateSpeed()
{
/*
	const float inc = -0.005f;
	position_temp_ += inc;
	if (position_temp_ > M_PI * 2.0)
		position_temp_ -= M_PI * 2.0;
	if (position_temp_ < -M_PI * 2.0)
		position_temp_ += M_PI * 2.0;
*/
	float position_delta = theta_cur_ - theta_old_;
	if (position_delta < -M_PI_2)
		position_delta += M_PI * 2.0;
	else if (position_delta > M_PI_2)
		position_delta -= M_PI * 2.0;
	lpf_speed_.DoFilter(position_delta * 1000.0);
	speed_ = lpf_speed_.Output() * update_hz_;
	theta_old_ = theta_cur_;
}

void ServoDrive::PeriodElapsedCallback()
{
	SignalThreadUpdate();
}

void ServoDrive::UpdateHandlerNoFb()
{

	if (update_counter_ < update_hz_ ) {
		/*
		 * Reset the rotor for 1/4 Second
		 */

		GetTimings(std::polar<float>(1.0, 0.0));
		pwm_->SetTimings(timings_, sizeof(timings_)/sizeof(timings_[0]));
		encoder_->ResetCounter(0);
	} else {
		std::complex<float> rotor = std::polar(1.0f, data_.theta_);
		theta_cur_ = std::arg(rotor);
		if (theta_cur_ < 0)
			theta_cur_ += 2.0 * M_PI;
		GetTimings(rotor * std::polar<float>(1.0f, ri_angle));
		pwm_->SetTimings(timings_, sizeof(timings_)/sizeof(timings_[0]));
		UpdateSpeed();

#if LOCAL_INJDATA
		int32_t injdata[] = {
			   LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_1),
			   LL_ADC_INJ_ReadConversionData12(adc2.hadc_->Instance, LL_ADC_INJ_RANK_1),
			   LL_ADC_INJ_ReadConversionData12(adc3.hadc_->Instance, LL_ADC_INJ_RANK_1),
		};
#endif

		if (data_.counter_dir_) {
			lpf_bias_a.DoFilter(data_.injdata_[0]);
			lpf_bias_b.DoFilter(data_.injdata_[1]);
			lpf_bias_c.DoFilter(data_.injdata_[2]);
		} else {
//			lpf_i_a.DoFilter(-(float)(((float)lpf_bias_a.Output() - (float)data_.injdata_[0]) / 1000.0f / (csa_gain_ * Rsense_)));
//			lpf_i_b.DoFilter(-(float)(((float)lpf_bias_b.Output() - (float)data_.injdata_[1]) / 1000.0f / (csa_gain_ * Rsense_)));
//			lpf_i_c.DoFilter(-(float)(((float)lpf_bias_c.Output() - (float)data_.injdata_[2]) / 1000.0f / (csa_gain_ * Rsense_)));

			lpf_i_a.DoFilter((float)(((float)lpf_bias_a.Output() - (float)data_.injdata_[0]) / 1000.0f / (csa_gain_ * Rsense_)));
			lpf_i_b.DoFilter((float)(((float)lpf_bias_b.Output() - (float)data_.injdata_[1]) / 1000.0f / (csa_gain_ * Rsense_)));
			lpf_i_c.DoFilter((float)(((float)lpf_bias_c.Output() - (float)data_.injdata_[2]) / 1000.0f / (csa_gain_ * Rsense_)));

		}

		float Ia = lpf_i_a.Output();
		float Ib = lpf_i_b.Output();
		float Ic = -(Ia + Ib);

		float Ialpha = 3.0f/2.0f * Ia;
		float Ibeta = 0.866025404f * (Ib - Ic);

		std::complex<float> I = std::complex<float>(Ialpha, Ibeta);
		float diff = acos((I.real() * rotor.real() + I.imag() * rotor.imag())/(std::abs(I) * std::abs(rotor)));
		float argR = std::arg(rotor);
		float argI = std::arg(I);

		if (argR < 0.0f)
			argR += M_PI * 2.0f;
		if (argI < 0.0f)
			argI += M_PI * 2.0f;


		if (!data_.counter_dir_ && (update_counter_ % 13) == 0) {
			fprintf(stderr, "SP: %5.1f mRad/s, arg(R): %6.1f, arg(I): %6.1f, abs(I): %4.1f, DIFF: %6.1f\n",
					lpf_speed_.Output(), argR / M_PI * 180.0f, argI / M_PI * 180.0f, lpf_i_abs.DoFilter(std::abs(I)), diff / M_PI * 180.0f);
		}

#if 0
		if (!data_.counter_dir_ && (update_counter_ % 13) == 0) {
			fprintf(stderr, "dir: %2lu,  VBUS: %5lu, INJDATA_: (%5lu, %5lu, %5lu)"
#if LOCAL_INJDATA
					" INJDATA: (%5lu, %5lu, %5lu)"
#endif
					" BIAS: (%5.2f, %5.2f, %5.2f) I: %5.2f, %5.2f, %5.2f (%5.2f)"
					" R: %5.2f, DIFF: %5.2f\n",
					data_.counter_dir_, data_.vbus_,
					(uint32_t) data_.injdata_[0], (uint32_t) data_.injdata_[1], (uint32_t) data_.injdata_[2],

#if LOCAL_INJDATA
					(uint32_t) injdata[0], (uint32_t) injdata[1], (uint32_t) injdata[2],
#endif
					lpf_bias_a.Output(), lpf_bias_b.Output(), lpf_bias_c.Output(),
					lpf_i_a.Output(), lpf_i_b.Output(), lpf_i_c.Output(), lpf_i_a.Output() + lpf_i_b.Output() + lpf_i_c.Output(),
					argR / M_PI * 180.0f, diff / M_PI * 180.0f
			);


		}
#endif

	}
}

void ServoDrive::UpdateHandler_wip()
{
	if (update_counter_ < update_hz_ / 4) {
		/*
		 * Reset the rotor for 1/4 Second
		 */

		tql_.GetTimings(throttle_, std::polar<float>(1.0, 0.0), period_, timings_, sizeof(timings_)/sizeof(timings_[0]));
		pwm_->SetTimings(timings_, sizeof(timings_)/sizeof(timings_[0]));
		encoder_->ResetCounter(0);
		position_temp_ = theta_old_ = encoder_->GetElectricPosition();
	} else {
		std::complex<float> rotor = lpf_R.DoFilter(std::polar(1.0f, GetEncoder()->GetElectricPosition()));
		theta_cur_ = std::arg(rotor);
		if (theta_cur_ < 0)
			theta_cur_ += 2.0 * M_PI;
		tql_.GetTimings(throttle_, rotor * std::complex<float>(0.0f, 1.0f), period_, timings_, sizeof(timings_)/sizeof(timings_[0]));
		pwm_->SetTimings(timings_, sizeof(timings_)/sizeof(timings_[0]));
		UpdateSpeed();

		float Ia = -(float)(((float)adc1.bias_[0] - (float)adc1.injdata_[0]) / 1000.0f / (csa_gain_ * Rsense_));
		float Ib = -(float)(((float)adc1.bias_[2] - (float)adc1.injdata_[1]) / 1000.0f / (csa_gain_ * Rsense_));
		float Ic = -(Ia + Ib);

		float Ialpha = 3.0f/2.0f * Ia;
		float Ibeta = 0.866025404f * (Ib - Ic);

		float cos_theta = cos(theta_cur_);
		float sin_theta = sin(theta_cur_);
		float Id = Ialpha * cos_theta + Ibeta * sin_theta;
		float Iq = -Ialpha * sin_theta + Ibeta * cos_theta;

/*
		Ia = -(timings_[0] - period_ * 0.5f);
		Ib = -(timings_[1] - period_ * 0.5f);
		Ic = -(timings_[2] - period_ * 0.5f);

		Ialpha = 3.0f/2.0f * Ia;
		Ibeta = 0.866025404f * (Ib - Ic);
		std::complex<float> Ipwm = lpf_Ipwm.DoFilter(std::complex<float>(Ialpha, Ibeta));
		Ipwm = Ipwm/std::abs(Ipwm);
*/

		std::complex<float> I = std::complex<float>(Ialpha, Ibeta);
		I = I/std::abs(I);
		float diff = acos((I.real() * rotor.real() + I.imag() * rotor.imag())/(std::abs(I) * std::abs(rotor)));
		if ((update_counter_ % 37) == 0) {
			fprintf(stderr, "%lu : R = %5.2f (%5.2f), speed: %5.2f\n", update_counter_, std::arg(rotor), diff, speed_);
		}

//		if ((update_counter_ % 37) == 0) {
//			fprintf(stderr, "%lu : Id = %5.2f, Iq = %5.2f, speed: %5.2f\n", update_counter_, Id, Iq, speed_);
//		}
	}
	update_counter_++;

}

void ServoDrive::RunControlLoop()
{
	for (;;) {
		if (WaitUpdate()) {
			UpdateHandlerNoFb();
			data_.processing_ = 0;
		}
	}
}

static void RunControlLoopWrapper(void const* ctx)
{
	reinterpret_cast<ServoDrive*>(const_cast<void*>(ctx))->RunControlLoop();
	reinterpret_cast<ServoDrive*>(const_cast<void*>(ctx))->control_thread_id_ = 0;
	osThreadExit();
}


bool ServoDrive::WaitUpdate()
{
	return (osSignalWait(THREAD_SIGNAL_UPDATE, wait_timeout_).status == osEventSignal) ? true : false;
}

void ServoDrive::SignalThreadUpdate()
{
	update_counter_++;
	if (!data_.processing_) {
		data_.injdata_[0] = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_1);
		data_.injdata_[1] = LL_ADC_INJ_ReadConversionData12(adc2.hadc_->Instance, LL_ADC_INJ_RANK_1);
		data_.injdata_[2] = LL_ADC_INJ_ReadConversionData12(adc3.hadc_->Instance, LL_ADC_INJ_RANK_1);
		data_.vbus_ = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_4);
		data_.theta_ = encoder_->GetElectricPosition();
		data_.counter_dir_ = LL_TIM_GetDirection(htim1.Instance);
	}
	data_.processing_++;
	if (control_thread_id_)
		osSignalSet(control_thread_id_, THREAD_SIGNAL_UPDATE);
}

void ServoDrive::StartControlThread()
{
	osThreadDef(RunControlLoopWrapper, osPriorityRealtime, 0, 2048);
	control_thread_id_ = osThreadCreate(&os_thread_def_RunControlLoopWrapper, this);
}


void ServoDrive::GetTimings(const std::complex<float>& vec)
{
	static std::complex<float> p1_ = std::polar<float>(1.0f, 0.0f);
	static std::complex<float> p2_ = std::polar<float>(1.0f, M_PI * 2.0f / 3.0f);
	static std::complex<float> p3_ = std::polar<float>(1.0f, M_PI * 4.0f / 3.0f);

	uint32_t half_pwm = period_ / 2;
	uint32_t throttle_duty = half_pwm * throttle_;

	timings_[0] = half_pwm + throttle_duty * ((vec.real()*p1_.real() + vec.imag()*p1_.imag()));
	timings_[1] = half_pwm + throttle_duty * ((vec.real()*p2_.real() + vec.imag()*p2_.imag()));
	timings_[2] = half_pwm + throttle_duty * ((vec.real()*p3_.real() + vec.imag()*p3_.imag()));
}

