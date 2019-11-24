/*
 * servo_drive.cpp
 *
 *  Created on: Oct 15, 2019
 *      Author: mstoilov
 */

#include "main.h"
#include "servo_drive.h"
#include "quadrature_encoder.h"
#include "adc.h"
#include "drv8323.h"
#include "ClString.h"

extern TIM_HandleTypeDef htim1;
extern Adc adc1;
extern Adc adc2;
extern Adc adc3;
extern Drv8323 drv1;
static std::complex<float> p1_ = std::polar<float>(1.0f, 0.0f);
static std::complex<float> p2_ = std::polar<float>(1.0f, M_PI * 2.0f / 3.0f);
static std::complex<float> p3_ = std::polar<float>(1.0f, M_PI * 4.0f / 3.0f);

ServoDrive::ServoDrive(IEncoder* encoder, IPwmGenerator *pwm, uint32_t update_hz)
	: Pa_(std::polar<float>(1.0f, 0.0f))
	, Pb_(std::polar<float>(1.0f, M_PI * 2.0 / 3.0))
	, Pc_(std::polar<float>(1.0f, M_PI * 4.0 / 3.0))
	, update_hz_(update_hz)
	, lpf_bias_a(bias_alpha_)
	, lpf_bias_b(bias_alpha_)
	, lpf_bias_c(bias_alpha_)
	, lpf_e_rotor_(rotor_alpha_)
	, lpf_Iab_(i_alpha_)
	, lpf_Idq_(i_alpha_)
	, lpf_Iabs_(iabs_alpha_)
	, lpf_RIdot_(ridot_alpha_)
	, lpf_vbus_(0.02f, 15.0f)
	, lpf_speed_(speed_alpha_)
{
	lpf_bias_a.Reset(1 << 11);
	lpf_bias_b.Reset(1 << 11);
	lpf_bias_c.Reset(1 << 11);
	encoder_ = encoder;
	pwm_ = pwm;
	props_= rexjson::property_map({
		{"speed", rexjson::property(&lpf_speed_.out_, rexjson::property_access::readonly)},
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
					lpf_Iab_.SetAlpha(i_alpha_);
					lpf_Idq_.SetAlpha(i_alpha_);
				})},
		{"iabs_alpha", rexjson::property(
				&iabs_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_Iabs_.SetAlpha(iabs_alpha_);
				})},
		{"rotor_alpha", rexjson::property(
				&rotor_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_e_rotor_.SetAlpha(rotor_alpha_);
				})},
		{"speed_alpha", rexjson::property(
				&speed_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_speed_.SetAlpha(speed_alpha_);
				})},
		{"ridot_alpha", rexjson::property(
				&ridot_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_RIdot_.SetAlpha(ridot_alpha_);
		})},
		{"csa_gain", rexjson::property(
				&csa_gain_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){int t = v.get_int(); if (t != 5 && t != 10 && t != 20 && t != 40) throw std::range_error("Invalid value");},
				[&](void*){drv1.SetCSAGainValue(csa_gain_);}
		)},
		{"runtasks", rexjson::property(
				&runtasks,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*){if (runtasks) RunSimpleTasks(); else sched.Abort();}
		)},


		{"ri_angle", &ri_angle_},
		{"torque_loop", tql_.props_},
		{"encoder", encoder->GetProperties()},
	});

}

ServoDrive::~ServoDrive()
{
	// TODO Auto-generated destructor stub
}

void ServoDrive::Attach()
{
	csa_gain_ = drv1.GetCSAGainValue();

	sched.StartDispatcherThread();
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
	lpf_speed_.DoFilter(position_delta * update_hz_);
	theta_old_ = theta_cur_;
}

template<typename T>
T Cross(const std::complex<T>& a, const std::complex<T>& b)
{
	return a.real() * b.imag() - a.imag() * b.real();
}

template<typename T>
T Dot(const std::complex<T>& a, const std::complex<T>& b)
{
	return a.real() * b.real() + a.imag() * b.imag();
}

float Acos(float x)
{
	if (x > 1.0f)
		return 0;
	if (x < -1.0f)
		return M_PI;
	return acos(x);
}

float Asin(float x)
{
	if (x > 1.0f)
		return M_PI_2;
	if (x < -1.0f)
		return -M_PI_2;
	return asin(x);
}


void ServoDrive::UpdateRotor()
{
	std::complex<float> r_prev = lpf_e_rotor_.Output();
	std::complex<float> r_cur = lpf_e_rotor_.DoFilter(std::polar(1.0f, data_.theta_));
//	float delta = Cross(r_prev, r_cur) < 0 ? -Acos(Dot(r_prev, r_cur)) : Acos(Dot(r_prev, r_cur));
	lpf_speed_.DoFilter(Cross(r_prev, r_cur));
}

void ServoDrive::UpdateVbus()
{
	lpf_vbus_.DoFilter(__LL_ADC_CALC_DATA_TO_VOLTAGE(Vref_, data_.vbus_, LL_ADC_RESOLUTION_12B) * Vbus_resistor_ratio_);
}

void ServoDrive::UpdateCurrentBias()
{
	lpf_bias_a.DoFilter(data_.injdata_[0]);
	lpf_bias_b.DoFilter(data_.injdata_[1]);
	lpf_bias_c.DoFilter(data_.injdata_[2]);
}


void ServoDrive::UpdateCurrent()
{
	float Ia = PhaseCurrent(data_.injdata_[0], lpf_bias_a.Output());
	float Ib = PhaseCurrent(data_.injdata_[1], lpf_bias_b.Output());
	float Ic = -(Ia + Ib);
	lpf_Iab_.DoFilter(Pa_ * Ia + Pb_ * Ib + Pc_ * Ic);
}

float ServoDrive::PhaseCurrent(float adc_val, float adc_bias)
{
	return ((adc_bias - adc_val) * Vref_ / adc_full_scale) / Rsense_ / csa_gain_;
}

void ServoDrive::PeriodElapsedCallback()
{
	SignalThreadUpdate();
}

void ServoDrive::UpdateHandlerNoFb()
{
	if (data_.counter_dir_)
		UpdateCurrentBias();
	else
		UpdateCurrent();
	UpdateRotor();
	UpdateVbus();
//	UpdateSpeed();

	if (update_counter_ < update_hz_ ) {
		/*
		 * Reset the rotor for 1/4 Second
		 */

		GetTimings(std::polar<float>(1.0, 0.0));
		pwm_->SetTimings(timings_, sizeof(timings_)/sizeof(timings_[0]));
		encoder_->ResetCounter(0);
	} else {
		std::complex<float> rotor = lpf_e_rotor_.Output();
		theta_cur_ = std::arg(rotor);
		if (theta_cur_ < 0)
			theta_cur_ += 2.0 * M_PI;
		GetTimings(rotor * std::polar<float>(1.0f, ri_angle_));
		pwm_->SetTimings(timings_, sizeof(timings_)/sizeof(timings_[0]));


		std::complex<float> I = lpf_Iab_.Output();
		float Iarg = std::arg(I);
		float Iabs = lpf_Iabs_.DoFilter(std::abs(I));
		std::complex<float> Inorm = std::polar<float>(1.0f, Iarg);
		float Rarg = std::arg(rotor);
		lpf_RIdot_.DoFilter(Dot(rotor, Inorm));
		float diff = (Cross(rotor, Inorm) < 0) ? -Acos(lpf_RIdot_.Output()) : Acos(lpf_RIdot_.Output());

		if (!data_.counter_dir_ && (update_counter_ % 13) == 0) {
			if (Rarg < 0.0f)
				Rarg += M_PI * 2.0f;
			if (Iarg < 0.0f)
				Iarg += M_PI * 2.0f;
			float speed = asinf(lpf_speed_.Output()) * update_hz_;
//			fprintf(stderr, "Vbus: %4.2f, SP: %6.1f, arg(R): %6.1f, arg(I): %6.1f, abs(I): %6.3f, DIFF: %5.1f (%+4.2f)\n",
//					lpf_vbus_.Output(), speed, Rarg / M_PI * 180.0f, Iarg / M_PI * 180.0f, Iabs,
//					diff / M_PI * 180.0f, lpf_RIdot_.Output());
		}

#if 0
		if (!data_.counter_dir_ && (update_counter_ % 13) == 0) {
			if (Rarg < 0.0f)
				Rarg += M_PI * 2.0f;
			if (Iarg < 0.0f)
				Iarg += M_PI * 2.0f;

			fprintf(stderr, "dir: %2lu,  VBUS: %5lu, INJDATA_: (%5lu, %5lu, %5lu)"
#if LOCAL_INJDATA
					" INJDATA: (%5lu, %5lu, %5lu)"
#endif
					" BIAS: (%5.2f, %5.2f, %5.2f) I: %6.3f, %6.3f, %6.3f (%6.3f)"
					" R: %5.2f, DIFF: %5.2f\n",
					data_.counter_dir_, data_.vbus_,
					(uint32_t) data_.injdata_[0], (uint32_t) data_.injdata_[1], (uint32_t) data_.injdata_[2],

#if LOCAL_INJDATA
					(uint32_t) injdata[0], (uint32_t) injdata[1], (uint32_t) injdata[2],
#endif
					lpf_bias_a.Output(), lpf_bias_b.Output(), lpf_bias_c.Output(),
					lpf_i_a.Output(), lpf_i_b.Output(), lpf_i_c.Output(), lpf_i_a.Output() + lpf_i_b.Output() + lpf_i_c.Output(),
					Rarg / M_PI * 180.0f, diff / M_PI * 180.0f
			);


		}
#endif

	}
}

void ServoDrive::UpdateHandler_wip()
{
#if 0
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
#endif
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

static void RunControlLoopWrapper(const void* ctx)
{
	*_impure_ptr = *_impure_data_ptr;
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
	osThreadDef(RunControlLoopWrapper, osPriorityHigh, 0, 16000);
	control_thread_id_ = osThreadCreate(&os_thread_def_RunControlLoopWrapper, this);
}


void ServoDrive::GetTimings(const std::complex<float>& vec)
{
	uint32_t half_pwm = period_ / 2;
	uint32_t throttle_duty = half_pwm * throttle_;

	timings_[0] = half_pwm + throttle_duty * ((vec.real()*Pa_.real() + vec.imag()*Pa_.imag()));
	timings_[1] = half_pwm + throttle_duty * ((vec.real()*Pb_.real() + vec.imag()*Pb_.imag()));
	timings_[2] = half_pwm + throttle_duty * ((vec.real()*Pc_.real() + vec.imag()*Pc_.imag()));
}

void ServoDrive::RunSimpleTasks()
{
	sched.AddTask([&](){
		if (sched.WaitAbort(3000)) {
			fprintf(stderr, "Task1 Aborting...\n");
			return;
		}
		fprintf(stderr, "Task1 finished\n");
	});
	sched.AddTask([&](){
		if (sched.WaitAbort(3000)) {
			fprintf(stderr, "Task2 Aborting...\n");
			return;
		}
		fprintf(stderr, "Task2 finished\n");
	});
	sched.AddTask([&](){
		runtasks = 0;
		if (sched.WaitAbort(3000)) {
			fprintf(stderr, "Task3 Aborting...\n");
			return;
		}
		fprintf(stderr, "Task3 finished\n");
	});
	sched.Run();

}
