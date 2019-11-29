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
#include "uartrpcserver.h"

extern UartRpcServer rpc_server;
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

	rpc_server.add("servo[0].start", rexjson::make_rpc_wrapper(this, &ServoDrive::Start, "void ServoDrive::Start()"));
	rpc_server.add("servo[0].stop", rexjson::make_rpc_wrapper(this, &ServoDrive::Stop, "void ServoDrive::Stop()"));
	rpc_server.add("servo[0].measure_resistance", rexjson::make_rpc_wrapper(this, &ServoDrive::RunResistanceMeasurement, "float ServoDrive::RunResistanceMeasurement()"));

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
	period_ = GetPwmGenerator()->GetPeriod();
//	GetPwmGenerator()->Start();
	RunRotateTasks();
}

void ServoDrive::Stop()
{
	GetPwmGenerator()->Stop();
	sched.Abort();
	osDelay(20);

}

bool ServoDrive::IsStarted()
{
	return GetPwmGenerator()->IsStarted();
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
	float Ia = CalculatePhaseCurrent(data_.injdata_[0], lpf_bias_a.Output());
	float Ib = CalculatePhaseCurrent(data_.injdata_[1], lpf_bias_b.Output());
	float Ic = CalculatePhaseCurrent(data_.injdata_[2], lpf_bias_b.Output());
//	float Ic = -(Ia + Ib);
	lpf_Iab_.DoFilter(Pa_ * Ia + Pb_ * Ib + Pc_ * Ic);
}

float ServoDrive::CalculatePhaseCurrent(float adc_val, float adc_bias)
{
	return ((adc_bias - adc_val) * Vref_ / adc_full_scale) / Rsense_ / csa_gain_;
}

void ServoDrive::SignalThreadUpdate()
{
	data_.injdata_[0] = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_1);
	data_.injdata_[1] = LL_ADC_INJ_ReadConversionData12(adc2.hadc_->Instance, LL_ADC_INJ_RANK_1);
	data_.injdata_[2] = LL_ADC_INJ_ReadConversionData12(adc3.hadc_->Instance, LL_ADC_INJ_RANK_1);
	data_.vbus_ = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_4);
	data_.theta_ = encoder_->GetElectricPosition();
	data_.counter_dir_ = LL_TIM_GetDirection(htim1.Instance);
	data_.update_counter_++;
	sched.SignalThreadUpdate();
}

bool ServoDrive::RunUpdateHandler(const std::function<bool(void)>& update_handler)
{
	uint32_t flags = sched.WaitSignals(Scheduler::THREAD_FLAG_ABORT | Scheduler::THREAD_FLAG_UPDATE, 3);
	if (flags == Scheduler::THREAD_FLAG_UPDATE) {
		if (data_.counter_dir_)
			UpdateCurrentBias();
		else
			UpdateCurrent();
		UpdateRotor();
		UpdateVbus();

		return update_handler();
	}

	/*
	 * If We got here the ABORT signal was received
	 * or the UPDATE didn't come
	 */
	GetPwmGenerator()->Stop();
	sched.Abort();
	return false;
}

float ServoDrive::VoltageToDuty(float v_bus, float v_abs)
{
	float v_rms = v_bus * 0.7071f;
	float duty = v_abs / v_rms;
	float mod = std::min(duty, 0.2f);
	return mod;
}

bool ServoDrive::GetTimings(const std::complex<float>& vec, uint32_t timing_period, uint32_t& timing_a, uint32_t& timing_b, uint32_t& timing_c)
{
	uint32_t half_pwm = timing_period / 2;

	if (std::abs(vec) >= 1.0f)
		return false;
	timing_a = half_pwm + half_pwm * ((vec.real()*Pa_.real() + vec.imag()*Pa_.imag()));
	timing_b = half_pwm + half_pwm * ((vec.real()*Pb_.real() + vec.imag()*Pb_.imag()));
	timing_c = half_pwm + half_pwm * ((vec.real()*Pc_.real() + vec.imag()*Pc_.imag()));
	return true;
}

void ServoDrive::RunRotateTasks()
{
	sched.AddTask([&](){
		float reset_voltage = 0.46; // Volts
		uint32_t i = 0;
		bool ret = false;
		pwm_->Start();
		do {
			/*
			 * Reset the rotor for 1 Second
			 */

			ret = RunUpdateHandler([&]()->bool {
				uint32_t timings[3];
				GetTimings(std::polar<float>(1.0f, 0.0f) * VoltageToDuty(lpf_vbus_.Output(),reset_voltage), period_, timings[0], timings[1], timings[2]);
				pwm_->SetTimings(timings, sizeof(timings)/sizeof(timings[0]));
				encoder_->ResetCounter(0);
				return true;
			});

		} while (ret && i++ < update_hz_ );
	});

	sched.AddTask([&](){
		float rot_voltage = 0.46; // Volts
		bool ret = false;
		uint32_t display_counter = 0;
		data_.update_counter_ = 0;
		do {
			ret = RunUpdateHandler([&]()->bool {
				std::complex<float> rotor = lpf_e_rotor_.Output();
				theta_cur_ = std::arg(rotor);
				if (theta_cur_ < 0)
					theta_cur_ += 2.0 * M_PI;
				uint32_t timings[3];
				GetTimings(rotor * std::polar<float>(1.0f, ri_angle_) * VoltageToDuty(lpf_vbus_.Output(), rot_voltage), period_, timings[0], timings[1], timings[2]);
				pwm_->SetTimings(timings, sizeof(timings)/sizeof(timings[0]));


				std::complex<float> I = lpf_Iab_.Output();
				float Iarg = std::arg(I);
				float Iabs = lpf_Iabs_.DoFilter(std::abs(I));
				std::complex<float> Inorm = std::polar<float>(1.0f, Iarg);
				float Rarg = std::arg(rotor);
				lpf_RIdot_.DoFilter(Dot(rotor, Inorm));
				float diff = (Cross(rotor, Inorm) < 0) ? -Acos(lpf_RIdot_.Output()) : Acos(lpf_RIdot_.Output());

				if (!data_.counter_dir_ && (display_counter++ % 13) == 0) {
					if (Rarg < 0.0f)
						Rarg += M_PI * 2.0f;
					if (Iarg < 0.0f)
						Iarg += M_PI * 2.0f;
					float speed = asinf(lpf_speed_.Output()) * update_hz_;
					fprintf(stderr, "Vbus: %4.2f, SP: %6.1f, arg(R): %6.1f, arg(I): %6.1f, abs(I): %6.3f, DIFF: %5.1f (%+4.2f), upd: %8lu, dis: %8lu\n",
							lpf_vbus_.Output(), speed, Rarg / M_PI * 180.0f, Iarg / M_PI * 180.0f, Iabs,
							diff / M_PI * 180.0f, lpf_RIdot_.Output(), data_.update_counter_, display_counter);
				}
				return true;
			});
		} while (ret);
	});

	sched.Run();
}

float ServoDrive::RunResistanceMeasurement()
{
	LowPassFilter<std::complex<float>, float> lpf_Imes(0.01);
	float rot_voltage = 0.46; // Volts
	uint32_t test_period = 1; // Seconds

	sched.AddTask([&](){
		uint32_t i = 0;
		bool ret = false;
		period_ = GetPwmGenerator()->GetPeriod();
		pwm_->Start();
		do {
			/*
			 * Run for 1 Second
			 */

			ret = RunUpdateHandler([&]()->bool {
				uint32_t timings[3];
				GetTimings(std::polar<float>(1.0f, 0.0f) * VoltageToDuty(lpf_vbus_.Output(),rot_voltage), period_, timings[0], timings[1], timings[2]);
				pwm_->SetTimings(timings, sizeof(timings)/sizeof(timings[0]));
				lpf_Imes.DoFilter(lpf_Iab_.Output());
				return true;
			});

		} while (ret && i++ < update_hz_ * test_period);
		pwm_->Stop();
	});

	sched.RunWaitForCompletion();
	return std::abs(lpf_Imes.Output());
}

void ServoDrive::RunSimpleTasks()
{
	sched.AddTask([&](){
		uint32_t t0 = xTaskGetTickCount();
		if (sched.WaitSignals(Scheduler::THREAD_FLAG_ABORT | Scheduler::THREAD_FLAG_UPDATE, 2000) == Scheduler::THREAD_FLAG_ABORT) {
			fprintf(stderr, "Task1 Aborting...\n\n\n");
			return;
		}
		fprintf(stderr, "Task1 finished %lu\n", xTaskGetTickCount() - t0);
	});
	sched.AddTask([&](){
		uint32_t t0 = xTaskGetTickCount();
		if (sched.WaitSignalAbort(2000)) {
			fprintf(stderr, "Task2 Aborting...\n\n\n");
			return;
		}
		fprintf(stderr, "Task2 finished %lu\n", xTaskGetTickCount() - t0);
	});
	sched.AddTask([&](){
		uint32_t t0 = xTaskGetTickCount();
		if (sched.WaitSignalAbort(2000)) {
			fprintf(stderr, "Task3 Aborting...\n\n\n");
			return;
		}
		fprintf(stderr, "Task3 finished %lu\n\n\n", xTaskGetTickCount() - t0);
		runtasks = 0;
	});
	sched.Run();

}
