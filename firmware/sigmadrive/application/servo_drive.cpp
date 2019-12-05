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
	, Pb_(std::polar<float>(1.0f, M_PI / 3.0 * 2.0 ))
	, Pc_(std::polar<float>(1.0f, M_PI / 3.0 * 4.0))
	, update_hz_(update_hz)
	, lpf_bias_a(bias_alpha_)
	, lpf_bias_b(bias_alpha_)
	, lpf_bias_c(bias_alpha_)
	, lpf_a(abc_alpha_)
	, lpf_b(abc_alpha_)
	, lpf_c(abc_alpha_)
	, lpf_e_rotor_(rotor_alpha_)
	, lpf_Iab_(i_alpha_)
	, lpf_Idq_(i_alpha_)
	, lpf_Iabs_(iabs_alpha_)
	, lpf_RIdot_(ridot_alpha_)
	, lpf_vbus_(0.5f, 12.0f)
	, lpf_speed_(speed_alpha_)
{
	lpf_bias_a.Reset(1 << 11);
	lpf_bias_b.Reset(1 << 11);
	lpf_bias_c.Reset(1 << 11);
	encoder_ = encoder;
	pwm_ = pwm;
	props_= rexjson::property_map({
		{"speed", rexjson::property(&lpf_speed_.out_, rexjson::property_access::readonly)},
		{"bias_alpha", rexjson::property(
				&bias_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_bias_a.SetAlpha(bias_alpha_);
					lpf_bias_b.SetAlpha(bias_alpha_);
					lpf_bias_c.SetAlpha(bias_alpha_);
				})},
		{"abc_alpha", rexjson::property(
				&abc_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_a.SetAlpha(abc_alpha_);
					lpf_b.SetAlpha(abc_alpha_);
					lpf_c.SetAlpha(abc_alpha_);
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
		{"run_simple_tasks", rexjson::property(
				&run_simple_tasks_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*){ if (run_simple_tasks_) RunSimpleTasks(); else sched_.Abort(); }
		)},
		{"ri_angle", &ri_angle_},
		{"encoder_dir", &encoder_dir_},
		{"svm_saddle", &svm_saddle_},
		{"reset_voltage", &reset_voltage_},
		{"reset_hz", &reset_hz_},
		{"encoder", encoder->GetProperties()},
	});

	rpc_server.add("servo[0].start", rexjson::make_rpc_wrapper(this, &ServoDrive::Start, "void ServoDrive::Start()"));
	rpc_server.add("servo[0].stop", rexjson::make_rpc_wrapper(this, &ServoDrive::Stop, "void ServoDrive::Stop()"));
	rpc_server.add("servo[0].measure_resistance", rexjson::make_rpc_wrapper(this, &ServoDrive::RunResistanceMeasurement, "float ServoDrive::RunResistanceMeasurement(uint32_t seconds, float test_voltage)"));
	rpc_server.add("servo[0].measure_resistance_od", rexjson::make_rpc_wrapper(this, &ServoDrive::RunResistanceMeasurementOD, "float ServoDrive::RunResistanceMeasurementOD(float seconds, float test_current, float max_voltage);"));
	rpc_server.add("servo[0].measure_inductance_od", rexjson::make_rpc_wrapper(this, &ServoDrive::RunInductanceMeasurementOD, "float ServoDrive::RunInductanceMeasurementOD(int num_cycles, float voltage_low, float voltage_high);"));

	rpc_server.add("servo[0].scheduler_run", rexjson::make_rpc_wrapper(this, &ServoDrive::SchedulerRun, "void SchedulerRun()"));
	rpc_server.add("servo[0].scheduler_abort", rexjson::make_rpc_wrapper(this, &ServoDrive::SchedulerAbort, "void SchedulerAbort()"));
	rpc_server.add("servo[0].add_task_arm_motor", rexjson::make_rpc_wrapper(this, &ServoDrive::AddTaskArmMotor, "void AddTaskArmMotor()"));
	rpc_server.add("servo[0].add_task_disarm_motor", rexjson::make_rpc_wrapper(this, &ServoDrive::AddTaskDisarmMotor, "void AddTaskDisarmMotor()"));
	rpc_server.add("servo[0].add_task_rotate_motor", rexjson::make_rpc_wrapper(this, &ServoDrive::AddTaskRotateMotor, "void AddTaskRotateMotor(float angle, float speed, float voltage, bool dir)"));
	rpc_server.add("servo[0].add_task_reset_rotor", rexjson::make_rpc_wrapper(this, &ServoDrive::AddTaskResetRotor, "void AddTaskResetRotor(float reset_voltage, uint32_t reset_hz)"));
	rpc_server.add("servo[0].alpha_pole_search", rexjson::make_rpc_wrapper(this, &ServoDrive::RunTaskAphaPoleSearch, "void RunTaskAphaPoleSearch()"));

	sched_.SetAbortTask([&]()->void {
		pwm_->Stop();
	});

	sched_.SetIdleTask([&]()->void {
	});
}

ServoDrive::~ServoDrive()
{
	// TODO Auto-generated destructor stub
}

void ServoDrive::Attach()
{
	csa_gain_ = drv1.GetCSAGainValue();

	sched_.StartDispatcherThread();
}

void ServoDrive::Start()
{
	osDelay(20);
	RunSpinTasks();
}

void ServoDrive::Stop()
{
	GetPwmGenerator()->Stop();
	sched_.Abort();
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
	Ia_ = CalculatePhaseCurrent(data_.injdata_[0], lpf_bias_a.Output());
	Ib_ = CalculatePhaseCurrent(data_.injdata_[1], lpf_bias_b.Output());
	Ic_ = CalculatePhaseCurrent(data_.injdata_[2], lpf_bias_b.Output());
	lpf_a.DoFilter(Ia_);
	lpf_b.DoFilter(Ib_);
	lpf_c.DoFilter(Ic_);
	lpf_Iab_.DoFilter(Pa_ * Ia_ + Pb_ * Ib_ + Pc_ * (-Ia_ -Ib_));
}

float ServoDrive::CalculatePhaseCurrent(float adc_val, float adc_bias)
{
	return ((adc_bias - adc_val) * Vref_ / adc_full_scale) / Rsense_ / csa_gain_;
}

void ServoDrive::SignalThreadUpdate()
{
	data_.counter_dir_ = pwm_->GetCounterDirection();
	data_.injdata_[0] = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_1);
	data_.injdata_[1] = LL_ADC_INJ_ReadConversionData12(adc2.hadc_->Instance, LL_ADC_INJ_RANK_1);
	data_.injdata_[2] = LL_ADC_INJ_ReadConversionData12(adc3.hadc_->Instance, LL_ADC_INJ_RANK_1);
	if (data_.counter_dir_) {
		UpdateCurrentBias();
		return;
	}
	data_.vbus_ = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_4);
	data_.theta_ = encoder_dir_ * encoder_->GetElectricPosition();
	data_.update_counter_++;
	sched_.SignalThreadUpdate();
}

bool ServoDrive::RunUpdateHandler(const std::function<bool(void)>& update_handler)
{
	uint32_t flags = sched_.WaitSignals(Scheduler::THREAD_FLAG_ABORT | Scheduler::THREAD_FLAG_UPDATE, 3);
	if (flags == Scheduler::THREAD_FLAG_ABORT) {
		GetPwmGenerator()->Stop();
		return false;
	} else if (flags == Scheduler::THREAD_FLAG_UPDATE) {
		if (data_.counter_dir_)
			UpdateCurrentBias();
		else
			UpdateCurrent();
		UpdateRotor();
		UpdateVbus();
		return update_handler();
	}

	/*
	 * If We got here the UPDATE didn't come
	 */
	GetPwmGenerator()->Stop();
	sched_.Abort();
	return false;
}

float ServoDrive::VoltageToDuty(float voltage, float v_bus)
{
	float v_rms = v_bus * 0.7071f; //(2.0f/3.0f);
	float duty = voltage / v_rms;
	float mod = std::min(duty, 0.4f);
	return mod;
}

bool ServoDrive::GetDutyTimings(float duty_a, float duty_b, float duty_c, uint32_t timing_period, uint32_t& timing_a, uint32_t& timing_b, uint32_t& timing_c)
{
	timing_a = (uint32_t)(duty_a * (float)timing_period);
	timing_b = (uint32_t)(duty_b * (float)timing_period);
	timing_c = (uint32_t)(duty_c * (float)timing_period);
	return true;
}

void ServoDrive::SineSVM(float duty, const std::complex<float>& v_theta, float& duty_a, float& duty_b, float& duty_c)
{
	std::complex<float> theta = v_theta * std::complex<float>(0, 1);
	duty_a = 0.5 + 0.5 * duty * (theta / Pa_).imag();
	duty_b = 0.5 + 0.5 * duty * (theta / Pb_).imag();
	duty_c = 0.5 + 0.5 * duty * (theta / Pc_).imag();
}

void ServoDrive::SaddleSVM(float duty, const std::complex<float>& v_theta, float& duty_a, float& duty_b, float& duty_c)
{
	std::complex<float> theta = v_theta * std::complex<float>(0, 1);
	std::complex<float> theta3 = theta * theta * theta * 0.3333f;
	duty_a = 0.5 + 0.5 * duty * (theta / Pa_ + theta3).imag();
	duty_b = 0.5 + 0.5 * duty * (theta / Pb_ + theta3).imag();
	duty_c = 0.5 + 0.5 * duty * (theta / Pc_ + theta3).imag();
}

bool ServoDrive::ApplyPhaseVoltage(float v_abs, const std::complex<float>& v_theta, float vbus)
{
	uint32_t timing_period = pwm_->GetPeriod();
	uint32_t t1, t2, t3;
	float duty = VoltageToDuty(v_abs, vbus);
	float duty_a = 0, duty_b = 0, duty_c = 0;

#if 0
	std::complex<float> vec = v_theta * duty;
	duty_a = 0.5 + 0.5 * ((vec.real() * Pa_.real() + vec.imag() * Pa_.imag()));
	duty_b = 0.5 + 0.5 * ((vec.real() * Pb_.real() + vec.imag() * Pb_.imag()));
	duty_c = 0.5 + 0.5 * ((vec.real() * Pc_.real() + vec.imag() * Pc_.imag()));
#else

	if (svm_saddle_) {
		SaddleSVM(duty, v_theta, duty_a, duty_b, duty_c);
	} else {
		SineSVM(duty, v_theta, duty_a, duty_b, duty_c);
	}

#endif
	if (!GetDutyTimings(duty_a, duty_b, duty_c, timing_period, t1, t2, t3))
		return false;
	pwm_->SetTiming(1, t1);
	pwm_->SetTiming(2, t2);
	pwm_->SetTiming(3, t3);
	return true;
}

bool ServoDrive::ApplyPhaseDuty(float duty_a, float duty_b, float duty_c)
{
	uint32_t timing_period = pwm_->GetPeriod();
	uint32_t t1, t2, t3;
	if (!GetDutyTimings(duty_a, duty_b, duty_c, timing_period, t1, t2, t3))
		return false;
	pwm_->SetTiming(1, t1);
	pwm_->SetTiming(2, t2);
	pwm_->SetTiming(3, t3);
	return true;
}

void ServoDrive::AddTaskArmMotor()
{
	sched_.AddTask([&](){
		ApplyPhaseVoltage(0, std::polar<float>(1.0f, 0.0f), lpf_vbus_.Output());
		pwm_->Start();
	});
}

void ServoDrive::AddTaskDisarmMotor()
{
	sched_.AddTask([&](){
		ApplyPhaseVoltage(0, std::polar<float>(0.0f, 0.0f), lpf_vbus_.Output());
		pwm_->Stop();
	});
}

bool ServoDrive::RunUpdateHandlerRotateMotor(float angle, float speed, float voltage, bool dir)
{
	float el_speed = speed * pole_pairs;
	float total_rotation = angle * pole_pairs;
	float total_time = total_rotation / el_speed;
	uint32_t total_cycles = update_hz_ * total_time;
	std::complex<float> step = std::polar<float>(1.0, total_rotation/total_cycles);
	std::complex<float> rotation = std::polar<float>(1.0, 0);
	bool ret = true;

	if (!dir)
		step = std::conj(step);
	for (size_t i = 0; ret && i < total_cycles; i++) {
		ret = RunUpdateHandler([&]()->bool {
			ApplyPhaseVoltage(voltage, rotation, lpf_vbus_.Output());
			rotation *= step;
			return true;
		});
	}
	return ret;
}


void ServoDrive::AddTaskRotateMotor(float angle, float speed, float voltage, bool dir)
{
	sched_.AddTask(std::bind([&](float angle, float speed, float voltage, bool dir){
		RunUpdateHandlerRotateMotor(angle, speed, voltage, dir);
	}, angle, speed, voltage, dir));
}


void ServoDrive::AddTaskResetRotor(float reset_voltage, uint32_t reset_hz, bool reset_encoder)
{
	sched_.AddTask(std::bind([&](float reset_voltage, uint32_t reset_hz, bool reset_encoder){
		bool ret = true;
		uint32_t set_angle_cycles = update_hz_ / reset_hz;
		for (float angle = M_PI_4; angle > 0.1; angle = angle * 3 / 4) {
			for (size_t i = 0; (i < set_angle_cycles) && ret; i++) {
				ret = RunUpdateHandler([&]()->bool {
					ApplyPhaseVoltage(reset_voltage, std::polar<float>(1.0f, angle), lpf_vbus_.Output());
					return true;
				});
			};
			for (size_t i = 0; (i < set_angle_cycles) && ret; i++) {
				ret = RunUpdateHandler([&]()->bool {
					ApplyPhaseVoltage(reset_voltage, std::polar<float>(1.0f, -angle), lpf_vbus_.Output());
					return true;
				});
			};
		}
		for (size_t i = 0; (i < update_hz_) && ret; i++) {
			ret = RunUpdateHandler([&]()->bool {
				ApplyPhaseVoltage(reset_voltage * 1.5, std::polar<float>(1.0f, 0), lpf_vbus_.Output());
				if (reset_encoder)
					encoder_->ResetPosition(0);
				return true;
			});
		};
	}, reset_voltage, reset_hz, reset_encoder));
}

void ServoDrive::AddTaskDetectEncoderDir()
{
	sched_.AddTask([&](){
		if (RunUpdateHandlerRotateMotor(M_PI_2, M_PI, reset_voltage_, true))
			encoder_dir_ = (encoder_->GetMechanicalPosition() > M_PI) ? -1 : 1;
	});
}

void ServoDrive::RunSpinTasks()
{
	AddTaskArmMotor();
	AddTaskResetRotor(reset_voltage_, reset_hz_);
	if (encoder_dir_ == 0)
		AddTaskDetectEncoderDir();
	sched_.AddTask([&](){
		float rot_voltage = 0.4; // Volts
		bool ret = false;
		uint32_t display_counter = 0;
		data_.update_counter_ = 0;
		do {
			ret = RunUpdateHandler([&]()->bool {
				std::complex<float> rotor = lpf_e_rotor_.Output();
				std::complex<float> ri_vec = std::polar<float>(1.0f, ri_angle_);
				ApplyPhaseVoltage(rot_voltage, rotor * ri_vec, lpf_vbus_.Output());
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
					fprintf(stderr, "Vbus: %4.2f, SP: %6.1f, arg(R): %6.1f, arg(I): %6.1f, abs(I): %6.3f, DIFF: %+7.1f (%+4.2f), EncIn: %6ld\n",
							lpf_vbus_.Output(), speed, Rarg / M_PI * 180.0f, Iarg / M_PI * 180.0f, Iabs,
							diff / M_PI * 180.0f, lpf_RIdot_.Output(), encoder_->GetIndexPosition());
				}
				return true;
			});
		} while (ret);
	});

	sched_.Run();
}

void ServoDrive::RunTaskAphaPoleSearch()
{
	AddTaskArmMotor();
	if (encoder_dir_ == 0) {
		AddTaskResetRotor(reset_voltage_, reset_hz_);
		AddTaskDetectEncoderDir();
	}
	AddTaskResetRotor(reset_voltage_, reset_hz_);
	for (size_t i = 0; i < pole_pairs; i++) {
		AddTaskRotateMotor((M_PI * 2)/pole_pairs, M_PI, 0.45, true);
		AddTaskResetRotor(reset_voltage_, reset_hz_, false);
		sched_.AddTask([&](void){
			fprintf(stderr, "Enc: %7lu\n", encoder_->GetPosition());
			encoder_->ResetPosition(0);
		});
	}
	AddTaskDisarmMotor();
	sched_.Run();
}

float ServoDrive::RunResistanceMeasurement(float seconds, float test_voltage, float max_current)
{
	LowPassFilter<std::complex<float>, float> lpf_Imes(0.01);

	sched_.AddTask([&](){
		uint32_t i = 0;
		uint32_t display_counter = 0;
		uint32_t test_cycles = update_hz_ * seconds;
		bool ret = false;
		pwm_->Start();
		do {
			ret = RunUpdateHandler([&]()->bool {
				ApplyPhaseDuty(VoltageToDuty(test_voltage, lpf_vbus_.Output()), 0.0f, 0.0f);
				if (lpf_a.Output() > max_current) {
					Stop();
					return false;
				}

				if (!data_.counter_dir_ && (display_counter++ % 13) == 0) {
					fprintf(stderr, "Vbus: %4.2f, arg(I): %6.1f, abs(I): %6.3f, Ia: %6.3f, Ib: %6.3f, Ic: %6.3f, Ia+Ib+Ic: %6.3f\n",
							lpf_vbus_.Output(), std::arg(lpf_Iab_.Output()) / M_PI * 180.0f, std::abs(lpf_Iab_.Output()),
							lpf_a.Output(), lpf_b.Output(), lpf_c.Output(), lpf_a.Output() + lpf_b.Output() + lpf_c.Output());
				}
				return true;
			});

		} while (ret && i++ < test_cycles);
		pwm_->Stop();
	});

	sched_.RunWaitForCompletion();
	return 2.0f / 3.0f * test_voltage / lpf_a.Output();
}


float ServoDrive::RunResistanceMeasurementOD(float seconds, float test_current, float max_voltage)
{
	LowPassFilter<std::complex<float>, float> lpf_Imes(0.01);
    static const float kI = 10.0f;                                 // [(V/s)/A]
    float test_voltage = 0.0f;

	sched_.AddTask([&](){
		uint32_t i = 0;
		uint32_t display_counter = 0;
		uint32_t test_cycles = update_hz_ * seconds;
		bool ret = false;
		pwm_->Start();
		do {
			ret = RunUpdateHandler([&]()->bool {
				float Ialpha = lpf_a.Output();
				test_voltage += (kI / update_hz_) * (test_current - Ialpha);
				if (test_voltage > max_voltage || test_voltage < -max_voltage) {
					Stop();
					fprintf(stderr, "(test_voltage > max_voltage || test_voltage < -max_voltage) failed! Vmax: %4.2f, Vtest: %4.2f\n", max_voltage, test_voltage);
					return false;
				}

				if (!ApplyPhaseVoltage(test_voltage, std::complex<float>(1.0f, 0.0f), lpf_vbus_.Output())) {
					Stop();
					fprintf(stderr, "ApplyPhaseVoltage failed...\n");
					return false;
				}

				if (!data_.counter_dir_ && (display_counter++ % 13) == 0) {
					fprintf(stderr, "Vbus: %4.2f, Vt: %4.2f, arg(I): %6.1f, abs(I): %6.3f, Ia: %6.3f, Ib: %6.3f, Ic: %6.3f, Ia+Ib+Ic: %6.3f\n",
							lpf_vbus_.Output(), test_voltage, std::arg(lpf_Iab_.Output()) / M_PI * 180.0f, std::abs(lpf_Iab_.Output()),
							lpf_a.Output(), lpf_b.Output(), lpf_c.Output(), lpf_a.Output() + lpf_b.Output() + lpf_c.Output());
				}
				return true;
			});

		} while (ret && i++ < test_cycles);
		pwm_->Stop();
	});

	sched_.RunWaitForCompletion();
	float R = test_voltage / test_current;
	return R;
}


float ServoDrive::RunInductanceMeasurementOD(int num_cycles, float voltage_low, float voltage_high)
{
	float L = 0.0f;

	sched_.AddTask([&]() {
		float test_voltages[2] = {voltage_low, voltage_high};
		float Ialphas[2] = {0.0f};
		bool ret = false;
		int t = 0;

		pwm_->Start();
		do {
			ret = RunUpdateHandler([&]()->bool {
				int i = t & 1;
				Ialphas[i] += Ia_;

				// Test voltage along phase A
				if (!ApplyPhaseVoltage(test_voltages[i], std::complex<float>(1.0f, 0.0f), lpf_vbus_.Output())) {
					Stop();
					fprintf(stderr, "ApplyPhaseVoltage failed...\n");
					return false;
				}
				if (!data_.counter_dir_ && (t % 13) == 0) {
					fprintf(stderr, "Vbus: %4.2f, Ia: %6.3f\n",
							lpf_vbus_.Output(), Ia_);
				}


				return true;
			});
		} while (ret && ++t < (num_cycles * 2));
		pwm_->Stop();
		float v_L = 0.5f * (voltage_high - voltage_low);
		float current_meas_period = 1.0f / update_hz_;
		float dI_by_dt = (Ialphas[0] - Ialphas[1]) / (current_meas_period * (float)num_cycles);
		L = v_L / dI_by_dt;
	});
	sched_.RunWaitForCompletion();
	return L;
}

void ServoDrive::RunSimpleTasks()
{
	sched_.AddTask([&](){
		uint32_t t0 = xTaskGetTickCount();
		if (sched_.WaitSignals(Scheduler::THREAD_FLAG_ABORT | Scheduler::THREAD_FLAG_UPDATE, 2000) == Scheduler::THREAD_FLAG_ABORT) {
			fprintf(stderr, "Task1 Aborting...\n\n\n");
			return;
		}
		fprintf(stderr, "Task1 finished %lu\n", xTaskGetTickCount() - t0);
	});
	sched_.AddTask([&](){
		uint32_t t0 = xTaskGetTickCount();
		if (sched_.WaitSignalAbort(2000)) {
			fprintf(stderr, "Task2 Aborting...\n\n\n");
			return;
		}
		fprintf(stderr, "Task2 finished %lu\n", xTaskGetTickCount() - t0);
	});
	sched_.AddTask([&](){
		uint32_t t0 = xTaskGetTickCount();
		if (sched_.WaitSignalAbort(2000)) {
			fprintf(stderr, "Task3 Aborting...\n\n\n");
			return;
		}
		fprintf(stderr, "Task3 finished %lu\n\n\n", xTaskGetTickCount() - t0);
	});
	sched_.Run();

}

void ServoDrive::SchedulerRun()
{
	sched_.Run();
}

void ServoDrive::SchedulerAbort()
{
	sched_.Abort();
}

