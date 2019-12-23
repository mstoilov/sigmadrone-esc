/*
 * servo_drive.cpp
 *
 *  Created on: Oct 15, 2019
 *      Author: mstoilov
 */

#include "main.h"
#include "iencoder.h"
#include "adc.h"
#include "drv8323.h"
#include "motordrive.h"
#include "uartrpcserver.h"
#include "sdmath.h"

extern UartRpcServer rpc_server;
extern Adc adc1;
extern Adc adc2;
extern Adc adc3;
extern Drv8323 drv1;
extern MinasA4Encoder ma4_abs_encoder;
static std::complex<float> p1_ = std::polar<float>(1.0f, 0.0f);
static std::complex<float> p2_ = std::polar<float>(1.0f, M_PI * 2.0f / 3.0f);
static std::complex<float> p3_ = std::polar<float>(1.0f, M_PI * 4.0f / 3.0f);

MotorDrive::MotorDrive(IEncoder* encoder, IPwmGenerator *pwm, uint32_t update_hz)
	: Pa_(std::polar<float>(1.0f, 0.0f))
	, Pb_(std::polar<float>(1.0f, M_PI / 3.0 * 2.0 ))
	, Pc_(std::polar<float>(1.0f, M_PI / 3.0 * 4.0))
	, update_hz_(update_hz)
	, lpf_bias_a(config_.bias_alpha_)
	, lpf_bias_b(config_.bias_alpha_)
	, lpf_bias_c(config_.bias_alpha_)
	, lpf_a(config_.abc_alpha_)
	, lpf_b(config_.abc_alpha_)
	, lpf_c(config_.abc_alpha_)
	, lpf_e_rotor_(config_.rotor_alpha_)
	, lpf_Iab_(config_.i_alpha_)
	, lpf_Idq_(config_.i_alpha_)
	, lpf_Iabs_(config_.iabs_alpha_)
	, lpf_RIdot_(config_.ridot_alpha_)
	, lpf_RIdot_disp_(config_.ridot_disp_alpha_)
	, lpf_vbus_(0.5f, 12.0f)
	, lpf_speed_(config_.speed_alpha_)
	, pid_current_arg_(0.4, 500, 0, 0.99)
{
	lpf_bias_a.Reset(1 << 11);
	lpf_bias_b.Reset(1 << 11);
	lpf_bias_c.Reset(1 << 11);
	encoder_ = encoder;
	pwm_ = pwm;
	props_= rexjson::property_map({
		{"speed", rexjson::property(&lpf_speed_.out_, rexjson::property_access::readonly)},
		{"update_hz", rexjson::property(&update_hz, rexjson::property_access::readonly)},
		{"bias_alpha", rexjson::property(
				&config_.bias_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_bias_a.SetAlpha(config_.bias_alpha_);
					lpf_bias_b.SetAlpha(config_.bias_alpha_);
					lpf_bias_c.SetAlpha(config_.bias_alpha_);
				})},
		{"abc_alpha", rexjson::property(
				&config_.abc_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_a.SetAlpha(config_.abc_alpha_);
					lpf_b.SetAlpha(config_.abc_alpha_);
					lpf_c.SetAlpha(config_.abc_alpha_);
				})},
		{"i_alpha", rexjson::property(
				&config_.i_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_Iab_.SetAlpha(config_.i_alpha_);
					lpf_Idq_.SetAlpha(config_.i_alpha_);
				})},
		{"iabs_alpha", rexjson::property(
				&config_.iabs_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_Iabs_.SetAlpha(config_.iabs_alpha_);
				})},
		{"rotor_alpha", rexjson::property(
				&config_.rotor_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_e_rotor_.SetAlpha(config_.rotor_alpha_);
				})},
		{"speed_alpha", rexjson::property(
				&config_.speed_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_speed_.SetAlpha(config_.speed_alpha_);
				})},
		{"ridot_alpha", rexjson::property(
				&config_.ridot_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_RIdot_.SetAlpha(config_.ridot_alpha_);
		})},
		{"ridot_disp_alpha", rexjson::property(
				&config_.ridot_disp_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_RIdot_disp_.SetAlpha(config_.ridot_disp_alpha_);
		})},
		{"csa_gain", rexjson::property(
				&config_.csa_gain_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){int t = v.get_int(); if (t != 5 && t != 10 && t != 20 && t != 40) throw std::range_error("Invalid value");},
				[&](void*){drv1.SetCSAGainValue(config_.csa_gain_);}
		)},
		{"run_simple_tasks", rexjson::property(
				&run_simple_tasks_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*){ if (run_simple_tasks_) RunSimpleTasks(); else sched_.Abort(); }
		)},
		{"ri_angle", &config_.ri_angle_},
		{"pole_pairs", &config_.pole_pairs},
		{"encoder_dir", &config_.encoder_dir_},
		{"svm_saddle", &config_.svm_saddle_},
		{"reset_voltage", &config_.reset_voltage_},
		{"reset_hz", &config_.reset_hz_},
		{"spin_voltage", &config_.spin_voltage_},
		{"pid_current_arg_kp", &pid_current_arg_.kp_},
		{"pid_current_arg_ki", &pid_current_arg_.ki_},
		{"pid_current_arg_leak", &pid_current_arg_.leak_},
		{"pid_current_arg_output_i_max", &pid_current_arg_.output_i_max_},
	});

	rpc_server.add("drive.start", rexjson::make_rpc_wrapper(this, &MotorDrive::Start, "void ServoDrive::Start()"));
	rpc_server.add("drive.stop", rexjson::make_rpc_wrapper(this, &MotorDrive::Stop, "void ServoDrive::Stop()"));
	rpc_server.add("drive.measure_resistance", rexjson::make_rpc_wrapper(this, &MotorDrive::RunResistanceMeasurement, "float ServoDrive::RunResistanceMeasurement(uint32_t seconds, float test_voltage, float max_current)"));
	rpc_server.add("drive.measure_resistance_od", rexjson::make_rpc_wrapper(this, &MotorDrive::RunResistanceMeasurementOD, "float ServoDrive::RunResistanceMeasurementOD(float seconds, float test_current, float max_voltage);"));
	rpc_server.add("drive.measure_inductance_od", rexjson::make_rpc_wrapper(this, &MotorDrive::RunInductanceMeasurementOD, "float ServoDrive::RunInductanceMeasurementOD(int num_cycles, float voltage_low, float voltage_high);"));

	rpc_server.add("drive.scheduler_run", rexjson::make_rpc_wrapper(this, &MotorDrive::SchedulerRun, "void SchedulerRun()"));
	rpc_server.add("drive.scheduler_abort", rexjson::make_rpc_wrapper(this, &MotorDrive::SchedulerAbort, "void SchedulerAbort()"));
	rpc_server.add("drive.add_task_arm_motor", rexjson::make_rpc_wrapper(this, &MotorDrive::AddTaskArmMotor, "void AddTaskArmMotor()"));
	rpc_server.add("drive.add_task_disarm_motor", rexjson::make_rpc_wrapper(this, &MotorDrive::AddTaskDisarmMotor, "void AddTaskDisarmMotor()"));
	rpc_server.add("drive.add_task_rotate_motor", rexjson::make_rpc_wrapper(this, &MotorDrive::AddTaskRotateMotor, "void AddTaskRotateMotor(float angle, float speed, float voltage, bool dir)"));
	rpc_server.add("drive.add_task_reset_rotor", rexjson::make_rpc_wrapper(this, &MotorDrive::AddTaskResetRotorWithParams, "void AddTaskResetRotorWithParams(float reset_voltage, uint32_t reset_hz)"));
	rpc_server.add("drive.alpha_pole_search", rexjson::make_rpc_wrapper(this, &MotorDrive::RunTaskAphaPoleSearch, "void RunTaskAphaPoleSearch()"));
	rpc_server.add("drive.rotate", rexjson::make_rpc_wrapper(this, &MotorDrive::RunTaskRotateMotor, "void RunTaskRotateMotor(float angle, float speed, float voltage, bool dir)"));
	rpc_server.add("drive.drv_get_fault1", rexjson::make_rpc_wrapper(&drv1, &Drv8323::GetFaultStatus1, "uint32_t Drv8323::GetFaultStatus1()"));
	rpc_server.add("drive.drv_get_fault2", rexjson::make_rpc_wrapper(&drv1, &Drv8323::GetFaultStatus2, "uint32_t Drv8323::GetFaultStatus2()"));
	rpc_server.add("drive.drv_clear_fault", rexjson::make_rpc_wrapper(&drv1, &Drv8323::ClearFault, "void Drv8323::ClearFault()"));

	sched_.SetAbortTask([&]()->void {
		pwm_->Stop();
	});

	sched_.SetIdleTask([&]()->void {
	});

	pid_current_arg_.SetMaxIntegralOutput(M_PI);
}

MotorDrive::~MotorDrive()
{
	// TODO Auto-generated destructor stub
}

void MotorDrive::Attach()
{
	drv1.SetCSAGainValue(config_.csa_gain_);
	sched_.StartDispatcherThread();
}

void MotorDrive::Start()
{
	osDelay(20);
	RunSpinTasks();
}

void MotorDrive::Stop()
{
	pwm_->Stop();
	sched_.Abort();
	osDelay(20);

}

IEncoder* MotorDrive::GetEncoder() const
{
	return encoder_;
}

void MotorDrive::SetEncoder(IEncoder* encoder)
{
	if (encoder)
		encoder_ = encoder;
}


int32_t MotorDrive::GetEncoderDir() const
{
	return config_.encoder_dir_;
}

uint32_t MotorDrive::GetUpdateFrequency() const
{
	return update_hz_;
}

uint32_t MotorDrive::GetPolePairs() const
{
	return config_.pole_pairs;
}


float MotorDrive::GetBusVoltage() const
{
	return lpf_vbus_.Output();
}

std::complex<float> MotorDrive::GetPhaseCurrent() const
{
	return lpf_Iab_.Output();
}

std::complex<float> MotorDrive::GetElecRotation() const
{
	return lpf_e_rotor_.Output();
}


bool MotorDrive::IsStarted()
{
	return pwm_->IsStarted();
}

void MotorDrive::IrqUpdateCallback()
{
	if (pwm_->GetCounterDirection()) {
		t1_ = hrtimer.GetCounter();
		ma4_abs_encoder.Update();

		encreply_.crc_ = 0;
		data_.injdata_[0] = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_1);
		data_.injdata_[1] = LL_ADC_INJ_ReadConversionData12(adc2.hadc_->Instance, LL_ADC_INJ_RANK_1);
		data_.injdata_[2] = LL_ADC_INJ_ReadConversionData12(adc3.hadc_->Instance, LL_ADC_INJ_RANK_1);
		data_.vbus_ = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_4);

		/*
		 * Sample ADC bias
		 */
		lpf_bias_a.DoFilter(data_.injdata_[0]);
		lpf_bias_b.DoFilter(data_.injdata_[1]);
		lpf_bias_c.DoFilter(data_.injdata_[2]);
	} else {
		if (update_handler_active_)
			return;
		if (!t2_)
			t2_ = hrtimer.GetCounter();

		data_.update_counter_++;
		sched_.SignalThreadUpdate();
	}
}

void MotorDrive::UpdateRotor()
{
	uint32_t rotor_position = encoder_->GetPosition();
	if (rotor_position != (uint32_t)-1) {
		data_.theta_ = config_.encoder_dir_ * (encoder_->GetElectricPosition(rotor_position, config_.pole_pairs));
		std::complex<float> r_prev = lpf_e_rotor_.Output();
		std::complex<float> r_cur = lpf_e_rotor_.DoFilter(std::polar(1.0f, data_.theta_));
		lpf_speed_.DoFilter(sdmath::cross(r_prev, r_cur));
	}

}

void MotorDrive::UpdateVbus()
{
	lpf_vbus_.DoFilter(__LL_ADC_CALC_DATA_TO_VOLTAGE(config_.Vref_, data_.vbus_, LL_ADC_RESOLUTION_12B) * config_.Vbus_resistor_ratio_);
}

void MotorDrive::UpdateCurrent()
{
	lpf_a.DoFilter(data_.phase_current_a_);
	lpf_b.DoFilter(data_.phase_current_b_);
	lpf_c.DoFilter(data_.phase_current_c_);
	lpf_Iab_.DoFilter(Pa_ * data_.phase_current_a_ + Pb_ * data_.phase_current_b_ + Pc_ * (-data_.phase_current_a_ -data_.phase_current_b_));
}

float MotorDrive::CalculatePhaseCurrent(float adc_val, float adc_bias)
{
	return ((adc_bias - adc_val) * config_.Vref_ / config_.adc_full_scale) / config_.Rsense_ / config_.csa_gain_;
}

bool MotorDrive::RunUpdateHandler(const std::function<bool(void)>& update_handler)
{
	uint32_t flags = sched_.WaitSignals(Scheduler::THREAD_FLAG_ABORT | Scheduler::THREAD_FLAG_UPDATE, 3);
	if (flags == Scheduler::THREAD_FLAG_ABORT) {
		pwm_->Stop();
		return false;
	} else if (flags == Scheduler::THREAD_FLAG_UPDATE) {
		update_handler_active_ = true;
		/*
		 * Sample ADC phase current
		 */
		data_.injdata_[0] = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_1);
		data_.injdata_[1] = LL_ADC_INJ_ReadConversionData12(adc2.hadc_->Instance, LL_ADC_INJ_RANK_1);
		data_.injdata_[2] = LL_ADC_INJ_ReadConversionData12(adc3.hadc_->Instance, LL_ADC_INJ_RANK_1);
		data_.vbus_ = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_4);

		data_.phase_current_a_ = CalculatePhaseCurrent(data_.injdata_[0], lpf_bias_a.Output());
		data_.phase_current_b_ = CalculatePhaseCurrent(data_.injdata_[1], lpf_bias_a.Output());
		data_.phase_current_c_ = CalculatePhaseCurrent(data_.injdata_[2], lpf_bias_a.Output());

		UpdateCurrent();
		UpdateVbus();
		UpdateRotor();
		t3_ = hrtimer.GetCounter();
		signal_time_ms_ = hrtimer.GetTimeElapsedMicroSec(t2_, t3_);
		bool ret = update_handler();
		t2_ = 0;
		update_handler_active_ = false;
		return ret;
	}

	/*
	 * If we got here the UPDATE didn't come
	 */
	pwm_->Stop();
	sched_.Abort();
	return false;
}

float MotorDrive::VoltageToDuty(float voltage, float v_bus)
{
	float v_rms = v_bus * 0.7071f;
	float duty = voltage / v_rms;
	return duty;
}

bool MotorDrive::GetDutyTimings(float duty_a, float duty_b, float duty_c, uint32_t timing_period, uint32_t& timing_a, uint32_t& timing_b, uint32_t& timing_c)
{
	timing_a = (uint32_t)(duty_a * (float)timing_period);
	timing_b = (uint32_t)(duty_b * (float)timing_period);
	timing_c = (uint32_t)(duty_c * (float)timing_period);
	return true;
}

void MotorDrive::SineSVM(float duty, const std::complex<float>& v_theta, float& duty_a, float& duty_b, float& duty_c)
{
	std::complex<float> theta = v_theta * std::complex<float>(0, 1);
	duty_a = 0.5 + 0.5 * duty * (theta / Pa_).imag();
	duty_b = 0.5 + 0.5 * duty * (theta / Pb_).imag();
	duty_c = 0.5 + 0.5 * duty * (theta / Pc_).imag();
}

void MotorDrive::SaddleSVM(float duty, const std::complex<float>& v_theta, float& duty_a, float& duty_b, float& duty_c)
{
	std::complex<float> theta = v_theta * std::complex<float>(0, 1);
	std::complex<float> theta3 = theta * theta * theta * 0.3333f;
	duty_a = 0.5 + 0.5 * duty * (theta / Pa_ + theta3).imag();
	duty_b = 0.5 + 0.5 * duty * (theta / Pb_ + theta3).imag();
	duty_c = 0.5 + 0.5 * duty * (theta / Pc_ + theta3).imag();
}

bool MotorDrive::ApplyPhaseVoltage(float v_abs, const std::complex<float>& v_theta, float vbus)
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

	if (config_.svm_saddle_) {
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

bool MotorDrive::ApplyPhaseDuty(float duty_a, float duty_b, float duty_c)
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

void MotorDrive::AddTaskArmMotor()
{
	sched_.AddTask([&](){
		ApplyPhaseVoltage(0, std::polar<float>(1.0f, 0.0f), lpf_vbus_.Output());
		pwm_->Start();
	});
}

void MotorDrive::AddTaskDisarmMotor()
{
	sched_.AddTask([&](){
		ApplyPhaseVoltage(0, std::polar<float>(0.0f, 0.0f), lpf_vbus_.Output());
		pwm_->Stop();
	});
}

bool MotorDrive::RunUpdateHandlerRotateMotor(float angle, float speed, float voltage, bool dir)
{
	float el_speed = speed * config_.pole_pairs;
	float total_rotation = angle * config_.pole_pairs;
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

void MotorDrive::AddTaskRotateMotor(float angle, float speed, float voltage, bool dir)
{
	sched_.AddTask(std::bind([&](float angle, float speed, float voltage, bool dir){
		RunUpdateHandlerRotateMotor(angle, speed, voltage, dir);
	}, angle, speed, voltage, dir));
}

void MotorDrive::AddTaskResetRotor()
{
	AddTaskResetRotorWithParams(config_.reset_voltage_, config_.reset_hz_, true);
}

void MotorDrive::AddTaskResetRotorWithParams(float reset_voltage, uint32_t reset_hz, bool reset_encoder)
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
					encoder_->ResetPosition();
				return true;
			});
		};
	}, reset_voltage, reset_hz, reset_encoder));
}

void MotorDrive::AddTaskDetectEncoderDir()
{
	sched_.AddTask([&](){
		if (RunUpdateHandlerRotateMotor(M_PI_2, M_PI, config_.reset_voltage_, true))
			config_.encoder_dir_ = (encoder_->GetMechanicalPosition(encoder_->GetPosition()) > M_PI) ? -1 : 1;
	});
}

void MotorDrive::RunSpinTasks()
{
	AddTaskArmMotor();
	AddTaskResetRotorWithParams(config_.reset_voltage_, config_.reset_hz_);
	if (config_.encoder_dir_ == 0)
		AddTaskDetectEncoderDir();
	sched_.AddTask([&](){
		bool ret = false;
		uint32_t display_counter = 0;
		data_.update_counter_ = 0;
		do {
			ret = RunUpdateHandler([&]()->bool {
				std::complex<float> I = lpf_Iab_.Output();
				float Iarg = std::arg(I);
				float Iabs = std::abs(I);
				std::complex<float> Inorm = std::polar<float>(1.0f, Iarg);
				std::complex<float> rotor = lpf_e_rotor_.Output();
				float Rarg = std::arg(rotor);
				float ri_dot = sdmath::dot(rotor, Inorm);
				lpf_RIdot_.DoFilter(ri_dot);
				lpf_RIdot_disp_.DoFilter(ri_dot);
				pid_current_arg_.Input(lpf_RIdot_.Output(), 1.0f/update_hz_);


//				std::complex<float> ri_vec = std::polar<float>(1.0f, config_.ri_angle_);
				std::complex<float> ri_vec = std::polar<float>(1.0f, config_.ri_angle_ + pid_current_arg_.Output());
				ApplyPhaseVoltage(config_.spin_voltage_, rotor * ri_vec, lpf_vbus_.Output());
				float diff = acosf(lpf_RIdot_disp_.Output());

				if ((display_counter++ % 13) == 0) {
					if (Rarg < 0.0f)
						Rarg += M_PI * 2.0f;
					if (Iarg < 0.0f)
						Iarg += M_PI * 2.0f;
					float speed = (asinf(lpf_speed_.Output()) * update_hz_)/(M_PI*2.0 * config_.pole_pairs);
					fprintf(stderr, "SigTime: %5lu, Vbus: %4.2f, RPM: %6.1f, arg(R): %6.1f, arg(I): %6.1f, abs(I): %6.3f, DIFF: %+7.1f (%+4.2f), Pid.Out: %8.5f (%5.2f)\n",
							signal_time_ms_,
							lpf_vbus_.Output(),
							speed, Rarg / M_PI * 180.0f, Iarg / M_PI * 180.0f, lpf_Iabs_.DoFilter(Iabs),
							diff / M_PI * 180.0f, lpf_RIdot_disp_.Output(), pid_current_arg_.Output(), pid_current_arg_.Output() / M_PI * 180.0f);
				}
				return true;
			});
		} while (ret);
	});

	sched_.Run();
}

void MotorDrive::RunTaskAphaPoleSearch()
{
	AddTaskArmMotor();
	if (config_.encoder_dir_ == 0) {
		AddTaskResetRotorWithParams(config_.reset_voltage_, config_.reset_hz_);
		AddTaskDetectEncoderDir();
	}
	AddTaskResetRotorWithParams(config_.reset_voltage_, config_.reset_hz_);
	for (size_t i = 0; i < config_.pole_pairs; i++) {
		AddTaskRotateMotor((M_PI * 2)/config_.pole_pairs, M_PI, 0.45, true);
		AddTaskResetRotorWithParams(config_.reset_voltage_, config_.reset_hz_, false);
		sched_.AddTask([&](void){
			fprintf(stderr, "Enc: %7lu\n", encoder_->GetPosition());
			encoder_->ResetPosition();
		});
	}
	AddTaskDisarmMotor();
	sched_.Run();
}

float MotorDrive::RunResistanceMeasurement(float seconds, float test_voltage, float max_current)
{
	LowPassFilter<std::complex<float>, float> lpf_Imes(0.01);

	sched_.AddTask(std::bind([&](float seconds, float test_voltage, float max_current){
		uint32_t i = 0;
		uint32_t display_counter = 0;
		uint32_t test_cycles = update_hz_ * seconds;
		bool ret = false;
		pwm_->Start();
		config_.resistance_ = 0;
		do {
			ret = RunUpdateHandler([&]()->bool {
//				ApplyPhaseDuty(VoltageToDuty(test_voltage, lpf_vbus_.Output()), 0.0f, 0.0f);
				ApplyPhaseVoltage(test_voltage, std::polar<float>(1, 0), lpf_vbus_.Output());
				if (lpf_a.Output() > max_current) {
					Stop();
					fprintf(stderr, "Max current exceeded! Current: %5.2f\n", lpf_a.Output());
					return false;
				}

				if ((display_counter++ % 13) == 0) {
					fprintf(stderr, "Vbus: %4.2f, Ia: %6.3f, Ib: %6.3f, Ic: %6.3f, Ia+Ib+Ic: %6.3f\n",
							lpf_vbus_.Output(), lpf_a.Output(), lpf_b.Output(), lpf_c.Output(), lpf_a.Output() + lpf_b.Output() + lpf_c.Output());
				}
				return true;
			});

		} while (ret && i++ < test_cycles);
		if (ret)
			config_.resistance_ = 2.0f / 3.0f * test_voltage / lpf_a.Output();
		pwm_->Stop();
	}, seconds, test_voltage, max_current));

	sched_.RunWaitForCompletion();
	return config_.resistance_;
}


float MotorDrive::RunResistanceMeasurementOD(float seconds, float test_current, float max_voltage)
{
	LowPassFilter<std::complex<float>, float> lpf_Imes(0.01);
    static const float kI = 10.0f;                                 // [(V/s)/A]
    float test_voltage = 0.0f;

	sched_.AddTask(std::bind([&](float seconds, float test_current, float max_voltage) {
		uint32_t i = 0;
		uint32_t display_counter = 0;
		uint32_t test_cycles = update_hz_ * seconds;
		bool ret = false;
		pwm_->Start();
		config_.resistance_ = 0;
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

				if ((display_counter++ % 13) == 0) {
					fprintf(stderr, "Vbus: %4.2f, Vt: %4.2f, arg(I): %6.1f, abs(I): %6.3f, Ia: %6.3f, Ib: %6.3f, Ic: %6.3f, Ia+Ib+Ic: %6.3f\n",
							lpf_vbus_.Output(), test_voltage, std::arg(lpf_Iab_.Output()) / M_PI * 180.0f, std::abs(lpf_Iab_.Output()),
							lpf_a.Output(), lpf_b.Output(), lpf_c.Output(), lpf_a.Output() + lpf_b.Output() + lpf_c.Output());
				}
				return true;
			});

		} while (ret && i++ < test_cycles);
		pwm_->Stop();
		if (ret)
			config_.resistance_ = 2.0f / 3.0f * test_voltage / lpf_a.Output();
	}, seconds, test_current, max_voltage));

	sched_.RunWaitForCompletion();
	return config_.resistance_;
}


float MotorDrive::RunInductanceMeasurementOD(int num_cycles, float voltage_low, float voltage_high)
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
				Ialphas[i] += data_.phase_current_a_;

				// Test voltage along phase A
				if (!ApplyPhaseVoltage(test_voltages[i], std::complex<float>(1.0f, 0.0f), lpf_vbus_.Output())) {
					Stop();
					fprintf(stderr, "ApplyPhaseVoltage failed...\n");
					return false;
				}
				if ((t % 13) == 0) {
					fprintf(stderr, "Vbus: %4.2f, Ia: %6.3f\n",
							lpf_vbus_.Output(), data_.phase_current_a_);
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

void MotorDrive::RunTaskRotateMotor(float angle, float speed, float voltage, bool dir)
{
	AddTaskArmMotor();
	AddTaskResetRotorWithParams(voltage, 35, false);
	AddTaskRotateMotor(angle, speed, voltage, dir);
	AddTaskDisarmMotor();
	sched_.RunWaitForCompletion();
}


void MotorDrive::RunSimpleTasks()
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

void MotorDrive::SchedulerRun()
{
	sched_.Run();
}

void MotorDrive::SchedulerAbort()
{
	sched_.Abort();
}

