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
static std::complex<float> p1_ = std::polar<float>(1.0f, 0.0f);
static std::complex<float> p2_ = std::polar<float>(1.0f, M_PI * 2.0f / 3.0f);
static std::complex<float> p3_ = std::polar<float>(1.0f, M_PI * 4.0f / 3.0f);

MotorDrive::MotorDrive(IEncoder* encoder, IPwmGenerator *pwm, uint32_t update_hz)
	: Pa_(std::polar<float>(1.0f, 0.0f))
	, Pb_(std::polar<float>(1.0f, M_PI / 3.0 * 2.0 ))
	, Pc_(std::polar<float>(1.0f, M_PI / 3.0 * 4.0))
	, update_hz_(update_hz)
	, update_period_(1.0f / update_hz_)
	, lpf_bias_a(config_.bias_alpha_)
	, lpf_bias_b(config_.bias_alpha_)
	, lpf_bias_c(config_.bias_alpha_)
	, lpf_vbus_(0.5f, 12.0f)
{
	lpf_bias_a.Reset(1 << 11);
	lpf_bias_b.Reset(1 << 11);
	lpf_bias_c.Reset(1 << 11);
	encoder_ = encoder;
	pwm_ = pwm;
	props_= rexjson::property_map({
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
		{"max_modulation_duty", rexjson::property(
				&config_.max_modulation_duty_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");}
		)},
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
		{"calib_max_i", &config_.calib_max_i_},
		{"calib_v", &config_.calib_v_},
		{"resistance", &config_.resistance_},
		{"inductance", &config_.inductance_},
		{"pole_pairs", &config_.pole_pairs},
		{"encoder_dir", &config_.encoder_dir_},
		{"svm_saddle", &config_.svm_saddle_},
		{"reset_voltage", &config_.reset_voltage_},
		{"reset_hz", &config_.reset_hz_},
		{"display_div", &config_.display_div_}

	});

	rpc_server.add("drive.abort", rexjson::make_rpc_wrapper(this, &MotorDrive::Abort, "void ServoDrive::Abort()"));
	rpc_server.add("drive.measure_resistance", rexjson::make_rpc_wrapper(this, &MotorDrive::RunResistanceMeasurement, "float ServoDrive::RunResistanceMeasurement(uint32_t seconds, float test_voltage, float max_current)"));
	rpc_server.add("drive.measure_inductance", rexjson::make_rpc_wrapper(this, &MotorDrive::RunInductanceMeasurement, "float ServoDrive::RunInductanceMeasurement(uint32_t seconds, float test_voltage, float max_current, uint32_t test_hz);"));

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

void MotorDrive::Run()
{
	sched_.Run();
}

void MotorDrive::RunWaitForCompletion()
{
	sched_.RunWaitForCompletion();
}

void MotorDrive::Abort()
{
	pwm_->Stop();
	sched_.Abort();
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

float MotorDrive::GetUpdatePeriod() const
{
	return update_period_;
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
	return Iab_;
}

bool MotorDrive::IsStarted()
{
	return pwm_->IsStarted();
}

void MotorDrive::IrqUpdateCallback()
{
	if (pwm_->GetCounterDirection()) {
		t1_begin_ = hrtimer.GetCounter();

		data_.injdata_[0] = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_1);
		data_.injdata_[1] = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_2);
		data_.injdata_[2] = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_3);
		data_.vbus_ = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_4);
		UpdateVbus();

		/*
		 * Sample ADC bias
		 */
		lpf_bias_a.DoFilter(data_.injdata_[0]);
		lpf_bias_b.DoFilter(data_.injdata_[1]);
		lpf_bias_c.DoFilter(data_.injdata_[2]);
		t1_span_ = hrtimer.GetTimeElapsedMicroSec(t1_begin_, hrtimer.GetCounter());
	} else {
		t2_to_t2_ = hrtimer.GetTimeElapsedMicroSec(t2_begin_, hrtimer.GetCounter());
		t2_begin_ = hrtimer.GetCounter();

		data_.update_counter_++;

		/*
		 * Sample ADC phase current
		 */
		data_.injdata_[0] = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_1);
		data_.injdata_[1] = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_2);
		data_.injdata_[2] = LL_ADC_INJ_ReadConversionData12(adc1.hadc_->Instance, LL_ADC_INJ_RANK_3);

		data_.phase_current_a_ = CalculatePhaseCurrent(data_.injdata_[0], lpf_bias_a.Output());
		data_.phase_current_b_ = CalculatePhaseCurrent(data_.injdata_[1], lpf_bias_a.Output());
		data_.phase_current_c_ = CalculatePhaseCurrent(data_.injdata_[2], lpf_bias_a.Output());
		UpdateCurrent();

//		if (data_.update_counter_ & 1)
//			encoder_->Update();
//		else
//			UpdateRotor();

//		encoder_->Update();
//		UpdateRotor();

		sched_.OnUpdate();
		t2_end_ = hrtimer.GetCounter();
		t2_span_ = hrtimer.GetTimeElapsedMicroSec(t2_begin_,t2_end_);

	}
}

#if 0
void MotorDrive::UpdateRotor()
{
	uint32_t rotor_position = encoder_->GetPosition();
	if (rotor_position != (uint32_t)-1) {
		data_.theta_ = config_.encoder_dir_ * (encoder_->GetElectricPosition(rotor_position, config_.pole_pairs));
		std::complex<float> r_prev = R_;
		R_ = std::complex<float>(cosf(data_.theta_), sinf(data_.theta_));
		std::complex<float> r_cur = R_;
		lpf_speed_.DoFilter(sdmath::cross(r_prev, r_cur));
	}

}
#endif

void MotorDrive::UpdateVbus()
{
	lpf_vbus_.DoFilter(__LL_ADC_CALC_DATA_TO_VOLTAGE(config_.Vref_, data_.vbus_, LL_ADC_RESOLUTION_12B) * config_.Vbus_resistor_ratio_);
}

void MotorDrive::UpdateCurrent()
{
	Iab_ = Pa_ * data_.phase_current_a_ + Pb_ * data_.phase_current_b_ + Pc_ * (-data_.phase_current_a_ -data_.phase_current_b_);
}

float MotorDrive::CalculatePhaseCurrent(float adc_val, float adc_bias)
{
	return ((adc_bias - adc_val) * config_.Vref_ / config_.adc_full_scale) / config_.Rsense_ / config_.csa_gain_;
}

#if 0
bool MotorDrive::RunUpdateHandler(const std::function<bool(void)>& update_handler)
{
	uint32_t flags = sched_.WaitSignals(Scheduler::THREAD_FLAG_ABORT | Scheduler::THREAD_FLAG_UPDATE, 3);
	if (flags == Scheduler::THREAD_FLAG_ABORT) {
		pwm_->Stop();
		return false;
	} else if (flags == Scheduler::THREAD_FLAG_UPDATE) {
		update_handler_active_ = true;
		signal_time_ms_ = hrtimer.GetTimeElapsedMicroSec(t2_end_, hrtimer.GetCounter());
		t3_begin_ = hrtimer.GetCounter();
		t3_span_ = hrtimer.GetTimeElapsedMicroSec(t2_begin_, hrtimer.GetCounter());
		bool ret = update_handler();
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
#endif

float MotorDrive::VoltageToDuty(float voltage, float v_bus)
{
	float v_rms = v_bus * 0.7071f;
	float duty = voltage / v_rms;
	return std::min(duty, config_.max_modulation_duty_);
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
	duty_a = 0.5f + 0.5f * duty * (theta * std::conj(Pa_)).imag();
	duty_b = 0.5f + 0.5f * duty * (theta * std::conj(Pb_)).imag();
	duty_c = 0.5f + 0.5f * duty * (theta * std::conj(Pc_)).imag();
}

void MotorDrive::SaddleSVM(float duty, const std::complex<float>& v_theta, float& duty_a, float& duty_b, float& duty_c)
{
	std::complex<float> theta = v_theta * std::complex<float>(0, 1);
	std::complex<float> theta3 = theta * theta * theta * 0.3333f;
	duty_a = 0.5f + 0.5f * duty * (theta * std::conj(Pa_) + theta3).imag();
	duty_b = 0.5f + 0.5f * duty * (theta * std::conj(Pb_) + theta3).imag();
	duty_c = 0.5f + 0.5f * duty * (theta * std::conj(Pc_) + theta3).imag();
}

bool MotorDrive::ApplyPhaseVoltage(float v_alpha, float v_beta, float vbus)
{
	float v_abs = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
	return ApplyPhaseModulation(VoltageToDuty(v_abs, vbus), std::complex<float>(v_alpha/v_abs, v_beta/v_abs));
}

bool MotorDrive::ApplyPhaseVoltage(float v_abs, const std::complex<float>& v_theta, float vbus)
{
	return ApplyPhaseModulation(VoltageToDuty(v_abs, vbus), v_theta);
}

bool MotorDrive::ApplyPhaseModulation(float mod, const std::complex<float>& v_theta)
{
	uint32_t timing_period = pwm_->GetPeriod();
	uint32_t t1, t2, t3;
	float duty_a = 0, duty_b = 0, duty_c = 0;

#if 0
	std::complex<float> vec = v_theta * mod;
	duty_a = 0.5 + 0.5 * ((vec.real() * Pa_.real() + vec.imag() * Pa_.imag()));
	duty_b = 0.5 + 0.5 * ((vec.real() * Pb_.real() + vec.imag() * Pb_.imag()));
	duty_c = 0.5 + 0.5 * ((vec.real() * Pc_.real() + vec.imag() * Pc_.imag()));
#else

	if (config_.svm_saddle_) {
		SaddleSVM(mod, v_theta, duty_a, duty_b, duty_c);
	} else {
		SineSVM(mod, v_theta, duty_a, duty_b, duty_c);
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
		ret = sched_.RunUpdateHandler([&]()->bool {
			ApplyPhaseVoltage(voltage, rotation, lpf_vbus_.Output());
			rotation *= step;
			return false;
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
				ret = sched_.RunUpdateHandler([&]()->bool {
					ApplyPhaseVoltage(reset_voltage, std::polar<float>(1.0f, angle), lpf_vbus_.Output());
					return false;
				});
			};
			for (size_t i = 0; (i < set_angle_cycles) && ret; i++) {
				ret = sched_.RunUpdateHandler([&]()->bool {
					ApplyPhaseVoltage(reset_voltage, std::polar<float>(1.0f, -angle), lpf_vbus_.Output());
					return false;
				});
			};
		}
		for (size_t i = 0; (i < update_hz_) && ret; i++) {
			ret = sched_.RunUpdateHandler([&]()->bool {
				ApplyPhaseVoltage(reset_voltage * 1.5, std::polar<float>(1.0f, 0), lpf_vbus_.Output());
				if (reset_encoder)
					encoder_->ResetPosition();
				return false;
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
	sched_.AddTask([&](){
		RunUpdateHandlerRotateMotor(M_PI_2, M_PI, config_.reset_voltage_, false);
	});


}

void MotorDrive::AddTaskCalibrationSequence()
{
	AddTaskArmMotor();
	AddTaskResetRotor();
	AddTaskMeasureResistance(1, config_.calib_v_, config_.calib_max_i_);
	AddTaskMeasureInductance(1, config_.calib_v_, config_.calib_max_i_, 3000);
	AddTaskResetRotor();
	AddTaskDetectEncoderDir();
	AddTaskDisarmMotor();
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

void MotorDrive::AddTaskMeasureResistance(float seconds, float test_voltage, float max_current)
{
	sched_.AddTask(std::bind([&](float seconds, float test_voltage, float max_current){
		uint32_t i = 0;
		uint32_t display_counter = 0;
		uint32_t test_cycles = update_hz_ * seconds;
		bool ret = false;

		ApplyPhaseVoltage(0, 0, lpf_vbus_.Output());
		config_.resistance_ = 0;
		do {
			ret = sched_.RunUpdateHandler([&]()->bool {
				return false;
			});

			ApplyPhaseVoltage(test_voltage, std::polar<float>(1, 0), lpf_vbus_.Output());
			if (data_.phase_current_a_ > max_current) {
				Abort();
				fprintf(stderr, "Max current exceeded! Current: %5.2f\n", data_.phase_current_a_);
				return;
			}

			if ((display_counter++ % 13) == 0) {
				fprintf(stderr, "Vbus: %4.2f, Ia: %6.3f, Ib: %6.3f, Ic: %6.3f, Ia+Ib+Ic: %6.3f\n",
						lpf_vbus_.Output(), data_.phase_current_a_, data_.phase_current_b_, data_.phase_current_c_,
						data_.phase_current_a_ + data_.phase_current_b_ + data_.phase_current_c_);
			}

		} while (ret && i++ < test_cycles);
		if (ret)
			config_.resistance_ = 2.0f / 3.0f * test_voltage / data_.phase_current_a_;
		ApplyPhaseVoltage(0, 0, lpf_vbus_.Output());
	}, seconds, test_voltage, max_current));
}

void MotorDrive::AddTaskMeasureInductance(float seconds, float test_voltage, float max_current, uint32_t test_hz)
{
	sched_.AddTask(std::bind([&](float seconds, float test_voltage, float max_current, uint32_t test_hz){
		LowPassFilter<float, float> lpf_a(0.001);
		uint32_t sine_steps = update_hz_ / test_hz;
		uint32_t test_cycles = update_hz_ * seconds;
		bool ret = true;
		std::complex<float> V(1.0, 0.0);
		std::complex<float> r = std::polar<float>(1.0, M_PI * 2 / sine_steps);

		ApplyPhaseVoltage(0, 0, lpf_vbus_.Output());
		for (uint32_t i = 0; ret && i < test_cycles;  i++) {

			ret = sched_.RunUpdateHandler([&]()->bool {

				lpf_a.DoFilter(std::abs(data_.phase_current_a_));
				V = V * r;

				// Test voltage along phase A
				if (!ApplyPhaseVoltage(test_voltage * V.real(), std::complex<float>(1.0f, 0.0f), lpf_vbus_.Output())) {
					Abort();
					fprintf(stderr, "ApplyPhaseVoltage failed...\n");
					return false;
				}

				return false;
			});

			if ((i % 13) == 0) {
				fprintf(stderr, "Vbus: %4.2f, Ia: %6.3f\n",
						lpf_vbus_.Output(), lpf_a.Output());
			}

		}

		float X = (lpf_vbus_.Output() - lpf_a.Output() * config_.resistance_)/ lpf_a.Output();
		float L = X / (2 * M_PI * test_hz);
		config_.inductance_ = L;
	}, seconds, test_voltage, max_current, test_hz));
}


float MotorDrive::RunResistanceMeasurement(float seconds, float test_voltage, float max_current)
{
	AddTaskArmMotor();
	AddTaskMeasureResistance(seconds, test_voltage, max_current);
	AddTaskDisarmMotor();
	sched_.RunWaitForCompletion();
	return config_.resistance_;
}

float MotorDrive::RunInductanceMeasurement(float seconds, float test_voltage, float max_current, uint32_t test_hz)
{
	AddTaskArmMotor();
	AddTaskMeasureInductance(seconds, test_voltage, max_current, test_hz);
	AddTaskDisarmMotor();
	sched_.RunWaitForCompletion();
	return config_.inductance_;
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
		if (sched_.WaitSignals(Scheduler::THREAD_FLAG_ABORT, 2000) == Scheduler::THREAD_FLAG_ABORT) {
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

