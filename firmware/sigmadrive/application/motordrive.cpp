/*
 * servo_drive.cpp
 *
 *  Created on: Oct 15, 2019
 *      Author: mstoilov
 */
#define ARM_MATH_CM7
#include "stm32f745xx.h"
#include "arm_math.h"
#include "main.h"
#include "iencoder.h"
#include "adc.h"
#include "drv8323.h"
#include "motordrive.h"
#include "uartrpcserver.h"
#include "sdmath.h"
#include "pwm_generator.h"


extern UartRpcServer rpc_server;


MotorDrive::MotorDrive(uint32_t axis_idx, Drv8323* drv, Adc* adc, Adc* dma_adc, IEncoder* encoder, IPwmGenerator *pwm, uint32_t update_hz)
	: update_hz_(update_hz)
	, time_slice_(1.0f / update_hz_)
	, lpf_bias_a(config_.bias_alpha_)
	, lpf_bias_b(config_.bias_alpha_)
	, lpf_bias_c(config_.bias_alpha_)
	, lpf_vbus_(config_.vbus_alpha_, 12.0f)
	, lpf_Wenc_(config_.wenc_alpha_)
{
	lpf_bias_a.Reset(1 << 11);
	lpf_bias_b.Reset(1 << 11);
	lpf_bias_c.Reset(1 << 11);
	axis_idx_ = axis_idx;
	drv_ = drv;
	adc_ = adc;
	dma_adc_ = dma_adc;
	encoder_ = encoder;
	pwm_ = pwm;

	sched_.SetAbortTask([&]() -> void {
		pwm_->Stop();
	});

	sched_.SetIdleTask([&]() -> void {
//		DefaultIdleTask();
	});

	sched_.StartDispatcherThread();

}

MotorDrive::~MotorDrive()
{
}

void MotorDrive::Attach()
{
	drv_->SetCSAGainValue(config_.csa_gain_);
	SetEncoder(encoder_);
}

void MotorDrive::RegisterRpcMethods(const std::string& prefix)
{
	rpc_server.add(prefix, "abort", rexjson::make_rpc_wrapper(this, &MotorDrive::Abort, "void ServoDrive::Abort()"));
	rpc_server.add(prefix, "measure_resistance", rexjson::make_rpc_wrapper(this, &MotorDrive::RunResistanceMeasurement, "float ServoDrive::RunResistanceMeasurement(uint32_t seconds, float test_voltage)"));
	rpc_server.add(prefix, "measure_inductance", rexjson::make_rpc_wrapper(this, &MotorDrive::RunInductanceMeasurement, "float ServoDrive::RunInductanceMeasurement(uint32_t seconds, float test_voltage, uint32_t test_hz);"));
	rpc_server.add(prefix, "scheduler_run", rexjson::make_rpc_wrapper(this, &MotorDrive::SchedulerRun, "void SchedulerRun()"));
	rpc_server.add(prefix, "scheduler_abort", rexjson::make_rpc_wrapper(this, &MotorDrive::SchedulerAbort, "void SchedulerAbort()"));
	rpc_server.add(prefix, "add_task_arm_motor", rexjson::make_rpc_wrapper(this, &MotorDrive::AddTaskArmMotor, "void AddTaskArmMotor()"));
	rpc_server.add(prefix, "add_task_disarm_motor", rexjson::make_rpc_wrapper(this, &MotorDrive::AddTaskDisarmMotor, "void AddTaskDisarmMotor()"));
	rpc_server.add(prefix, "add_task_rotate_motor", rexjson::make_rpc_wrapper(this, &MotorDrive::AddTaskRotateMotor, "void AddTaskRotateMotor(float angle, float speed, float voltage, bool dir)"));
	rpc_server.add(prefix, "add_task_reset_rotor", rexjson::make_rpc_wrapper(this, &MotorDrive::AddTaskResetRotorWithParams, "void AddTaskResetRotorWithParams(float reset_voltage, uint32_t reset_hz, bool reset_encoder)"));
	rpc_server.add(prefix, "alpha_pole_search", rexjson::make_rpc_wrapper(this, &MotorDrive::RunTaskAlphaPoleSearch, "void RunTaskAlphaPoleSearch()"));
	rpc_server.add(prefix, "reset_rotor_hold", rexjson::make_rpc_wrapper(this, &MotorDrive::RunTaskResetRotorAndHold, "void RunTaskResetRotorAndHold()"));
	rpc_server.add(prefix, "rotate", rexjson::make_rpc_wrapper(this, &MotorDrive::RunTaskRotateMotor, "void RunTaskRotateMotor(float angle, float speed, float voltage, bool dir)"));
	rpc_server.add(prefix, "velocitypts", rexjson::make_rpc_wrapper(this, &MotorDrive::GetRotorVelocityPTS, "void GetRotorVelocityPTS()"));
	rpc_server.add(prefix, "set_resolution_bits", rexjson::make_rpc_wrapper(this, &MotorDrive::SetResolutionBits, "void SetResolutionBits(uint32_t resolution_bits)"));
	rpc_server.add(prefix, "run_encoder_debug", rexjson::make_rpc_wrapper(this, &MotorDrive::RunEncoderDisplayDebugInfo, "void RunEncoderDisplayDebugInfo()"));
	rpc_server.add(prefix, "drv_get_fault1", rexjson::make_rpc_wrapper(drv_, &Drv8323::GetFaultStatus1, "uint32_t Drv8323::GetFaultStatus1()"));
	rpc_server.add(prefix, "drv_get_fault2", rexjson::make_rpc_wrapper(drv_, &Drv8323::GetFaultStatus2, "uint32_t Drv8323::GetFaultStatus2()"));
	rpc_server.add(prefix, "drv_clear_fault", rexjson::make_rpc_wrapper(drv_, &Drv8323::ClearFault, "void Drv8323::ClearFault()"));

}

rexjson::property MotorDrive::GetPropertyMap()
{
	rexjson::property props= rexjson::property_map({
		{"Rencest", {&Rencest_, rexjson::property_get<decltype(Rencest_)>}},
		{"Renc", {&Renc_, rexjson::property_get<decltype(Renc_)>}},
		{"update_hz", rexjson::property(&update_hz_, rexjson::property_get<decltype(update_hz_)>)},
		{"tim1_cnt", rexjson::property(&tim1_cnt_, rexjson::property_get<decltype(tim1_cnt_)>)},
		{"tim8_cnt", rexjson::property(&tim8_cnt_, rexjson::property_get<decltype(tim8_cnt_)>)},
		{"tim1_tim8_offset", rexjson::property(&tim1_tim8_offset_, rexjson::property_get<decltype(tim1_tim8_offset_)>)},
		{"tim8_tim1_offset", rexjson::property(&tim8_tim1_offset_, rexjson::property_get<decltype(tim8_tim1_offset_)>)},
		{"TIM1_CNT", rexjson::property((void*)&TIM1->CNT, rexjson::property_get<decltype(TIM1->CNT)>)},
		{"TIM8_CNT", rexjson::property((void*)&TIM8->CNT, rexjson::property_get<decltype(TIM8->CNT)>)},
		{"time_slice", rexjson::property(&time_slice_, rexjson::property_get<decltype(time_slice_)>)},
		{"Vbus", rexjson::property(&lpf_vbus_.out_, rexjson::property_get<decltype(lpf_vbus_.out_)>)},
		{"error", rexjson::property(&error_info_.error_, rexjson::property_get<decltype(error_info_.error_)>)},
		{"error_msg", rexjson::property(&error_info_.error_msg_, rexjson::property_get<decltype(error_info_.error_msg_)>)},
		{"lpf_bias_a", rexjson::property(&lpf_bias_a.out_, rexjson::property_get<decltype(lpf_bias_a.out_)>)},
		{"lpf_bias_b", rexjson::property(&lpf_bias_b.out_, rexjson::property_get<decltype(lpf_bias_b.out_)>)},
		{"lpf_bias_c", rexjson::property(&lpf_bias_c.out_, rexjson::property_get<decltype(lpf_bias_c.out_)>)},
	});
	return props;
}


rexjson::property MotorDrive::GetConfigPropertyMap()
{
	rexjson::property props= rexjson::property_map({
		{"bias_alpha", rexjson::property(
			&config_.bias_alpha_,
			rexjson::property_get<decltype(config_.bias_alpha_)>,
			[&](const rexjson::value& v, void* ctx) {
				float t = v.get_real(); 
				if (t < 0 || t > 1.0) 
					throw std::range_error("Invalid value");
				rexjson::property_set<decltype(config_.bias_alpha_)>(v, ctx);
				lpf_bias_a.SetAlpha(config_.bias_alpha_);
				lpf_bias_b.SetAlpha(config_.bias_alpha_);
				lpf_bias_c.SetAlpha(config_.bias_alpha_);
			})},
		{"vbus_alpha", rexjson::property(
				&config_.vbus_alpha_,
				rexjson::property_get<decltype(config_.vbus_alpha_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					float t = v.get_real(); 
					if (t < 0 || t > 1.0) 
						throw std::range_error("Invalid value");
					rexjson::property_set<decltype(config_.vbus_alpha_)>(v, ctx);
					lpf_vbus_.SetAlpha(config_.vbus_alpha_);
				})},

		{"wenc_alpha", rexjson::property(
				&config_.wenc_alpha_,
				rexjson::property_get<decltype(config_.vbus_alpha_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					float t = v.get_real();
					if (t < 0 || t > 1.0) 
						throw std::range_error("Invalid value");
					rexjson::property_set<decltype(config_.vbus_alpha_)>(v, ctx);
					lpf_Wenc_.SetAlpha(config_.wenc_alpha_);
				})},

		{"enc_skip_updates", rexjson::property(
				&config_.enc_skip_updates_,
				rexjson::property_get<decltype(config_.enc_skip_updates_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					pwm_->Stop();
					update_counter_ = 0;
					rexjson::property_set<decltype(config_.enc_skip_updates_)>(v, ctx);
					HAL_Delay(2);
					pwm_->Start();
				})},
		{"max_modulation_duty", rexjson::property(
				&config_.max_modulation_duty_,
				rexjson::property_get<decltype(config_.max_modulation_duty_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					float t = v.get_real(); 
					if (t < 0 || t > 1.0) 
						throw std::range_error("Invalid value");
					rexjson::property_set<decltype(config_.max_modulation_duty_)>(v, ctx);
				})},
		{"csa_gain", rexjson::property(
				&config_.csa_gain_,
				rexjson::property_get<decltype(config_.csa_gain_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					int t = v.get_int(); 
					if (t != 5 && t != 10 && t != 20 && t != 40) 
						throw std::range_error("Invalid value");
					rexjson::property_set<decltype(config_.csa_gain_)>(v, ctx);
					drv_->SetCSAGainValue(config_.csa_gain_);
				})},
		{"run_simple_tasks", rexjson::property(
				&run_simple_tasks_,
				rexjson::property_get<decltype(run_simple_tasks_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					rexjson::property_set<decltype(run_simple_tasks_)>(v, ctx);
					if (run_simple_tasks_) 
						RunSimpleTasks(); 
					else 
						sched_.Abort();
				})},
		{"pos_offset", rexjson::property(
				&config_.pos_offset_,
				rexjson::property_get<decltype(config_.pos_offset_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					if (sched_.IsDispatching())
						throw std::runtime_error("Cannot set the value when the drive is running");
					rexjson::property_set<decltype(config_.pos_offset_)>(v, ctx);
				})},
		{"enc_position_shiftleft", {&enc_position_shiftleft_, rexjson::property_get<decltype(enc_position_shiftleft_)>, rexjson::property_set<decltype(enc_position_shiftleft_)>}},
		{"enc_position_shiftright", {&enc_position_shiftright_, rexjson::property_get<decltype(enc_position_shiftright_)>, rexjson::property_set<decltype(enc_position_shiftright_)>}},
		{"trip_i", {&config_.trip_i_, rexjson::property_get<decltype(config_.trip_i_)>, rexjson::property_set<decltype(config_.trip_i_)>}},
		{"trip_v", {&config_.trip_v_, rexjson::property_get<decltype(config_.trip_v_)>, rexjson::property_set<decltype(config_.trip_v_)>}},
		{"calib_max_i", {&config_.calib_max_i_, rexjson::property_get<decltype(config_.calib_max_i_)>, rexjson::property_set<decltype(config_.calib_max_i_)>}},
		{"calib_v", {&config_.calib_v_, rexjson::property_get<decltype(config_.calib_v_)>, rexjson::property_set<decltype(config_.calib_v_)>}},
		{"resistance", {&config_.resistance_, rexjson::property_get<decltype(config_.resistance_)>, rexjson::property_set<decltype(config_.resistance_)>}},
		{"inductance", {&config_.inductance_, rexjson::property_get<decltype(config_.inductance_)>, rexjson::property_set<decltype(config_.inductance_)>}},
		{"pole_pairs", {&config_.pole_pairs_, rexjson::property_get<decltype(config_.pole_pairs_)>, rexjson::property_set<decltype(config_.pole_pairs_)>}},
		{"pole_offset", {&config_.pole_offset_, rexjson::property_get<decltype(config_.pole_offset_)>, rexjson::property_set<decltype(config_.pole_offset_)>}},
		{"encoder_dir", {&config_.encoder_dir_, rexjson::property_get<decltype(config_.encoder_dir_)>, rexjson::property_set<decltype(config_.encoder_dir_)>}},
		{"svm_saddle", {&config_.svm_saddle_, rexjson::property_get<decltype(config_.svm_saddle_)>, rexjson::property_set<decltype(config_.svm_saddle_)>}},
		{"reset_voltage", {&config_.reset_voltage_, rexjson::property_get<decltype(config_.reset_voltage_)>, rexjson::property_set<decltype(config_.reset_voltage_)>}},
		{"reset_hz", {&config_.reset_hz_, rexjson::property_get<decltype(config_.reset_hz_)>, rexjson::property_set<decltype(config_.reset_hz_)>}},
		{"display_div", {&config_.display_div_, rexjson::property_get<decltype(config_.display_div_)>, rexjson::property_set<decltype(config_.display_div_)>}},
	});
	return props;
}


/** Clear any outstanding error and put the
 * scheduler in run mode.
 *
 * If there are any tasks loaded in the scheduler queue,
 * they will start executing immediately.
 */
void MotorDrive::Run()
{
	error_info_.ClearError();
	sched_.Run();
}

/** This method is similar to @ref MotorDrive::Run,
 * but it will block until all tasks finish.
 *
 */
void MotorDrive::RunWaitForCompletion()
{
	error_info_.ClearError();
	sched_.RunWaitForCompletion();
}

/** Immediately cut the PWM outputs and abort
 * all outstanding tasks in the scheduler queue.
 *
 */
void MotorDrive::Abort()
{
	pwm_->Stop();
	sched_.Abort();
}

/** Return pointer to the currently used encoder object
 *
 * @return IEncoder pointer interface
 */
IEncoder* MotorDrive::GetEncoder() const
{
	return encoder_;
}

/** Return the encoder Counts Per Revolution  (CPR)
 *
 * @return Encoder CPR
 */
uint32_t MotorDrive::GetEncoderCPR() const
{
	return enc_cpr_;
}

/** Get the total number of bits used by @ref GetEncoderPosition,
 * this is encoder revolution bits + encoder resolution bits
 *
 * @return Max number of bits used by encoder position value
 * @note this value might not correspond to the physical encoder
 * bits if the enc_position_shift_ is more than 0, i.e. the encoder
 * position is scaled back.
 */
uint32_t MotorDrive::GetEncoderPositionBits() const
{
	return enc_revolution_bits_ + enc_resolution_bits_;
}

/** Get the current encoder position
 *
 * @return Encoder Position
 */
uint64_t MotorDrive::GetEncoderPosition() const
{
	return ((encoder_->GetPosition() << enc_position_shiftleft_) >> enc_position_shiftright_);
}

/** Get the current encoder position
 *
 * @return Encoder Max Position
 */
uint64_t MotorDrive::GetEncoderMaxPosition() const
{
	return enc_position_size_;
}


/** Set the IEncoder interface
 *
 * @param encoder IEncoder interface
 */
void MotorDrive::SetEncoder(IEncoder *encoder)
{
	if (encoder) {
		if (encoder->GetStatus()) {
			error_info_.SetError(e_encoder, "Encoder Error");
		}
		encoder_ = encoder;
		SetResolutionBits();
	}
}

/**
 * @brief Set the desired resolution bits. The number of resolution bits
 * defines the encoder counts corresponding to one full revolution
 * CPR = 1 << resolution_bits
 * 
 * @param resolution_bits Desired resolution bits. If the encoder hardware uses more bits the
 * encoder position will be shifted right to match the desired resolution. If the encoder
 * hardware uses less bits the encoder position value will be shifted left.
 */
void MotorDrive::SetResolutionBits(uint32_t resolution_bits)
{
	enc_revolution_bits_ = encoder_->GetRevolutionBits();
	enc_resolution_bits_ = resolution_bits;
	if (resolution_bits <= encoder_->GetResolutionBits()) {
		enc_position_shiftright_ = encoder_->GetResolutionBits() - resolution_bits;
		enc_position_shiftleft_ = 0;
	} else {
		enc_position_shiftleft_ = encoder_->GetResolutionBits() - resolution_bits;
		enc_position_shiftright_ = 0;
	}
	enc_cpr_ = (1 << enc_resolution_bits_);
	enc_resolution_mask_ = (1 << enc_resolution_bits_) - 1;
	enc_position_size_ = 1LLU << (enc_resolution_bits_ + enc_revolution_bits_);
	enc_position_size_half_ = enc_position_size_ / 2;
	enc_position_mask_ = enc_position_size_ - 1;
}


/** Get the configure encoder direction.
 *
 * The configured encoder direction allows the direction
 * of encoder rotation and the positive direction of the
 * rotor rotation for the current current wiring to be
 * synchronized. It is also possible the direction synchronization
 * to be achieved by swapping two of the cables connected to the
 * motor.
 *
 * @return 1 - encoder increasing or -1 encoder decreasing,
 * 0 the encoder direction is not set. You need this parameter to be
 * set to 1 or -1 (depending on how the cables are connected) for
 * the motor operate correctly.
 */
int32_t MotorDrive::GetEncoderDir() const
{
	return config_.encoder_dir_;
}

/** Return the update rate of the motor drive
 *
 * @return Update rate
 */
uint32_t MotorDrive::GetUpdateFrequency() const
{
	return update_hz_;
}

/** Return the update time period in seconds
 *
 * @return
 */
float MotorDrive::GetTimeSlice() const
{
	return time_slice_;
}

/** Get the motor pole pairs.
 *
 * @return motor pole pairs
 */
uint32_t MotorDrive::GetPolePairs() const
{
	return config_.pole_pairs_;
}

/** Get the Vbus voltage after it has been
 * run through a low pass filter.
 *
 * @return Filtered Vbus voltage
 */
float MotorDrive::GetBusVoltage() const
{
	return lpf_vbus_.Output();
}

/** Get the phase current as a vector (complex value).
 *
 * @return complex valule representing the magnitude
 * and the direction of the phase current
 */
std::complex<float> MotorDrive::GetPhaseCurrent() const
{
	return Iab_;
}

/**
 * @brief Get the magnetude of the phase current.
 * 
 * @return float Magnetude of the phase current
 */
float MotorDrive::GetPhaseCurrentMagnetude() const
{
	return Iab_m_;
}

/** Check if the PWM timer counter is currently enabled.
 *
 * If the PWM timer is enabled it will trigger the interrupts
 * and the @ref IrqUpdateCallback will be executed.
 *
 * @note This doesn't mean the outputs are enabled though.
 * @return true if the PWM timer is enabled, false otherwise
 */
bool MotorDrive::IsStarted()
{
	return pwm_->IsStarted();
}

/** 
 *
 * We use center aligned PWM. When the counter is going up the
 * current through the windings should be zero at that point
 * we update the bias. When the counter is going down we update
 * the current.
 * ```
 *   |       _____|_____       |
 *   |      |     |     |      |
 *   |______|     |     |______|
 *   |            |            |
 *   /\     up    /\     down
 *    \            \
 *     \            \___ update current
 *      \
 *       \___update bias
 *
 * ```
 */


void MotorDrive::UpdateBias()
{
//    tim1_cnt_ = TIM1->CNT;
//    tim8_cnt_ = TIM8->CNT;
//    tim1_tim8_offset_ = (int32_t)TIM8->CNT - (int32_t)TIM1->CNT;
//    tim8_tim1_offset_ = (int32_t)TIM1->CNT - (int32_t)TIM8->CNT;

	/*
	 * Sample ADC bias
	 */
	float injdata_a = (int32_t) adc_->InjReadConversionData(LL_ADC_INJ_RANK_1);
	float injdata_b = (int32_t) adc_->InjReadConversionData(LL_ADC_INJ_RANK_2);
	float injdata_c = (int32_t) adc_->InjReadConversionData(LL_ADC_INJ_RANK_3);

	lpf_bias_a.DoFilter(injdata_a);
	lpf_bias_b.DoFilter(injdata_b);
	lpf_bias_c.DoFilter(injdata_c);
}

void MotorDrive::UpdateCurrent()
{
	update_counter_++;
	if (update_counter_ % (config_.enc_skip_updates_ + 1) == 0) {
		encoder_->Update();
		UpdateRotor();
	} else {
		EstimateRotor();
	}

	/*
	 * Sample VBus voltage
	 */
#if 1
	float vbus = dma_adc_->RegReadConversionData(4 - 1);
	lpf_vbus_.DoFilter(__LL_ADC_CALC_DATA_TO_VOLTAGE(config_.Vref_, vbus, LL_ADC_RESOLUTION_12B) * config_.Vbus_resistor_ratio_);
#else

	float injdata_vbus = (int32_t) adc_->InjReadConversionData(LL_ADC_INJ_RANK_4);
	lpf_vbus_.DoFilter(__LL_ADC_CALC_DATA_TO_VOLTAGE(config_.Vref_, injdata_vbus, LL_ADC_RESOLUTION_12B) * config_.Vbus_resistor_ratio_);

#endif

	/*
	 * Sample ADC phase current
	 */
	float injdata_a = (int32_t) adc_->InjReadConversionData(LL_ADC_INJ_RANK_1);
	float injdata_b = (int32_t) adc_->InjReadConversionData(LL_ADC_INJ_RANK_2);
	float injdata_c = (int32_t) adc_->InjReadConversionData(LL_ADC_INJ_RANK_3);

	/*
	 * Apply the ADC bias to the current data
	 */
	phase_current_a_ = CalculatePhaseCurrent(injdata_a, lpf_bias_a.Output());
	phase_current_b_ = CalculatePhaseCurrent(injdata_b, lpf_bias_b.Output());
	phase_current_c_ = CalculatePhaseCurrent(injdata_c, lpf_bias_c.Output());

	/*
	 * Update the Iab vector
	 */
	float Ia = phase_current_a_;
	float Ib = phase_current_b_;
	float Ic = - phase_current_a_ - phase_current_b_;
	Iab_ = Pa_ * Ia + Pb_ * Ib + Pc_ * Ic;
	Iab_m_ = std::abs(Iab_);

	/*
	 * Check for abnormal conditions.
	 */
	AbortOnBusVoltageViolation(lpf_vbus_.Output());
	AbortOnPhaseCurrentViolation(Iab_m_);
	/*
	 * Run the scheduler tasks
	 */
	sched_.OnUpdate();
}

/** 
 * Update the rotor position and velocity
 *
 */
void MotorDrive::UpdateRotor()
{
	uint64_t Renc_prev = Renc_; 
	Renc_ = GetEncoderPosition();

	int32_t Rangle_prev = Renc_prev & enc_resolution_mask_;
	int32_t Rangle = Renc_ & enc_resolution_mask_;
	int32_t Wenc = (Rangle + enc_cpr_ - Rangle_prev) % enc_cpr_;
	if (Wenc > (int32_t)(enc_cpr_ / 2))
		Wenc -= enc_cpr_;
	lpf_Wenc_.DoFilter(((float)Wenc) / (config_.enc_skip_updates_ + 1));

	Rencest_ = (Renc_ + config_.pos_offset_) & enc_position_mask_;
	Rencpred_ = (Renc_ + (uint64_t)lpf_Wenc_.Output()) & enc_position_mask_;
	float theta_e = GetEncoderDir() * GetElectricAngle(Rencpred_ - config_.pole_offset_);
	float cos_theta = arm_cos_f32(theta_e);
	float sin_theta = arm_sin_f32(theta_e);
	E_ = std::complex<float>(cos_theta, sin_theta);
}

/** 
 * Estimate the rotor position
 *
 */
void MotorDrive::EstimateRotor()
{
	Rencest_ = (Rencpred_ + config_.pos_offset_) & enc_position_mask_;
}

/** Return the current rotor position in encoder counts
 *
 * @return rotor position
 */
uint64_t MotorDrive::GetRotorPosition() const
{
	return Rencest_;
}

/** Calculate the position error. Trim the error within [-maxerr, maxerr]
 *
 * @param position
 * @param target
 * @return
 */
int64_t MotorDrive::GetRotorPositionError(uint64_t position, uint64_t target)
{
	int64_t position_err = (target + enc_position_size_ - position) & enc_position_mask_;
	if (position_err > (int64_t)(enc_position_size_half_))
		position_err -= enc_position_size_;
	return position_err;
}


/** Calculate the angle of the rotor from the
 * encoder position in electrical radians.
 *
 * @param enc_position Encoder position
 * @return The orientation of the rotor in electrical radians
 */
float MotorDrive::GetElectricAngle(uint64_t enc_position) const
{
	uint32_t enc_orientation = enc_position & enc_resolution_mask_;
	uint32_t cpr_per_pair = (enc_cpr_ / config_.pole_pairs_);
	return (2.0f * M_PI / cpr_per_pair) * (enc_orientation % cpr_per_pair);
}

/** Calculate the angle of the rotor from the encoder position in mechanical radians.
 *
 * @param enc_position Encoder position
 * @return The orientation of the rotor in mechanical radians
 */
float MotorDrive::GetMechanicalAngle(uint64_t enc_position) const
{
	return (2.0f * M_PI / enc_cpr_) * (enc_position & enc_resolution_mask_);
}

std::complex<float> MotorDrive::GetRotorElecRotation()
{
	return E_;
}

/** Get the rotor velocity in encoder counts per second[counts/s]
 *
 * @return Rotor velocity
 */
float MotorDrive::GetRotorVelocity()
{
	return GetRotorVelocityPTS() * update_hz_;
}


/** Get the rotor velocity in encoder counts per time slice
 *
 * @return Mechanical rotor velocity
 */
float MotorDrive::GetRotorVelocityPTS()
{
	return lpf_Wenc_.Output();
}

/** Get the rotor velocity in elec encoder counts per time_slice
 *
 * @return Electrical rotor velocity
 */
float MotorDrive::GetRotorElecVelocityPTS()
{
	return GetRotorVelocityPTS() * config_.pole_pairs_;
}

float MotorDrive::CalculatePhaseCurrent(float adc_val, float adc_bias)
{
	return ((adc_bias - adc_val) * config_.Vref_ / config_.adc_full_scale_) / config_.Rsense_ / config_.csa_gain_;
}

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

bool MotorDrive::ApplyPhaseVoltage(float v_alpha, float v_beta)
{
	float v_abs = 0.0f;
	arm_sqrt_f32(v_alpha * v_alpha + v_beta * v_beta, &v_abs);
	return ApplyPhaseModulation(VoltageToDuty(v_abs, GetBusVoltage()), std::complex<float>(v_alpha/v_abs, v_beta/v_abs));
}

bool MotorDrive::ApplyPhaseVoltage(float v_abs, const std::complex<float>& v_theta)
{
	return ApplyPhaseModulation(VoltageToDuty(v_abs, GetBusVoltage()), v_theta);
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
		ApplyPhaseVoltage(0, std::polar<float>(1.0f, 0.0f));
		pwm_->Start();
	});
}

void MotorDrive::AddTaskDisarmMotor()
{
	sched_.AddTask([&](){
		ApplyPhaseVoltage(0, std::polar<float>(0.0f, 0.0f));
		pwm_->Stop();
	});
}

bool MotorDrive::RunUpdateHandlerRotateMotor(float angle, float speed, float voltage, bool dir)
{
	float el_speed = speed * config_.pole_pairs_;
	float total_rotation = angle * config_.pole_pairs_;
	float total_time = total_rotation / el_speed;
	uint32_t total_cycles = update_hz_ * total_time;
	std::complex<float> step = std::polar<float>(1.0, total_rotation/total_cycles);
	std::complex<float> rotation = std::polar<float>(1.0, 0);
	bool ret = true;

	if (!dir)
		step = std::conj(step);
	for (size_t i = 0; ret && i < total_cycles; i++) {
		ret = sched_.RunUpdateHandler([&]()->bool {
			ApplyPhaseVoltage(voltage, rotation);
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

void MotorDrive::AddTaskResetRotorWithParams(float reset_voltage, uint32_t reset_hz, bool reset_encoder)
{
	sched_.AddTask(std::bind([&](float reset_voltage, uint32_t reset_hz, bool reset_encoder){
		bool ret = true;
		uint32_t set_angle_cycles = update_hz_ / reset_hz;
		for (float angle = M_PI_4; angle > 0.1; angle = angle * 3 / 4) {
			for (size_t i = 0; (i < set_angle_cycles) && ret; i++) {
				ret = sched_.RunUpdateHandler([&]()->bool {
					ApplyPhaseVoltage(reset_voltage, std::polar<float>(1.0f, angle));
					return false;
				});
			};
			for (size_t i = 0; (i < set_angle_cycles) && ret; i++) {
				ret = sched_.RunUpdateHandler([&]()->bool {
					ApplyPhaseVoltage(reset_voltage, std::polar<float>(1.0f, -angle));
					return false;
				});
			};
		}
		for (size_t i = 0; (i < update_hz_) && ret; i++) {
			ret = sched_.RunUpdateHandler([&]()->bool {
				ApplyPhaseVoltage(reset_voltage * 1.5, std::polar<float>(1.0f, 0));
				return false;
			});
		};
		if (reset_encoder)
			encoder_->ResetPosition();
		config_.pole_offset_ = (GetEncoderPosition() % enc_cpr_) % (enc_cpr_ / config_.pole_pairs_);
		if (config_.pole_offset_ > (int32_t)(enc_cpr_ / config_.pole_pairs_ / 2))
			config_.pole_offset_ -= enc_cpr_ / config_.pole_pairs_;
	}, reset_voltage, reset_hz, reset_encoder));
}


void MotorDrive::RunEncoderDisplayDebugInfo()
{
	sched_.AddTask([&](){

		sched_.RunUpdateHandler([&]()->bool {
			if ((update_counter_ % (update_hz_ >> 2)) == 0)
				encoder_->DisplayDebugInfo();
			return true;
		});

	});

	Run();
}


void MotorDrive::AddTaskDetectEncoderDir()
{
	sched_.AddTask([&](){
		uint64_t enc1 = GetEncoderPosition();
		if (RunUpdateHandlerRotateMotor(M_PI_2, M_PI, config_.reset_voltage_, true)) {
			uint64_t enc2 = GetEncoderPosition();
			config_.encoder_dir_  = (enc2 > enc1) ? 1 : -1;
//			config_.encoder_dir_ = (GetMechanicalAngle(GetEncoderPosition()) > M_PI) ? -1 : 1;
		}
	});
	sched_.AddTask([&](){
		RunUpdateHandlerRotateMotor(M_PI_2, M_PI, config_.reset_voltage_, false);
	});


}

void MotorDrive::AddTaskCalibrationSequence(bool reset_rotor)
{
	AddTaskArmMotor();
	AddTaskResetRotorWithParams(config_.reset_voltage_, config_.reset_hz_, reset_rotor);
	AddTaskMeasureResistance(1, config_.calib_v_);
	AddTaskMeasureInductance(1, config_.calib_v_, 3000);
	AddTaskDetectEncoderDir();
	AddTaskResetRotorWithParams(config_.reset_voltage_, config_.reset_hz_, false);
	AddTaskDisarmMotor();
}

void MotorDrive::RunTaskResetRotorAndHold()
{
	AddTaskArmMotor();
	AddTaskResetRotorWithParams(config_.reset_voltage_, config_.reset_hz_, false);
	sched_.AddTask([&](void){
			sched_.RunUpdateHandler([&]()->bool {
				ApplyPhaseVoltage(config_.reset_voltage_, std::polar<float>(1, 0));
				return true;
			});
	});
	sched_.Run();
}

void MotorDrive::RunTaskAlphaPoleSearch()
{
	AddTaskArmMotor();
	if (config_.encoder_dir_ == 0) {
		AddTaskResetRotorWithParams(config_.reset_voltage_, config_.reset_hz_);
		AddTaskDetectEncoderDir();
	}
	AddTaskResetRotorWithParams(config_.reset_voltage_, config_.reset_hz_);
	for (size_t i = 0; i < config_.pole_pairs_; i++) {
		AddTaskRotateMotor((M_PI * 2)/(config_.pole_pairs_ + 1), M_PI, 0.45, true);
		AddTaskResetRotorWithParams(config_.reset_voltage_, config_.reset_hz_, false);
		sched_.AddTask([&](void){
			fprintf(stderr, "Enc: %7lu\r\n", (uint32_t)(GetEncoderPosition() & enc_resolution_bits_));
			encoder_->ResetPosition();
		});
	}
	AddTaskDisarmMotor();
	sched_.Run();
}

void MotorDrive::AddTaskMeasureResistance(float seconds, float test_voltage)
{
	sched_.AddTask(std::bind([&](float seconds, float test_voltage){
		uint32_t i = 0;
		uint32_t test_cycles = update_hz_ * seconds;
		bool ret = false;

		ApplyPhaseVoltage(0, 0);
		config_.resistance_ = 0;
		do {
			ret = sched_.RunUpdateHandler([&]()->bool {
				ApplyPhaseVoltage(test_voltage, std::polar<float>(1, 0));
				return false;
			});

#if 0
			if ((data_.update_counter_ % 13) == 0) {
				fprintf(stderr, "Vbus: %4.2f, Ia: %6.3f, Ib: %6.3f, Ic: %6.3f, Ia+Ib+Ic: %6.3f\r\n",
						lpf_vbus_.Output(), data_.phase_current_a_, data_.phase_current_b_, data_.phase_current_c_,
						data_.phase_current_a_ + data_.phase_current_b_ + data_.phase_current_c_);
			}
#endif
		} while (ret && i++ < test_cycles);
		if (ret)
			config_.resistance_ = 2.0f / 3.0f * test_voltage / phase_current_a_;
		ApplyPhaseVoltage(0, 0);
	}, seconds, test_voltage));
}

void MotorDrive::AddTaskMeasureInductance(float seconds, float test_voltage, uint32_t test_hz)
{
	sched_.AddTask(std::bind([&](float seconds, float test_voltage, uint32_t test_hz){
		LowPassFilter<float, float> lpf_a(0.001);
		uint32_t sine_steps = update_hz_ / test_hz;
		uint32_t test_cycles = update_hz_ * seconds;
		bool ret = true;
		std::complex<float> V(1.0, 0.0);
		std::complex<float> r = std::polar<float>(1.0, M_PI * 2 / sine_steps);

		ApplyPhaseVoltage(0, 0);
		for (uint32_t i = 0; ret && i < test_cycles;  i++) {

			ret = sched_.RunUpdateHandler([&]()->bool {

				lpf_a.DoFilter(std::abs(phase_current_a_));
				V = V * r;

				// Test voltage along phase A
				ApplyPhaseVoltage(test_voltage * V.real(), std::complex<float>(1.0f, 0.0f));
				return false;
			});
#if 0
			if ((i % 13) == 0) {
				fprintf(stderr, "Vbus: %4.2f, Ia: %6.3f\r\n",
						lpf_vbus_.Output(), lpf_a.Output());
			}
#endif
		}

		if (ret) {
			float X = (lpf_vbus_.Output() - lpf_a.Output() * config_.resistance_)/ lpf_a.Output();
			float L = X / (2 * M_PI * test_hz);
			config_.inductance_ = L;
		}
	}, seconds, test_voltage, test_hz));
}


float MotorDrive::RunResistanceMeasurement(float seconds, float test_voltage)
{
	AddTaskArmMotor();
	AddTaskMeasureResistance(seconds, test_voltage);
	AddTaskDisarmMotor();
	sched_.RunWaitForCompletion();
	return config_.resistance_;
}

float MotorDrive::RunInductanceMeasurement(float seconds, float test_voltage, uint32_t test_hz)
{
	AddTaskArmMotor();
	AddTaskMeasureInductance(seconds, test_voltage, test_hz);
	AddTaskDisarmMotor();
	sched_.RunWaitForCompletion();
	return config_.inductance_;
}

/** Rotate the rotor the using the specified angle, speed, voltage and direction.
 *
 * This method creates a task to rotate the rotor. Before the actual rotation starts
 * the rotor will be armed and reset to against the A phase. The reset is done using
 * the configuration parameteres reset voltage: config_.reset_voltage_ and the
 * reset oscilation rate config_.reset_hz_.
 *
 * @param angle
 * @param speed
 * @param voltage
 * @param dir
 */
void MotorDrive::RunTaskRotateMotor(float angle, float speed, float voltage, bool dir)
{
	AddTaskArmMotor();
	AddTaskResetRotorWithParams(config_.reset_voltage_, config_.reset_hz_, false);
	AddTaskRotateMotor(angle, speed, voltage, dir);
	AddTaskDisarmMotor();
	sched_.RunWaitForCompletion();
}

void MotorDrive::DefaultIdleTask()
{
	fprintf(stderr, "VBus: %+5.2f, Bias: %+5.2f, %+5.2f, Currents: %+5.2f, %+5.2f, %+5.2f\r\n", lpf_vbus_.Output(),
			lpf_bias_a.Output(), lpf_bias_b.Output(), phase_current_a_, phase_current_b_,
			- phase_current_a_ - phase_current_b_);
}

bool MotorDrive::AbortOnPhaseCurrentViolation(float current)
{
	if (current > config_.trip_i_) {
		Abort();
		error_info_.SetError(e_trip_voltage, "Current limit violation, %f", current);
		return true;
	}
	return false;
}

bool MotorDrive::AbortOnBusVoltageViolation(float voltage)
{
	if (voltage > config_.trip_v_) {
		Abort();
		error_info_.SetError(e_trip_voltage, "Voltage bus limit violation, %f", voltage);
		return true;
	}
	return false;
}

void MotorDrive::RunSimpleTasks()
{
	sched_.AddTask([&](){
		uint32_t t0 = xTaskGetTickCount();
		if (sched_.WaitSignals(Scheduler::THREAD_FLAG_ABORT, 2000) == Scheduler::THREAD_FLAG_ABORT) {
			fprintf(stderr, "Task1 Aborting...\r\n\r\n\r\n");
			return;
		}
		fprintf(stderr, "Task1 finished %lu\n", xTaskGetTickCount() - t0);
	});
	sched_.AddTask([&](){
		uint32_t t0 = xTaskGetTickCount();
		if (sched_.WaitSignalAbort(2000)) {
			fprintf(stderr, "Task2 Aborting...\r\n\r\n\r\n");
			return;
		}
		fprintf(stderr, "Task2 finished %lu\r\n", xTaskGetTickCount() - t0);
	});
	sched_.AddTask([&](){
		uint32_t t0 = xTaskGetTickCount();
		if (sched_.WaitSignalAbort(2000)) {
			fprintf(stderr, "Task3 Aborting...\r\n\r\n\r\n");
			return;
		}
		fprintf(stderr, "Task3 finished %lu\r\n\r\n\r\n", xTaskGetTickCount() - t0);
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

