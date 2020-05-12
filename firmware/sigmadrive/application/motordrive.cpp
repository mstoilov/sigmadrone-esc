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
#include "adcdata.h"

extern UartRpcServer rpc_server;
extern Adc adc1;
extern Adc adc2;
extern Adc adc3;
extern Drv8323 drv1;


MotorDrive::MotorDrive(IEncoder* encoder, IPwmGenerator *pwm, uint32_t update_hz)
    : Pa_(std::polar<float>(1.0f, 0.0f))
    , Pb_(std::polar<float>(1.0f, M_PI / 3.0 * 2.0 ))
    , Pc_(std::polar<float>(1.0f, M_PI / 3.0 * 4.0))
    , update_hz_(update_hz)
    , time_slice_(1.0f / update_hz_)
    , enc_update_hz_(update_hz / (config_.enc_skip_updates_ + 1))
    , enc_time_slice_(1.0f / enc_update_hz_)
    , lpf_bias_a(config_.bias_alpha_)
    , lpf_bias_b(config_.bias_alpha_)
    , lpf_bias_c(config_.bias_alpha_)
    , lpf_vbus_(config_.vbus_alpha_, 12.0f)
    , lpf_Ia_(config_.iabc_alpha_)
    , lpf_Ib_(config_.iabc_alpha_)
    , lpf_Ic_(config_.iabc_alpha_)
    , lpf_Wenc_(config_.wenc_alpha_)
{
    lpf_bias_a.Reset(1 << 11);
    lpf_bias_b.Reset(1 << 11);
    lpf_bias_c.Reset(1 << 11);
    encoder_ = encoder;
    pwm_ = pwm;

    sched_.SetAbortTask([&]() -> void {
        pwm_->Stop();
    });

    sched_.SetIdleTask([&]() -> void {
//		DefaultIdleTask();
    });

    RegisterRpcMethods();
}

MotorDrive::~MotorDrive()
{
}

void MotorDrive::Attach()
{
    drv1.SetCSAGainValue(config_.csa_gain_);
    sched_.StartDispatcherThread();
}

void MotorDrive::RegisterRpcMethods()
{
    rpc_server.add("drive.abort", rexjson::make_rpc_wrapper(this, &MotorDrive::Abort, "void ServoDrive::Abort()"));
    rpc_server.add("drive.measure_resistance", rexjson::make_rpc_wrapper(this, &MotorDrive::RunResistanceMeasurement, "float ServoDrive::RunResistanceMeasurement(uint32_t seconds, float test_voltage)"));
    rpc_server.add("drive.measure_inductance", rexjson::make_rpc_wrapper(this, &MotorDrive::RunInductanceMeasurement, "float ServoDrive::RunInductanceMeasurement(uint32_t seconds, float test_voltage, uint32_t test_hz);"));
    rpc_server.add("drive.scheduler_run", rexjson::make_rpc_wrapper(this, &MotorDrive::SchedulerRun, "void SchedulerRun()"));
    rpc_server.add("drive.scheduler_abort", rexjson::make_rpc_wrapper(this, &MotorDrive::SchedulerAbort, "void SchedulerAbort()"));
    rpc_server.add("drive.add_task_arm_motor", rexjson::make_rpc_wrapper(this, &MotorDrive::AddTaskArmMotor, "void AddTaskArmMotor()"));
    rpc_server.add("drive.add_task_disarm_motor", rexjson::make_rpc_wrapper(this, &MotorDrive::AddTaskDisarmMotor, "void AddTaskDisarmMotor()"));
    rpc_server.add("drive.add_task_rotate_motor", rexjson::make_rpc_wrapper(this, &MotorDrive::AddTaskRotateMotor, "void AddTaskRotateMotor(float angle, float speed, float voltage, bool dir)"));
    rpc_server.add("drive.add_task_reset_rotor", rexjson::make_rpc_wrapper(this, &MotorDrive::AddTaskResetRotorWithParams, "void AddTaskResetRotorWithParams(float reset_voltage, uint32_t reset_hz)"));
    rpc_server.add("drive.alpha_pole_search", rexjson::make_rpc_wrapper(this, &MotorDrive::RunTaskAlphaPoleSearch, "void RunTaskAlphaPoleSearch()"));
    rpc_server.add("drive.rotate", rexjson::make_rpc_wrapper(this, &MotorDrive::RunTaskRotateMotor, "void RunTaskRotateMotor(float angle, float speed, float voltage, bool dir)"));
    rpc_server.add("drive.drv_get_fault1", rexjson::make_rpc_wrapper(&drv1, &Drv8323::GetFaultStatus1, "uint32_t Drv8323::GetFaultStatus1()"));
    rpc_server.add("drive.drv_get_fault2", rexjson::make_rpc_wrapper(&drv1, &Drv8323::GetFaultStatus2, "uint32_t Drv8323::GetFaultStatus2()"));
    rpc_server.add("drive.drv_clear_fault", rexjson::make_rpc_wrapper(&drv1, &Drv8323::ClearFault, "void Drv8323::ClearFault()"));
}

rexjson::property MotorDrive::GetPropertyMap()
{
    rexjson::property props= rexjson::property_map({
        {"update_hz", rexjson::property(&update_hz_, rexjson::property_access::readonly)},
        {"error", rexjson::property(&error_info_.error_, rexjson::property_access::readonly)},
        {"error_msg", rexjson::property(&error_info_.error_msg_, rexjson::property_access::readonly)},
        {"lpf_bias_a", rexjson::property(&lpf_bias_a.out_, rexjson::property_access::readonly)},
        {"lpf_bias_b", rexjson::property(&lpf_bias_b.out_, rexjson::property_access::readonly)},
        {"lpf_bias_c", rexjson::property(&lpf_bias_c.out_, rexjson::property_access::readonly)},
        {"bias_alpha", rexjson::property(
            &config_.bias_alpha_,
            rexjson::property_access::readwrite,
            [](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
            [&](void*)->void {
                lpf_bias_a.SetAlpha(config_.bias_alpha_);
                lpf_bias_b.SetAlpha(config_.bias_alpha_);
                lpf_bias_c.SetAlpha(config_.bias_alpha_);
            })},
        {"vbus_alpha", rexjson::property(
                &config_.vbus_alpha_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
                [&](void*)->void {
                    lpf_vbus_.SetAlpha(config_.vbus_alpha_);
                })},
        {"wenc_alpha", rexjson::property(
                &config_.wenc_alpha_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
                [&](void*)->void {
                    lpf_Wenc_.SetAlpha(config_.wenc_alpha_);
                })},
        {"enc_skip_updates", rexjson::property(
                &config_.enc_skip_updates_,
                rexjson::property_access::readwrite,
                [&](const rexjson::value& v){ pwm_->Stop(); },
                [&](void*)->void {
                    enc_update_hz_ = update_hz_ / (config_.enc_skip_updates_ + 1);
                    enc_time_slice_ = (1.0f / (update_hz_ / enc_update_hz_));
                    data_.update_counter_ = 0;
                    HAL_Delay(2);
                    pwm_->Start();
                })},

        {"iabc_alpha", rexjson::property(
                &config_.iabc_alpha_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
                [&](void*)->void {
                    lpf_Ia_.SetAlpha(config_.iabc_alpha_);
                    lpf_Ib_.SetAlpha(config_.iabc_alpha_);
                    lpf_Ic_.SetAlpha(config_.iabc_alpha_);
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
        {"trip_i", &config_.trip_i_},
        {"trip_v", &config_.trip_v_},
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
 * but it will block until will all tasks finish.
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


/** Get the current encoder position
 *
 * @return Encoder Position
 */
uint64_t MotorDrive::GetEncoderPosition() const
{
    return (encoder_->GetPosition() >> enc_position_shift_);
}

/** Set the IEncoder interface
 *
 * @param encoder IEncoder interface
 */
void MotorDrive::SetEncoder(IEncoder *encoder)
{
    if (encoder) {
        if (!encoder->Initialize())
            throw std::runtime_error("Encoder initialization error.");
        encoder_ = encoder;
        enc_resolution_bits_ = encoder_->GetResolutionBits() - enc_position_shift_;
        enc_cpr_ = (1 << enc_resolution_bits_);
        enc_resolution_mask_ = (1 << enc_resolution_bits_) - 1;
        enc_revolution_bits_ = encoder_->GetRevolutionBits();
    }
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

/** Return the encoder update time period in seconds
 *
 * This might be different than the @ref GetTimeSlice,
 * because the update period for the encoder might be
 * slower (meaning it takes longer time for the encoder
 * to update).
 *
 * @return Encoder update time period
 */
float MotorDrive::GetEncoderTimeSlice() const
{
    return enc_time_slice_;
}

/** Get the motor pole pairs.
 *
 * @return motor pole pairs
 */
uint32_t MotorDrive::GetPolePairs() const
{
    return config_.pole_pairs;
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

/** IrqUpdateCallback is the interrupt handler that drives the motor.
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
void MotorDrive::IrqUpdateCallback()
{
    if (pwm_->GetCounterDirection()) {
        t1_begin_ = hrtimer.GetCounter();

        data_.vbus_ = AdcData::ReadBusVoltage();
        AdcData::ReadPhaseCurrent(data_.injdata_, 3);

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
        if (data_.update_counter_ % (config_.enc_skip_updates_ + 1) == 0) {
            encoder_->Update();
            UpdateRotor();
        }

        /*
         * Sample ADC phase current
         */
        data_.vbus_ = AdcData::ReadBusVoltage();
        AdcData::ReadPhaseCurrent(data_.injdata_, 3);

        /*
         * Apply the ADC bias to the current data
         */
        data_.phase_current_a_ = CalculatePhaseCurrent(data_.injdata_[0], lpf_bias_a.Output());
        data_.phase_current_b_ = CalculatePhaseCurrent(data_.injdata_[1], lpf_bias_b.Output());
        data_.phase_current_c_ = CalculatePhaseCurrent(data_.injdata_[2], lpf_bias_c.Output());

        /*
         * Update the Iab vector
         */
        UpdateCurrent();

        /*
         * Check for abnormal conditions.
         */
        CheckTripViolations();

        /*
         * Run the scheduler tasks
         */
        sched_.OnUpdate();

        /*
         * Record the high resolution time.
         */
        t2_end_ = hrtimer.GetCounter();
        t2_span_ = hrtimer.GetTimeElapsedMicroSec(t2_begin_,t2_end_);
    }
}

void MotorDrive::UpdateVbus()
{
	lpf_vbus_.DoFilter(__LL_ADC_CALC_DATA_TO_VOLTAGE(config_.Vref_, data_.vbus_, LL_ADC_RESOLUTION_12B) * config_.Vbus_resistor_ratio_);
}

void MotorDrive::UpdateCurrent()
{
	lpf_Ia_.DoFilter(data_.phase_current_a_);
	lpf_Ib_.DoFilter(data_.phase_current_b_);
	lpf_Ic_.DoFilter(-data_.phase_current_a_ -data_.phase_current_b_);
	Iab_ = Pa_ * lpf_Ia_.Output() + Pb_ * lpf_Ib_.Output() + Pc_ * lpf_Ic_.Output();
}

/** Update the rotor position and velocity
 *
 */
void MotorDrive::UpdateRotor()
{
    uint64_t Renc_prev = Renc_; 
    std::complex<float> Eprev = E_;
    Renc_ = GetEncoderPosition();
	float theta_e = GetEncoderDir() * GetElectricAngle(Renc_);
	float theta_m = GetEncoderDir() * GetMechanicalAngle(Renc_);
	E_ = std::complex<float>(cosf(theta_e), sinf(theta_e));
	R_ = std::complex<float>(cosf(theta_m), sinf(theta_m));
	crossE_ = sdmath::cross(Eprev, E_);

	int32_t Rangle_prev = Renc_prev & enc_resolution_mask_;
	int32_t Rangle = Renc_ & enc_resolution_mask_;
	int32_t Wenc = (Rangle + enc_cpr_ - Rangle_prev) % enc_cpr_;
	if (Wenc > (int32_t)(enc_cpr_ / 2))
	    Wenc -= enc_cpr_;
	lpf_Wenc_.DoFilter(Wenc);
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
    uint32_t cpr_per_pair = (enc_cpr_ / GetPolePairs());
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


std::complex<float> MotorDrive::GetRotorMechRotation()
{
	return R_;
}

std::complex<float> MotorDrive::GetRotorElecRotation()
{
	return E_;
}

/** Get the rotor velocity in encoder counts per second
 *
 * @return Mechanical rotor velocity
 */
float MotorDrive::GetRotorVelocity()
{
    return lpf_Wenc_.Output();
}

/** Get the rotor velocity in elec encoder counts per second
 *
 * @return Electrical rotor velocity
 */
float MotorDrive::GetRotorElecVelocity()
{
    return GetRotorVelocity() * config_.pole_pairs;
}


/** Get the velocity of the rotor's electrical rotation.
 *
 * This method returns the electrical velocity of the rotor,
 * calculated as cross product of two consecutive E_ vectors sampled
 * at the beginning and the end of one time slice. The returned magnitude
 * is proportional to the sin(Theta), where Theta is the angle of rotation
 * of the rotor for one time slice.
 *
 * @note This is not measured in Rad/Sec.
 * @return The magnetude of the vector calculated as cross(Eprev, E_)
 */
float MotorDrive::GetRotorElecVelocityCrossProd()
{
	return crossE_;
}



float MotorDrive::CalculatePhaseCurrent(float adc_val, float adc_bias)
{
	return ((adc_bias - adc_val) * config_.Vref_ / config_.adc_full_scale) / config_.Rsense_ / config_.csa_gain_;
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
	float v_abs = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
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
	}, reset_voltage, reset_hz, reset_encoder));
}

void MotorDrive::AddTaskDetectEncoderDir()
{
	sched_.AddTask([&](){
		if (RunUpdateHandlerRotateMotor(M_PI_2, M_PI, config_.reset_voltage_, true))
			config_.encoder_dir_ = (GetMechanicalAngle(GetEncoderPosition()) > M_PI) ? -1 : 1;
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

void MotorDrive::RunTaskAlphaPoleSearch()
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
			fprintf(stderr, "Enc: %7lu\n", (uint32_t)(GetEncoderPosition() & enc_resolution_bits_));
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
                fprintf(stderr, "Vbus: %4.2f, Ia: %6.3f, Ib: %6.3f, Ic: %6.3f, Ia+Ib+Ic: %6.3f\n",
                        lpf_vbus_.Output(), data_.phase_current_a_, data_.phase_current_b_, data_.phase_current_c_,
                        data_.phase_current_a_ + data_.phase_current_b_ + data_.phase_current_c_);
            }
#endif
        } while (ret && i++ < test_cycles);
        if (ret)
            config_.resistance_ = 2.0f / 3.0f * test_voltage / data_.phase_current_a_;
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

                lpf_a.DoFilter(std::abs(data_.phase_current_a_));
                V = V * r;

                // Test voltage along phase A
                ApplyPhaseVoltage(test_voltage * V.real(), std::complex<float>(1.0f, 0.0f));
                return false;
            });
#if 0
            if ((i % 13) == 0) {
                fprintf(stderr, "Vbus: %4.2f, Ia: %6.3f\n",
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
    fprintf(stderr, "VBus: %+5.2f, Bias: %+5.2f, %+5.2f, %+5.2f, Currents: %+5.2f, %+5.2f, %+5.2f\n", lpf_vbus_.Output(),
            lpf_bias_a.Output(), lpf_bias_b.Output(), lpf_bias_c.Output(), data_.phase_current_a_, data_.phase_current_b_,
            data_.phase_current_c_);
}

bool MotorDrive::CheckPhaseCurrentViolation(float current)
{
    if (current > config_.trip_i_) {
        Abort();
        error_info_.SetError(e_trip_voltage, "Current limit violation, %f", current);
        return true;
    }
    return false;
}

bool MotorDrive::CheckPhaseVoltageViolation(float voltage)
{
    if (voltage > config_.trip_v_) {
        Abort();
        error_info_.SetError(e_trip_voltage, "Voltage bus limit violation, %f", voltage);
        return true;
    }
    return false;
}

bool MotorDrive::CheckTripViolations()
{
    bool ret = false;
    if (delay_trip_check_ < (update_hz_ / 4)) {
        delay_trip_check_++;
        return false;
    }
    ret = ret || CheckPhaseVoltageViolation(lpf_vbus_.Output());
    ret = ret || CheckPhaseCurrentViolation(lpf_Ia_.Output());
    ret = ret || CheckPhaseCurrentViolation(lpf_Ib_.Output());
    ret = ret || CheckPhaseCurrentViolation(lpf_Ic_.Output());
    if (ret)
        delay_trip_check_ = 0;
    return ret;
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

