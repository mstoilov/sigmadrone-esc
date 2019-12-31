
#include <string.h>

#include "motorctrl_foc.h"
#include "sdmath.h"
#include "uartrpcserver.h"

extern UartRpcServer rpc_server;

MotorCtrlFOC::MotorCtrlFOC(MotorDrive* drive)
	: drive_(drive)
	, lpf_Id_(config_.id_alpha_)
	, lpf_Iq_(config_.iq_alpha_)
	, pid_Vd_(config_.pid_current_kp_, config_.pid_current_ki_, 0, config_.pid_current_decay_, config_.pid_current_maxout_)
	, pid_Vq_(config_.pid_current_kp_, config_.pid_current_ki_, 0, config_.pid_current_decay_, config_.pid_current_maxout_)
	, pid_W_(config_.pid_w_kp_, config_.pid_w_ki_, 0, config_.pid_w_decay_, config_.pid_w_maxout_)
	, lpf_speed_disp_(config_.speed_disp_alpha_)
{
	props_= rexjson::property_map({
		{"id_alpha", rexjson::property(
				&config_.id_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_Id_.SetAlpha(config_.id_alpha_);
				})},
		{"iq_alpha", rexjson::property(
				&config_.iq_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_Iq_.SetAlpha(config_.iq_alpha_);
				})},
		{"pid_current_kp", rexjson::property(
				&config_.pid_current_kp_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_Vd_.SetGainP(config_.pid_current_kp_);
					pid_Vq_.SetGainP(config_.pid_current_kp_);
				})},
		{"pid_current_ki", rexjson::property(
				&config_.pid_current_ki_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_Vd_.SetGainI(config_.pid_current_ki_);
					pid_Vq_.SetGainI(config_.pid_current_ki_);
				})},
		{"pid_current_decay", rexjson::property(
				&config_.pid_current_decay_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_Vd_.SetDecayRate(config_.pid_current_decay_);
					pid_Vq_.SetDecayRate(config_.pid_current_decay_);
				})},
		{"pid_current_maxout", rexjson::property(
				&config_.pid_current_maxout_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_Vd_.SetMaxIntegralOutput(config_.pid_current_maxout_);
					pid_Vq_.SetMaxIntegralOutput(config_.pid_current_maxout_);
				})},
		{"vq_bias", rexjson::property(
				&config_.vq_bias_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_Vq_.SetBias(config_.vq_bias_);
				})},
		{"w_bias", rexjson::property(
				&config_.w_bias_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_W_.SetBias(config_.w_bias_);
				})},
		{"pid_w_kp", rexjson::property(
				&config_.pid_w_kp_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_W_.SetGainP(config_.pid_w_kp_);
				})},
		{"pid_w_ki", rexjson::property(
				&config_.pid_w_ki_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_W_.SetGainI(config_.pid_w_ki_);
				})},
		{"pid_w_decay", rexjson::property(
				&config_.pid_w_decay_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_W_.SetDecayRate(config_.pid_w_decay_);
				})},
		{"pid_w_maxout", rexjson::property(
				&config_.pid_w_maxout_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_W_.SetMaxIntegralOutput(config_.pid_w_maxout_);
				})},
		{"speed_disp_alpha", rexjson::property(
				&config_.speed_disp_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_speed_disp_.SetAlpha(config_.speed_disp_alpha_);
				})},

		{"control_bandwith", &config_.control_bandwidth_},
		{"iq_setpoint", &config_.iq_setpoint_},
		{"w_setpoint", &config_.w_setpoint_},
		{"i_trip", &config_.i_trip_},
		{"vab_advance_factor", &config_.vab_advance_factor_},
		{"ri_angle", &config_.ri_angle_},
		{"display", &config_.display_},
		{"spin_voltage", &config_.spin_voltage_},
	});

	rpc_server.add("foc.vel_setpoint", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::VelocitySetPoint, "void MotorCtrlFOC::VelocitySetPoint(float revpersec)"));
	rpc_server.add("foc.velocity", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Velocity, "void MotorCtrlFOC::Velocity()"));
	rpc_server.add("foc.torque", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Torque, "void MotorCtrlFOC::Torque()"));
	rpc_server.add("foc.spin", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Spin, "void MotorCtrlFOC::Spin()"));
	rpc_server.add("foc.stop", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Stop, "void MotorCtrlFOC::Stop()"));
	rpc_server.add("foc.calibration", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::RunCalibrationSequence, "void MotorCtrlFOC::RunCalibrationSequence()"));

	StartDebugThread();
}

void MotorCtrlFOC::Stop()
{
	drive_->sched_.Abort();
}

void MotorCtrlFOC::RunDebugLoop()
{
	for (;;) {
		uint32_t status = osThreadFlagsWait(SIGNAL_DEBUG_DUMP_TORQUE | SIGNAL_DEBUG_DUMP_SPIN, osFlagsWaitAny, -1);
		if (status & osFlagsError) {
			continue;
		} else if (status & SIGNAL_DEBUG_DUMP_TORQUE) {
			fprintf(stderr,
					"Speed: %13.9f (%5.2f), I_d: %+5.3f, I_q: %+6.3f, PID_Vd: %+7.3f, PID_Vq: %+7.3f, PID_VqP: %+7.3f, PID_VqI: %+7.3f, "
					"Werr: %+12.9f, PID_W: %+12.9f, PID_WP: %+12.9f, PID_WI: %+12.9f\n",
					lpf_speed_disp_.Output(),
					asinf(lpf_speed_disp_.Output()) * drive_->GetUpdateFrequency() / (M_PI * 2 * drive_->GetPolePairs()),
					lpf_Id_.Output(),
					lpf_Iq_.Output(),
					pid_Vd_.Output(),
					pid_Vq_.Output(),
					pid_Vq_.OutputP(),
					pid_Vq_.OutputI(),
					Werr_,
					pid_W_.Output(),
					pid_W_.OutputP(),
					pid_W_.OutputI()
			);
		} else if (status & SIGNAL_DEBUG_DUMP_SPIN) {
			fprintf(stderr,
					"Speed: %13.9f (%5.2f), I_d: %+5.3f, I_q: %+6.3f, t1_span: %4lu, t2_span: %4lu, signal: %4lu, UpdT: %4lu, FocT: %4lu, \n",
					lpf_speed_disp_.Output(),
					asinf(lpf_speed_disp_.Output()) * drive_->GetUpdateFrequency() / (M_PI * 2 * drive_->GetPolePairs()),
					lpf_Id_.Output(),
					lpf_Iq_.Output(),
					drive_->t1_span_,
					drive_->t2_span_,
					drive_->signal_time_ms_,
					upd_time_,
					foc_time_
			);
		}
	}
}

void MotorCtrlFOC::RunDebugLoopWrapper(void* ctx)
{
	reinterpret_cast<MotorCtrlFOC*>(const_cast<void*>(ctx))->RunDebugLoop();
}

bool MotorCtrlFOC::WaitDebugDump()
{
	return (osThreadFlagsWait(SIGNAL_DEBUG_DUMP_TORQUE, osFlagsWaitAll, -1) == SIGNAL_DEBUG_DUMP_TORQUE) ? true : false;
}

void MotorCtrlFOC::SignalDumpTorque()
{
	if (debug_thread_)
		osThreadFlagsSet(debug_thread_, SIGNAL_DEBUG_DUMP_TORQUE);
}

void MotorCtrlFOC::SignalDumpSpin()
{
	if (debug_thread_)
		osThreadFlagsSet(debug_thread_, SIGNAL_DEBUG_DUMP_SPIN);
}


void MotorCtrlFOC::StartDebugThread()
{
	osThreadAttr_t task_attributes;
	memset(&task_attributes, 0, sizeof(osThreadAttr_t));
	task_attributes.name = "DebugFOC";
	task_attributes.priority = (osPriority_t) osPriorityNormal;
	task_attributes.stack_size = 2048;
	debug_thread_ = osThreadNew(RunDebugLoopWrapper, this, &task_attributes);
}

void MotorCtrlFOC::Torque()
{
	drive_->AddTaskArmMotor();

	drive_->sched_.AddTask([&](){
		bool ret = false;
		float update_period = drive_->GetUpdatePeriod();
		uint32_t display_counter = 0;
		drive_->data_.update_counter_ = 0;
		pid_Vd_.Reset();
		pid_Vq_.Reset();
		pid_W_.Reset();
		lpf_Id_.Reset();
		lpf_Iq_.Reset();
		drive_->lpf_speed_.Reset();
		do {
			ret = drive_->RunUpdateHandler([&]()->bool {
				std::complex<float> Iab = drive_->GetPhaseCurrent();
				std::complex<float> R = drive_->GetElecRotation();
				float phase_speed = drive_->GetPhaseSpeedVector();
//				float rotor_speed = phase_speed * rotor_speed_factor;

				/*
				 *  Park Transform
				 *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
				 *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
				 *
				 *  Idq = std::complex<float>(Id, Iq);
				 */
				std::complex<float> Idq = Iab * std::conj(R);

				/*
				 * Apply filters
				 */
				lpf_Id_.DoFilter(Idq.real());
				lpf_Iq_.DoFilter(Idq.imag());
				lpf_speed_disp_.DoFilter(phase_speed);

				if (std::abs(Idq) > config_.i_trip_) {
					drive_->Abort();
					fprintf(stderr, "i_trip: %7.2f, exceeded by abs(Idq): %7.2f\n", config_.i_trip_, std::abs(Idq));
					return false;
				}

				pid_Vd_.Input(0.0f - Idq.real(), update_period);
				pid_Vq_.Input(config_.iq_setpoint_ - Idq.imag(), update_period);

				/*
				 * Inverse Park Transform
				 * Va = Vd * cos(R) - Vq * sin(R)
				 * Vb = Vd * sin(R) + Vq * cos(R)
				 *
				 * Vab = std::complex<float>(Va, Vb)
				 */
				std::complex<float> V_ab = std::complex<float>(pid_Vd_.Output(), pid_Vq_.Output()) * R;

				/*
				 * Apply advance
				 */
				V_ab *= std::polar<float>(1.0f, config_.vab_advance_factor_ * phase_speed * update_period);

				/*
				 * Apply the voltage timings
				 */
				drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag(), drive_->GetBusVoltage());

				upd_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t2_begin_, drive_->t3_begin_);
				foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t2_begin_, hrtimer.GetCounter());
				if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
					SignalDumpTorque();
				}
				return true;
			});
		} while (ret);
	});
	drive_->AddTaskDisarmMotor();
	drive_->sched_.Run();
}

void MotorCtrlFOC::Velocity()
{
	drive_->AddTaskArmMotor();

	drive_->sched_.AddTask([&](){
		bool ret = false;
		float update_period = drive_->GetUpdatePeriod();
		uint32_t display_counter = 0;
		drive_->data_.update_counter_ = 0;
		pid_Vd_.Reset();
		pid_Vq_.Reset();
		pid_W_.Reset();
		lpf_Id_.Reset();
		lpf_Iq_.Reset();
		drive_->lpf_speed_.Reset();
		do {
			ret = drive_->RunUpdateHandler([&]()->bool {
				std::complex<float> Iab = drive_->GetPhaseCurrent();
				std::complex<float> R = drive_->GetElecRotation();
				float phase_speed = drive_->GetPhaseSpeedVector();
//				float rotor_speed = phase_speed * rotor_speed_factor;

				/*
				 *  Park Transform
				 *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
				 *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
				 *
				 *  Idq = std::complex<float>(Id, Iq);
				 */
				std::complex<float> Idq = Iab * std::conj(R);

				/*
				 * Apply filters
				 */
				lpf_Id_.DoFilter(Idq.real());
				lpf_Iq_.DoFilter(Idq.imag());
				lpf_speed_disp_.DoFilter(phase_speed);

				if (std::abs(Idq) > config_.i_trip_) {
					drive_->Abort();
					fprintf(stderr, "i_trip: %7.2f, exceeded by abs(Idq): %7.2f\n", config_.i_trip_, std::abs(Idq));
					return false;
				}

				Werr_ = config_.w_setpoint_ - phase_speed;
				float Iq_out = pid_W_.Input(Werr_, update_period);
				pid_Vd_.Input(0.0f - lpf_Id_.Output(), update_period);
				pid_Vq_.Input(Iq_out - lpf_Iq_.Output(), update_period);


				/*
				 * Inverse Park Transform
				 * Va = Vd * cos(R) - Vq * sin(R)
				 * Vb = Vd * sin(R) + Vq * cos(R)
				 *
				 * Vab = std::complex<float>(Va, Vb)
				 */
				std::complex<float> V_ab = std::complex<float>(pid_Vd_.Output(), pid_Vq_.Output()) * R;

				/*
				 * Apply advance
				 */
				V_ab *= std::polar<float>(1.0f, config_.vab_advance_factor_ * phase_speed * update_period);

				/*
				 * Apply the voltage timings
				 */
				drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag(), drive_->GetBusVoltage());

				upd_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t2_begin_, drive_->t3_begin_);
				foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t2_begin_, hrtimer.GetCounter());
				if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
					SignalDumpTorque();
				}

				return true;
			});
		} while (ret);
	});
	drive_->AddTaskDisarmMotor();
	drive_->sched_.Run();
}

void MotorCtrlFOC::Spin()
{
	drive_->AddTaskArmMotor();

	drive_->sched_.AddTask([&](){
		bool ret = false;
		float update_period = drive_->GetUpdatePeriod();
		uint32_t display_counter = 0;
		drive_->data_.update_counter_ = 0;
		pid_Vd_.Reset();
		pid_Vq_.Reset();
		pid_W_.Reset();
		lpf_Id_.Reset();
		lpf_Iq_.Reset();
		drive_->lpf_speed_.Reset();
		do {
			ret = drive_->RunUpdateHandler([&]()->bool {
				std::complex<float> Iab = drive_->GetPhaseCurrent();
				std::complex<float> R = drive_->GetElecRotation();
				float phase_speed = drive_->GetPhaseSpeedVector();

				/*
				 *  Park Transform
				 *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
				 *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
				 *
				 *  Idq = std::complex<float>(Id, Iq);
				 */
				std::complex<float> Idq = Iab * std::conj(R);

				if (std::abs(Idq) > config_.i_trip_) {
					drive_->Abort();
					fprintf(stderr, "i_trip: %7.2f, exceeded by abs(Idq): %7.2f\n", config_.i_trip_, std::abs(Idq));
					return false;
				}

				/*
				 * Inverse Park Transform
				 * Va = Vd * cos(R) - Vq * sin(R)
				 * Vb = Vd * sin(R) + Vq * cos(R)
				 *
				 * Vab = std::complex<float>(Va, Vb)
				 */
				std::complex<float> V_ab = std::complex<float>(0, config_.spin_voltage_) * R;

				/*
				 * Apply advance
				 */
				V_ab *= std::polar<float>(1.0f, config_.vab_advance_factor_ * phase_speed * update_period);

				/*
				 * Apply the voltage timings
				 */
				drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag(), drive_->GetBusVoltage());

				upd_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t2_begin_, drive_->t3_begin_);
				foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t2_begin_, hrtimer.GetCounter());
				if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
					SignalDumpSpin();
				}

				return true;
			});
		} while (ret);
	});
	drive_->AddTaskDisarmMotor();
	drive_->sched_.Run();
}



float MotorCtrlFOC::VelocitySetPoint(float revpersec)
{
	config_.w_setpoint_ = std::sin(M_PI * 2 * revpersec * drive_->GetPolePairs() * drive_->GetUpdatePeriod());
	return config_.w_setpoint_;
}

void MotorCtrlFOC::RunCalibrationSequence()
{
	drive_->AddTaskCalibrationSequence();
	drive_->sched_.AddTask([&](){
		config_.pid_current_kp_ = config_.control_bandwidth_ * drive_->config_.inductance_;
		config_.pid_current_ki_ = config_.control_bandwidth_ * drive_->config_.resistance_;
		pid_Vd_.SetGainP(config_.pid_current_kp_);
		pid_Vq_.SetGainP(config_.pid_current_kp_);
		pid_Vd_.SetGainI(config_.pid_current_ki_);
		pid_Vq_.SetGainI(config_.pid_current_ki_);

		config_.pid_w_kp_ = drive_->config_.inductance_ * config_.control_bandwidth_;
		config_.pid_w_ki_ = drive_->config_.resistance_ * config_.control_bandwidth_;
		pid_W_.SetGainP(config_.pid_w_kp_);
		pid_W_.SetGainI(config_.pid_w_ki_);
	});
	drive_->RunWaitForCompletion();
}
