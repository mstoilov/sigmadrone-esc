
#include <string.h>
#include <math.h>

#include "motorctrl_foc.h"
#include "sdmath.h"
#include "uartrpcserver.h"
#include "minasa4encoder.h"

extern UartRpcServer rpc_server;

MotorCtrlFOC::MotorCtrlFOC(MotorDrive* drive, std::string axis_id, TIM_HandleTypeDef* htim_pulse)
	: drive_(drive)
	, axis_id_(axis_id)
	, pid_Id_(config_.pid_current_kp_, config_.pid_current_ki_, config_.pid_current_maxout_)
	, pid_Iq_(config_.pid_current_kp_, config_.pid_current_ki_, config_.pid_current_maxout_)
	, pid_W_(config_.pid_w_kp_, config_.pid_w_ki_, 0, 1, config_.pid_w_maxout_, 0)
	, pid_P_(config_.pid_p_kp_, config_.pid_p_maxout_)
	, htim_pulse_(htim_pulse)
	, capture_interval_(200)
	, capture_mode_(15)
	, capture_capacity_(500)
	, go_(false)
{
	capture_position_.reserve(capture_capacity_);
	capture_velocity_.reserve(capture_capacity_);
	capture_velocityspec_.reserve(capture_capacity_);
	capture_current_.reserve(capture_capacity_);
	StartMonitorThread();
	RegisterRpcMethods();
	UpdatePulseTimerPeriod();
}

void MotorCtrlFOC::RegisterRpcMethods()
{
	std::string prefix = axis_id_ + ".";
	rpc_server.add(prefix, "modeclp", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeClosedLoopPositionTrajectory, "void MotorCtrlFOC::ModeClosedLoopPositionTrajectory()"));
	rpc_server.add(prefix, "modeclv", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeClosedLoopVelocity, "void MotorCtrlFOC::ModeClosedLoopVelocity()"));
	rpc_server.add(prefix, "modeclt", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeClosedLoopTorque, "void MotorCtrlFOC::ModeClosedLoopTorque()"));
	rpc_server.add(prefix, "modespin", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeSpin, "void MotorCtrlFOC::ModeSpin()"));
	rpc_server.add(prefix, "stop", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Stop, "void MotorCtrlFOC::Stop()"));
	rpc_server.add(prefix, "stopmove", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::StopMove, "void MotorCtrlFOC::StopMove()"));
	rpc_server.add(prefix, "calibration", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::RunCalibrationSequence, "void MotorCtrlFOC::RunCalibrationSequence()"));
	rpc_server.add(prefix, "velocity_rps", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::VelocityRPS, "void MotorCtrlFOC::VelocityRPS(float revpersec)"));
	rpc_server.add(prefix, "mvp", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::MoveToPosition, "void MotorCtrlFOC::MoveToPosition(uint64_t position)"));
	rpc_server.add(prefix, "mvr", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::MoveRelative, "void MotorCtrlFOC::MoveRelative(int64_t relative)"));
	rpc_server.add(prefix, "mvpp", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::MoveToPositionParams, "void MotorCtrlFOC::MoveToPositionParams(uint64_t target, uint32_t v, uint32_t acc, uint32_t dec)"));
	rpc_server.add(prefix, "mvrp", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::MoveRelativeParams, "void MotorCtrlFOC::MoveRelativeParams(int64_t relateive, uint32_t v, uint32_t acc, uint32_t dec)"));
	rpc_server.add(prefix, "get_captured_position", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::GetCapturedPosition, "resjson::array MotorCtrlFOC::GetCapturePosition()"));
	rpc_server.add(prefix, "get_captured_velocity", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::GetCapturedVelocity, "resjson::array MotorCtrlFOC::GetCaptureVelocity()"));
	rpc_server.add(prefix, "get_captured_velocityspec", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::GetCapturedVelocitySpec, "resjson::array MotorCtrlFOC::GetCaptureVelocitySpec()"));
	rpc_server.add(prefix, "get_captured_current", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::GetCapturedCurrent, "resjson::array MotorCtrlFOC::GetCaptureCurrent()"));

	rpc_server.add(prefix, "push", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::PushStreamPoint, "void PushStreamPoint(int64_t time, int64_t velocity, int64_t position)"));
	rpc_server.add(prefix, "pushv", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::PushStreamPointV, "void PushStreamPointV(const std::vector<int64_t>& v)"));
	rpc_server.add(prefix, "go", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Go, "void Go()"));


	rpc_server.add(prefix, "modeclps", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeClosedLoopPositionStream, "void MotorCtrlFOC::ModeClosedLoopPositionStream()"));
	rpc_server.add(prefix, "pulse_stream", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::PulseStream, "void PulseStream(const std::vector<int64_t>& v)"));
	rpc_server.add(prefix, "mvrps", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::MoveRelativePulseStream, "void MoveRelativePulseStream(int64_t relative)"));
	rpc_server.add(prefix, "mvt", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::MoveTrapezoid, "uint64_t MoveTrapezoid(int64_t relative, uint32_t v, uint32_t acc, uint32_t dec)"));
	
	
	rpc_server.add(prefix, "smodeclp", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::SimpleModeClosedLoopPosition, "void MotorCtrlFOC::SimpleModeClosedLoopPosition()"));
	rpc_server.add(prefix, "smvp", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::SimpleMoveToPosition, "void SimpleMotorCtrlFOC::MoveToPosition(uint64_t position)"));
	rpc_server.add(prefix, "smvr", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::SimpleMoveRelative, "void SimpleMotorCtrlFOC::MoveRelative(int64_t relative)"));


	drive_->RegisterRpcMethods(prefix + "drive.");
}

rexjson::property MotorCtrlFOC::GetPropertyMap()
{
	rexjson::property props = rexjson::property_map({
		{"drive", rexjson::property({drive_->GetPropertyMap()})},
		{"q_current", {&q_current_, rexjson::property_get<decltype(q_current_)>, rexjson::property_set<decltype(q_current_)>}},
		{"velocity", {&velocity_, rexjson::property_get<decltype(velocity_)>, rexjson::property_set<decltype(velocity_)>}},
		{"acceleration", {&acceleration_, rexjson::property_get<decltype(acceleration_)>, rexjson::property_set<decltype(acceleration_)>}},
		{"deceleration", {&deceleration_, rexjson::property_get<decltype(deceleration_)>, rexjson::property_set<decltype(deceleration_)>}},
		{"target", {&target_, rexjson::property_get<decltype(target_)>, rexjson::property_set<decltype(target_)>}},
		{"spin_voltage", {&spin_voltage_, rexjson::property_get<decltype(spin_voltage_)>, rexjson::property_set<decltype(spin_voltage_)>}},
		{"capture_interval", {&capture_interval_, rexjson::property_get<decltype(capture_interval_)>, rexjson::property_set<decltype(capture_interval_)>}},
		{"capture_mode", {&capture_mode_, rexjson::property_get<decltype(capture_mode_)>, rexjson::property_set<decltype(capture_mode_)>}},
		{"capture_capacity", rexjson::property(
			&capture_capacity_, 
			rexjson::property_get<decltype(capture_capacity_)>,
			[&](const rexjson::value& v, void* ctx) {
				__disable_irq();
				rexjson::property_set<decltype(capture_capacity_)>(v, ctx);
				capture_position_.reserve(capture_capacity_);
				capture_velocity_.reserve(capture_capacity_);
				capture_velocityspec_.reserve(capture_capacity_);
				capture_current_.reserve(capture_capacity_);
				capture_velocityspec_.reserve(capture_capacity_);
				__enable_irq(); 
			}
		)},
		{"lpf_Iq", {&lpf_Iq_, rexjson::property_get<decltype(lpf_Iq_)>}},
		{"pulse_counter", rexjson::property(
			&pulse_counter_, 
			rexjson::property_get<decltype(pulse_counter_)>,
			[&](const rexjson::value& v, void* ctx) {
				rexjson::property_set<decltype(pulse_counter_)>(v, ctx);
				__HAL_TIM_ENABLE(htim_pulse_);
			}
		)},
		{"pulse_direction", {&pulse_direction_, rexjson::property_get<decltype(pulse_direction_)>, rexjson::property_set<decltype(pulse_direction_)>}},
		{"pulses_per_sec", rexjson::property(
			&pulses_per_sec_, 
			rexjson::property_get<decltype(pulses_per_sec_)>,
			[&](const rexjson::value& v, void* ctx) {
				rexjson::property_set<decltype(pulses_per_sec_)>(v, ctx);
				UpdatePulseTimerPeriod();
			}
		)},
		{"hclk", rexjson::property(
			nullptr, 
			[&](void* ctx)->rexjson::value {return HAL_RCC_GetHCLKFreq(); }
		)},
		{"pclk1", rexjson::property(
			nullptr, 
			[&](void* ctx)->rexjson::value {return HAL_RCC_GetPCLK1Freq(); }
		)},
		{"pclk2", rexjson::property(
			nullptr, 
			[&](void* ctx)->rexjson::value {return HAL_RCC_GetPCLK2Freq(); }
		)},
		{"arr", rexjson::property(
			nullptr, 
			[&](void* ctx)->rexjson::value {return __HAL_TIM_GET_AUTORELOAD(htim_pulse_); }
		)},
	});
	return props;
}


rexjson::property MotorCtrlFOC::GetConfigPropertyMap()
{
	rexjson::property props = rexjson::property_map({
		{"drive", rexjson::property({drive_->GetConfigPropertyMap()})},
		{"pid_current_kp", rexjson::property(
				&config_.pid_current_kp_,
				rexjson::property_get<decltype(config_.pid_current_kp_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					rexjson::property_set<decltype(config_.pid_current_kp_)>(v, ctx);
					pid_Iq_.SetGainP(config_.pid_current_kp_);
					pid_Id_.SetGainP(config_.pid_current_kp_);
				})},
		{"pid_current_ki", rexjson::property(
				&config_.pid_current_ki_,
				rexjson::property_get<decltype(config_.pid_current_ki_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					rexjson::property_set<decltype(config_.pid_current_ki_)>(v, ctx);
					pid_Iq_.SetGainI(config_.pid_current_ki_);
					pid_Id_.SetGainI(config_.pid_current_ki_);
				})},
		{"pid_current_maxout", rexjson::property(
				&config_.pid_current_maxout_,
				rexjson::property_get<decltype(config_.pid_current_maxout_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					rexjson::property_set<decltype(config_.pid_current_maxout_)>(v, ctx);
					pid_Iq_.SetMaxOutput(config_.pid_current_maxout_);
					pid_Id_.SetMaxOutput(config_.pid_current_maxout_);
				})},
		{"w_bias", rexjson::property(
				&config_.w_bias_,
				rexjson::property_get<decltype(config_.w_bias_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					rexjson::property_set<decltype(config_.w_bias_)>(v, ctx);
					pid_W_.SetBias(config_.w_bias_);
				})},
		{"pid_w_kp", rexjson::property(
				&config_.pid_w_kp_,
				rexjson::property_get<decltype(config_.pid_w_kp_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					rexjson::property_set<decltype(config_.pid_w_kp_)>(v, ctx);
					pid_W_.SetGainP(config_.pid_w_kp_);
				})},
		{"pid_w_ki", rexjson::property(
				&config_.pid_w_ki_,
				rexjson::property_get<decltype(config_.pid_w_ki_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					rexjson::property_set<decltype(config_.pid_w_ki_)>(v, ctx);
					pid_W_.SetGainI(config_.pid_w_ki_);
				})},
		{"pid_w_maxout", rexjson::property(
				&config_.pid_w_maxout_,
				rexjson::property_get<decltype(config_.pid_w_maxout_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					rexjson::property_set<decltype(config_.pid_w_maxout_)>(v, ctx);
					pid_W_.SetMaxOutput(config_.pid_w_maxout_);
				})},
		{"pid_p_kp", rexjson::property(
				&config_.pid_p_kp_,
				rexjson::property_get<decltype(config_.pid_p_kp_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					rexjson::property_set<decltype(config_.pid_p_kp_)>(v, ctx);
					pid_P_.SetGainP(config_.pid_p_kp_);
				})},
		{"pid_p_maxout", rexjson::property(
				&config_.pid_p_maxout_,
				rexjson::property_get<decltype(config_.pid_p_maxout_)>,
				[&](const rexjson::value& v, void* ctx)->void {
					rexjson::property_set<decltype(config_.pid_p_maxout_)>(v, ctx);
					pid_P_.SetMaxOutput(config_.pid_p_maxout_);
				})},
		{"tau_ratio", {&config_.tau_ratio_, rexjson::property_get<decltype(config_.tau_ratio_)>, rexjson::property_set<decltype(config_.tau_ratio_)>}},
		{"vab_advance_factor", {&config_.vab_advance_factor_, rexjson::property_get<decltype(config_.vab_advance_factor_)>, rexjson::property_set<decltype(config_.vab_advance_factor_)>}},
		{"crash_current", {&config_.crash_current_, rexjson::property_get<decltype(config_.crash_current_)>, rexjson::property_set<decltype(config_.crash_current_)>}},
		{"crash_backup", {&config_.crash_backup_, rexjson::property_get<decltype(config_.crash_backup_)>, rexjson::property_set<decltype(config_.crash_backup_)>}},
		{"display", {&config_.display_, rexjson::property_get<decltype(config_.display_)>, rexjson::property_set<decltype(config_.display_)>}},
		{"pulse_enc_counts", {&config_.pulse_enc_counts_, rexjson::property_get<decltype(config_.pulse_enc_counts_)>, rexjson::property_set<decltype(config_.pulse_enc_counts_)>}},
	});
	return props;
}


void MotorCtrlFOC::Stop()
{
	drive_->Abort();
}

void MotorCtrlFOC::RunMonitorLoop()
{
	for (;;) {
		uint32_t status = osThreadFlagsWait(SIGNAL_DEBUG_DUMP_POSITION | 
			SIGNAL_DEBUG_DUMP_TRAJECTORY | 
			SIGNAL_DEBUG_DUMP_VELOCITY | 
			SIGNAL_DEBUG_DUMP_TORQUE | 
			SIGNAL_DEBUG_DUMP_SPIN | 
			SIGNAL_CRASH_DETECTED |
			SIGNAL_RELATEDCRASH_DETECTED, osFlagsWaitAny, -1);
		if (status & osFlagsError) {
			continue;
		} else if (status & SIGNAL_DEBUG_DUMP_TORQUE) {
			fprintf(stderr,
					"Sp: %+6.2f (%+9.2f) I_d: %+6.3f I_q: %+6.3f PVd: %+5.2f PVq: %+7.2f PVqP: %+7.2f PVqI: %+7.2f "
					"Ierr: %+7.3f T: %3lu\r\n",
					drive_->GetRotorVelocity() / drive_->GetEncoderCPR(),
					drive_->GetRotorVelocityPTS(),
					lpf_Id_,
					lpf_Iq_,
					pid_Id_.Output(),
					pid_Iq_.Output(),
					pid_Iq_.OutputP(),
					pid_Iq_.OutputI(),
					Ierr_,
					foc_time_
			);
		} else if (status & SIGNAL_DEBUG_DUMP_VELOCITY) {
			fprintf(stderr,
					"PR: %10llu CV: %+6.2f (%+9.2f) I_d: %+6.3f I_q: %+6.3f PVd: %+5.2f PVq: %+7.2f PVqP: %+7.2f PVqI: %+7.2f "
					"Ierr: %+7.3f Werr: %+7.3f PID_W: %+6.3f PID_WP: %+6.3f PID_WI: %+6.3f T: %3lu\r\n",
					drive_->GetRotorPosition(),
					drive_->GetRotorVelocity() / drive_->GetEncoderCPR(),
					drive_->GetRotorVelocityPTS(),
					lpf_Id_,
					lpf_Iq_,
					pid_Id_.Output(),
					pid_Iq_.Output(),
					pid_Iq_.OutputP(),
					pid_Iq_.OutputI(),
					Ierr_,
					Werr_,
					pid_W_.Output(),
					pid_W_.OutputP(),
					pid_W_.OutputI(),
					foc_time_
			);

		} else if (status & SIGNAL_DEBUG_DUMP_POSITION) {
			fprintf(stderr,
					"P: %10llu (%10llu) I_q: %+6.3f PVd: %+5.2f PVq: %+7.2f PVqP: %+7.2f PVqI: %+7.2f "
					"Werr: %+7.3f PID_W: %+6.3f Perr: %+6.1f PID_PP: %+6.1f V_PEP: %+6.1f, T: %3lu\r\n",
					drive_->GetRotorPosition(),
					target_,
					lpf_Iq_,
					pid_Id_.Output(),
					pid_Iq_.Output(),
					pid_Iq_.OutputP(),
					pid_Iq_.OutputI(),
					Werr_,
					pid_W_.Output(),
					Perr_,
					pid_P_.Output(),
					drive_->GetRotorVelocityPTS(),
					foc_time_
			);

		} else if (status & SIGNAL_DEBUG_DUMP_TRAJECTORY) {
			fprintf(stderr,
					"%s: %10llu (%10llu) I_q: %+6.3f PVq: %+7.2f "
					"Werr: %+7.3f PID_W: %+6.3f Perr: %+6.1f PID_PP: %+6.1f V_PTS: %+8.3f V: %5.1f T: %3lu\r\n",
					axis_id_.c_str(),
					drive_->GetRotorPosition(),
					target_,
					lpf_Iq_,
					pid_Iq_.Output(),
					Werr_,
					pid_W_.Output(),
					Perr_,
					pid_P_.Output(),
					drive_->GetRotorVelocityPTS() * drive_->GetUpdateFrequency(),
					ms_.V * drive_->GetUpdateFrequency(),
					foc_time_
			);
		} else if (status & SIGNAL_DEBUG_DUMP_SPIN) {
			fprintf(stderr,
					"VBus: %5.2f Speed: %9.2f (%9.2f), I_d: %+5.3f, I_q: %+6.3f, T: %4lu, Adv1: %+5.3f\r\n",
					drive_->GetBusVoltage(),
					drive_->GetRotorVelocity() / drive_->GetEncoderCPR(),
					drive_->GetRotorVelocityPTS(),
					lpf_Id_,
					lpf_Iq_,
					foc_time_,
					config_.vab_advance_factor_ * drive_->GetRotorElecVelocityPTS()/drive_->GetEncoderCPR() * 2.0f * M_PI
			);
		} else if (status & (SIGNAL_CRASH_DETECTED | SIGNAL_RELATEDCRASH_DETECTED)) {

		}
	}
}

void MotorCtrlFOC::RunDebugLoopWrapper(void* ctx)
{
	reinterpret_cast<MotorCtrlFOC*>(const_cast<void*>(ctx))->RunMonitorLoop();
}

void MotorCtrlFOC::SignalDumpTrajectory()
{
	if (monitor_thread_)
		osThreadFlagsSet(monitor_thread_, SIGNAL_DEBUG_DUMP_TRAJECTORY);
}

void MotorCtrlFOC::SignalDumpPosition()
{
	if (monitor_thread_)
		osThreadFlagsSet(monitor_thread_, SIGNAL_DEBUG_DUMP_POSITION);
}

void MotorCtrlFOC::SignalDumpVelocity()
{
	if (monitor_thread_)
		osThreadFlagsSet(monitor_thread_, SIGNAL_DEBUG_DUMP_VELOCITY);
}

void MotorCtrlFOC::SignalDumpTorque()
{
	if (monitor_thread_)
		osThreadFlagsSet(monitor_thread_, SIGNAL_DEBUG_DUMP_TORQUE);
}

void MotorCtrlFOC::SignalDumpSpin()
{
	if (monitor_thread_)
		osThreadFlagsSet(monitor_thread_, SIGNAL_DEBUG_DUMP_SPIN);
}

void MotorCtrlFOC::SignalCrashDetected()
{
	if (monitor_thread_)
		osThreadFlagsSet(monitor_thread_, SIGNAL_CRASH_DETECTED);
}

void MotorCtrlFOC::SignalRelatedCrashDetected()
{
	if (monitor_thread_)
		osThreadFlagsSet(monitor_thread_, SIGNAL_RELATEDCRASH_DETECTED);
}


rexjson::array MotorCtrlFOC::GetCapturedPosition() 
{
	rexjson::array ret;
	for (const auto& i : capture_position_)
		ret.push_back(i);
	return ret;
}

rexjson::array MotorCtrlFOC::GetCapturedVelocity() 
{
	rexjson::array ret;
	for (const auto& i : capture_velocity_)
		ret.push_back(i);
	return ret;
}

rexjson::array MotorCtrlFOC::GetCapturedVelocitySpec() 
{
	rexjson::array ret;
	for (const auto& i : capture_velocityspec_)
		ret.push_back(i);
	return ret;
}

rexjson::array MotorCtrlFOC::GetCapturedCurrent() 
{
	rexjson::array ret;
	for (const auto& i : capture_current_)
		ret.push_back(i);
	return ret;
}

void MotorCtrlFOC::StartMonitorThread()
{
	osThreadAttr_t task_attributes;
	memset(&task_attributes, 0, sizeof(osThreadAttr_t));
	task_attributes.name = "DebugFOC";
	task_attributes.priority = (osPriority_t) osPriorityNormal;
	task_attributes.stack_size = 2048;
	monitor_thread_ = osThreadNew(RunDebugLoopWrapper, this, &task_attributes);
}

void MotorCtrlFOC::ModeSpin()
{
	drive_->AddTaskArmMotor();
	drive_->sched_.AddTask([&](){
		uint32_t display_counter = 0;
		drive_->ResetUpdateCounter();
		pid_Id_.Reset();
		pid_Iq_.Reset();
		pid_W_.Reset();
		drive_->sched_.RunUpdateHandler([&]()->bool {

			std::complex<float> Iab = drive_->GetPhaseCurrent();
			std::complex<float> E = drive_->GetRotorElecRotation();

			/*
			 *  Park Transform
			 *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
			 *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
			 *
			 *  Idq = std::complex<float>(Id, Iq);
			 */
			std::complex<float> Idq = Iab * std::conj(E);
			lpf_Id_ = Idq.real();
			lpf_Iq_ = Idq.imag();

			/*
			 * Apply advance
			 */
			float advance = config_.vab_advance_factor_ * drive_->GetRotorElecVelocityPTS()/drive_->GetEncoderCPR() * 2.0f * M_PI;
			std::complex<float> Eadv = E * std::complex<float>(cosf(advance), sinf(advance));

			/*
			 * Inverse Park Transform
			 * Va = Vd * cos(R) - Vq * sin(R)
			 * Vb = Vd * sin(R) + Vq * cos(R)
			 *
			 * Vab = std::complex<float>(Va, Vb)
			 */
			std::complex<float> V_ab = std::complex<float>(0, spin_voltage_) * Eadv;

			/*
			 * Apply the voltage timings
			 */
			drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

			if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
				foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t_begin_, hrtimer.GetCounter());
				SignalDumpSpin();
			}

			return true;
		});
	});
	drive_->AddTaskDisarmMotor();
	drive_->Run();
}

void MotorCtrlFOC::ModeClosedLoopTorque()
{
	drive_->AddTaskArmMotor();
	drive_->sched_.AddTask([&](){
		float timeslice = drive_->GetTimeSlice();
		uint32_t display_counter = 0;
		drive_->ResetUpdateCounter();
		pid_Id_.Reset();
		pid_Iq_.Reset();
		pid_W_.Reset();
		drive_->sched_.RunUpdateHandler([&]()->bool {

			std::complex<float> Iab = drive_->GetPhaseCurrent();
			std::complex<float> R = drive_->GetRotorElecRotation();

			/*
			 *  Park Transform
			 *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
			 *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
			 *
			 *  Idq = std::complex<float>(Id, Iq);
			 */
			std::complex<float> Idq = Iab * std::conj(R);
			lpf_Id_ = Idq.real();
			lpf_Iq_ = Idq.imag();

			/*
			 * Update D/Q PID Regulators.
			 */
			Ierr_ = q_current_ - lpf_Iq_;
			pid_Id_.Input(0.0f - lpf_Id_, timeslice);
			pid_Iq_.Input(Ierr_, timeslice);

			/*
			 * Inverse Park Transform
			 * Va = Vd * cos(R) - Vq * sin(R)
			 * Vb = Vd * sin(R) + Vq * cos(R)
			 *
			 * Vab = std::complex<float>(Va, Vb)
			 */
			std::complex<float> V_ab = std::complex<float>(pid_Id_.Output(), pid_Iq_.Output()) * R;

			/*
			 * Apply the voltage timings
			 */
			drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

			if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
				foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t_begin_, hrtimer.GetCounter());
				SignalDumpTorque();
			}

			return true;
		});

	});
	drive_->AddTaskDisarmMotor();
	drive_->Run();
}

void MotorCtrlFOC::ModeClosedLoopVelocity()
{
	drive_->AddTaskArmMotor();
	drive_->sched_.AddTask([&](){
		float timeslice = drive_->GetTimeSlice();
		uint32_t display_counter = 0;
		drive_->ResetUpdateCounter();
		pid_Id_.Reset();
		pid_Iq_.Reset();
		pid_W_.Reset();
		drive_->sched_.RunUpdateHandler([&]()->bool {
			std::complex<float> Iab = drive_->GetPhaseCurrent();
			std::complex<float> R = drive_->GetRotorElecRotation();
			float velocity_ecp = velocity_ * timeslice;

			Werr_ = velocity_ecp - drive_->GetRotorVelocityPTS();
			pid_W_.Input(Werr_, timeslice);

			/*
			 *  Park Transform
			 *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
			 *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
			 *
			 *  Idq = std::complex<float>(Id, Iq);
			 */
			std::complex<float> Idq = Iab * std::conj(R);
			lpf_Id_ = Idq.real();
			lpf_Iq_ = Idq.imag();

			/*
			 * Update D/Q PID Regulators.
			 */
			Ierr_ = pid_W_.Output() - lpf_Iq_;
			pid_Id_.Input(0.0f - lpf_Id_, timeslice);
			pid_Iq_.Input(Ierr_, timeslice);

			/*
			 * Inverse Park Transform
			 * Va = Vd * cos(R) - Vq * sin(R)
			 * Vb = Vd * sin(R) + Vq * cos(R)
			 *
			 * Vab = std::complex<float>(Va, Vb)
			 */
			std::complex<float> V_ab = std::complex<float>(pid_Id_.Output(), pid_Iq_.Output()) * R;

			/*
			 * Apply the voltage timings
			 */
			drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

			if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
				foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t_begin_, hrtimer.GetCounter());
				SignalDumpVelocity();
			}

			return true;
		});
	});
	drive_->AddTaskDisarmMotor();
	drive_->Run();
}

void MotorCtrlFOC::ModeClosedLoopPositionStream()
{
	drive_->AddTaskArmMotor();
	drive_->sched_.AddTask([&](){
		float timeslice = drive_->GetTimeSlice();
		uint32_t display_counter = 0;
		uint32_t npulse = 0; // 0..3 The current pulse. There are 4 pulses in one byte.
		drive_->ResetUpdateCounter();
		pid_Id_.Reset();
		pid_Iq_.Reset();
		pid_W_.Reset();
		pid_P_.Reset();
		target_ = drive_->GetRotorPosition();
		pulse_stream_.clear();

		drive_->sched_.RunUpdateHandler([&]()->bool {
			/*
			 * Get the latest values for I,R 
			*/
			std::complex<float> Iab = drive_->GetPhaseCurrent();
			std::complex<float> R = drive_->GetRotorElecRotation();
			uint64_t rotor_position = drive_->GetRotorPosition();
			uint32_t update_counter = drive_->GetUpdateCounter();

			/* 
			 * Target calculations
			 */
			if (!pulse_stream_.empty() && go_) {
				uint8_t pulse = *pulse_stream_.get_read_ptr() >> (2 * npulse);
				if (pulse & 0x1) {
					if (pulse & 0x2) {
						target_ -= config_.pulse_enc_counts_;
					} else {
						target_ += config_.pulse_enc_counts_;
					}
				}
				npulse = (npulse + 1) & 0x3;
				if (npulse == 0) {
					pulse_stream_.read_update(1);
					if (pulse_stream_.empty())
						go_ = false;
				}
				if (update_counter % capture_interval_ == 0 && go_)
					Capture();
			}

			/* Calculate the position error and input it in the Position PID */
			Perr_ = drive_->GetRotorPositionError(rotor_position, target_) * timeslice;
			pid_P_.SetMaxOutput(velocity_ * timeslice);
			pid_P_.Input(Perr_, timeslice);

			/* Calculate the velocity error and input it in the Velocity PID */
			Werr_ = pid_P_.Output() - drive_->GetRotorVelocityPTS();
			pid_W_.Input(Werr_, timeslice);

			/*
			 *  Park Transform
			 *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
			 *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
			 *
			 *  Idq = std::complex<float>(Id, Iq);
			 */
			std::complex<float> Idq = Iab * std::conj(R);
			lpf_Id_ = Idq.real();
			lpf_Iq_ = Idq.imag();

			/*
			 * Update D/Q PID Regulators.
			 */
			Ierr_ = pid_W_.Output() - lpf_Iq_;
			pid_Id_.Input(0.0f - lpf_Id_, timeslice);
			pid_Iq_.Input(Ierr_, timeslice);

			/*
			 * Inverse Park Transform
			 * Va = Vd * cos(R) - Vq * sin(R)
			 * Vb = Vd * sin(R) + Vq * cos(R)
			 *
			 * Vab = std::complex<float>(Va, Vb)
			 */
			std::complex<float> V_ab = std::complex<float>(pid_Id_.Output(), pid_Iq_.Output()) * R;

			/*
			 * Apply the voltage timings
			 */
			drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

			if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
				foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t_begin_, hrtimer.GetCounter());
				SignalDumpPosition();
			}
			return true;
		});
	});
	drive_->AddTaskDisarmMotor();
	drive_->Run();
}

void MotorCtrlFOC::Capture()
{
	if (capture_mode_ & CAPTURE_POSITION && 
		capture_position_.size() < capture_capacity_) {
			capture_position_.push_back(drive_->GetRotorPosition());
	}
	if (capture_mode_ & CAPTURE_VELOCITY && 
		capture_velocity_.size() < capture_capacity_) {
			capture_velocity_.push_back(drive_->GetRotorVelocity());
	}
	if (capture_mode_ & CAPTURE_CURRENT && 
		capture_current_.size() < capture_capacity_) {
			capture_current_.push_back(lpf_Iq_);
	}
}

void MotorCtrlFOC::PulseStream(std::vector<uint8_t> v)
{
	if (pulse_stream_.space_size() < v.size())
		throw std::runtime_error("Out of memory");

	for (uint8_t& pulse : v) {
		pulse_stream_.push(pulse);
	}
}

void MotorCtrlFOC::MoveRelativePulseStream(int64_t relative)
{
	uint8_t dir = (relative < 0) ? 1 : 0;
	float moveTime = std::abs(relative)/std::abs(velocity_);
	uint32_t moveUpdates = moveTime * drive_->GetUpdateFrequency();
	uint32_t npulses = std::abs(relative)/config_.pulse_enc_counts_;
	uint32_t pulsePeriod = moveUpdates / npulses;
	for (uint32_t i = 0; i < moveUpdates; i++) {
		PulseStreamPush(dir, (i % pulsePeriod == 0) ? 1 : 0, i);
	}
	PulseStreamFlush();
}

void MotorCtrlFOC::PulseStreamFlush()
{
	uint32_t timeout = 1000;
	while (!pulse_stream_.space_size()) {
		osDelay(1);
		if (timeout == 0)
			throw std::runtime_error("PulseStreamPush timed out.");
		timeout--;
	}
	pulse_stream_.push(scratch_);
	scratch_ = 0;
}

void MotorCtrlFOC::PulseStreamPush(uint32_t dir, uint32_t pulse, uint32_t seq)
{
	uint8_t bits = (dir << 1) | pulse;

	seq &= 3;

	// Clear bits
	if (seq == 0)
		scratch_ = 0;
	
	// Set the Pulse bits
	scratch_ |= (bits << (seq*2));

	seq = (seq + 1) & 3;
	if (seq == 0)
		PulseStreamFlush();
}


void MotorCtrlFOC::MoveTrapezoid(int64_t relative, uint32_t v, uint32_t acc, uint32_t dec)
{
	std::vector<std::vector<int64_t>> points = CalculateTrapezoidPoints(0, relative, 0, 0, v, acc, dec, drive_->GetUpdateFrequency());
	int32_t PULSEN = (relative < 0) ? -config_.pulse_enc_counts_ : config_.pulse_enc_counts_;
	uint32_t dir = (relative < 0) ? 1 : 0;
	uint32_t seq = 0;

	// Iterate through the trapezoid velocity segments
	int64_t Scur = 0;
	uint32_t T1 = 0, T2 = 0, T = 0;
	float V1 = 0, V2 = 0, V = 0;
	float S2 = 0, S = 0;
	float A = 0;
	uint64_t movePeriods = 0;
	uint32_t goTrig = pulse_stream_.space_size()/2;
	for (const std::vector<int64_t>& pt : points)
		movePeriods += pt[0];
	if (movePeriods < goTrig)
		goTrig = movePeriods/2;

	for (std::vector<int64_t>& pt : points) {
		T1 = 0; T2 = (uint32_t)pt[0];
		V1 = V2; V2 = pt[1] / drive_->GetUpdateFrequency();
		S2 = pt[2];
		if (T2 > T1) {
			A = (V2 - V1)/(T2 - T1);
			for (T = 0; T < T2; T++) {
				/*   
				*  V2     /|
				*        / |   
				*       /  |
				*  V   /|  |
				*     / |  |
				*    /__|__|
				*   0   T  T2
				*
				*  S is the area of the triangle 0-T-V
				*  it is calculated by subtracting:
				*  the area of the trapezoid T - T2 - V2 - V
				*  from S2
				*  S = S2 - (V + V2) * (T2 - T) / 2
				*/

				V = V1 + A * T;
				S = S2 - (V2 + V) * (T2 - T) / 2;
				if (std::abs(S) - std::abs(Scur) >= std::abs(PULSEN)) {
					/*
					 * Only generate a pulse when we reach 
					 * the treshold of PULSEN (the number of steps in the pulse)
					*/
					PulseStreamPush(dir, 1, seq);
					Scur += PULSEN;
				} else {
					PulseStreamPush(dir, 0, seq);
				}
				if (seq == goTrig)
					Go();
				seq++;
			}
		}
	}
	// for (int i = 0; i < 8000; i++)
	// 	PulseStreamPush(0, 0, seq++);
	PulseStreamFlush();
}


void MotorCtrlFOC::SimpleModeClosedLoopPosition()
{
	drive_->AddTaskArmMotor();
	drive_->sched_.AddTask([&](){
		float timeslice = drive_->GetTimeSlice();
		uint32_t display_counter = 0;
		drive_->ResetUpdateCounter();
		pid_Id_.Reset();
		pid_Iq_.Reset();
		pid_W_.Reset();
		pid_P_.Reset();
		target_ = drive_->GetRotorPosition();

		drive_->sched_.RunUpdateHandler([&]()->bool {
			/*
			 * Get the latest values for I,R 
			*/
			std::complex<float> Iab = drive_->GetPhaseCurrent();
			std::complex<float> R = drive_->GetRotorElecRotation();
			uint64_t rotor_position = drive_->GetRotorPosition();
			uint32_t update_counter = drive_->GetUpdateCounter();

			/* Calculate the position error and input it in the Position PID */
			Perr_ = drive_->GetRotorPositionError(rotor_position, target_) * timeslice;
			pid_P_.SetMaxOutput(velocity_ * timeslice);
			pid_P_.Input(Perr_, timeslice);

			/* Calculate the velocity error and input it in the Velocity PID */
			Werr_ = pid_P_.Output() - drive_->GetRotorVelocityPTS();
			pid_W_.Input(Werr_, timeslice);

			/*
			 *  Park Transform
			 *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
			 *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
			 *
			 *  Idq = std::complex<float>(Id, Iq);
			 */
			std::complex<float> Idq = Iab * std::conj(R);
			lpf_Id_ = Idq.real();
			lpf_Iq_ = Idq.imag();

			/*
			 * Update D/Q PID Regulators.
			 */
			Ierr_ = pid_W_.Output() - lpf_Iq_;
			pid_Id_.Input(0.0f - lpf_Id_, timeslice);
			pid_Iq_.Input(Ierr_, timeslice);

			/*
			 * Inverse Park Transform
			 * Va = Vd * cos(R) - Vq * sin(R)
			 * Vb = Vd * sin(R) + Vq * cos(R)
			 *
			 * Vab = std::complex<float>(Va, Vb)
			 */
			std::complex<float> V_ab = std::complex<float>(pid_Id_.Output(), pid_Iq_.Output()) * R;

			/*
			 * Apply the voltage timings
			 */
			drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

			if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
				foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t_begin_, hrtimer.GetCounter());
				SignalDumpPosition();
			}
			if (capture_mode_ & CAPTURE_POSITION && 
				update_counter % capture_interval_ == 0 && 
				capture_position_.size() < capture_capacity_) {
					capture_position_.push_back(rotor_position);
			}
			if (capture_mode_ & CAPTURE_VELOCITY && 
				update_counter % capture_interval_ == 0 && 
				capture_velocity_.size() < capture_capacity_) {
					capture_velocity_.push_back(drive_->GetRotorVelocity());
			}
			if (capture_mode_ & CAPTURE_CURRENT && 
				update_counter % capture_interval_ == 0 && 
				capture_current_.size() < capture_capacity_) {
					capture_current_.push_back(lpf_Iq_);
			}
			return true;
		});
	});
	drive_->AddTaskDisarmMotor();
	drive_->Run();
}

void MotorCtrlFOC::StopMove()
{
	__disable_irq();
	velocity_stream_.clear();
	ms_.S = target_ = drive_->GetRotorPosition();
	__enable_irq();
}

void MotorCtrlFOC::ModeClosedLoopPositionTrajectory()
{
	drive_->AddTaskArmMotor();

	drive_->sched_.AddTask([&](){
		float timeslice = drive_->GetTimeSlice();
		uint32_t display_counter = 0;
		drive_->ResetUpdateCounter();
		pid_Id_.Reset();
		pid_Iq_.Reset();
		pid_W_.Reset();
		pid_P_.Reset();
		std::vector<int64_t>* prof_ptr = nullptr;
		ms_.V1 = 0.0f;
		ms_.V2 = 0.0f;
		ms_.V = 0.0f;
		ms_.A = 0.0f;
		ms_.T1 = 0;
		ms_.T2 = 0;
		ms_.S2 = drive_->GetRotorPosition();
		ms_.S = drive_->GetRotorPosition();
		velocity_stream_.clear();
		target_ = drive_->GetRotorPosition();

		drive_->sched_.RunUpdateHandler([&]()->bool {
			std::complex<float> Iab = drive_->GetPhaseCurrent();
			std::complex<float> R = drive_->GetRotorElecRotation();
			uint64_t rotor_position = drive_->GetRotorPosition();
			uint32_t update_counter = drive_->GetUpdateCounter();

			if (velocity_stream_.empty()) {
				go_ = false;
				prof_ptr = nullptr;
			}
again:
			if (go_ && !prof_ptr && !velocity_stream_.empty()) {
				prof_ptr = velocity_stream_.get_read_ptr();
				ms_.T1 = update_counter;
				ms_.T2 = ms_.T1 + prof_ptr->at(0);
				ms_.V1 = ms_.V2;
				ms_.V2 = prof_ptr->at(1) * timeslice;
				ms_.S2 = prof_ptr->at(2);
				if (ms_.T1 == ms_.T2) {
					prof_ptr = nullptr;
					velocity_stream_.pop();
					goto again;
				}
				ms_.A = (ms_.V2 - ms_.V1) / (ms_.T2 - ms_.T1);
			}

			if (prof_ptr) {
				/*   
				*  V2     /|
				*        / |   
				*       /  |
				*  V   /|  |
				*     / |  |
				*    /__|__|
				*   0   T  T2
				*
				*  S is the area of the triangle 0-T-V
				*  it is calculated by subtracting:
				*  the area of the trapezoid T - T2 - V2 - V
				*  from S2
				*  S = S2 - (V + V2) * (T2 - T) / 2
				*/
				ms_.T = (update_counter - ms_.T1);
				ms_.V = ms_.V1 + ms_.A * ms_.T;
				ms_.S = ms_.S2 - (ms_.V + ms_.V2) * (ms_.T2 - update_counter) / 2;
				Perr_ = drive_->GetRotorPositionError(rotor_position, ms_.S) * timeslice;
				pid_P_.Input(Perr_, timeslice);
				Werr_ = pid_P_.Output() + ms_.V - drive_->GetRotorVelocityPTS();
				pid_W_.Input(Werr_, timeslice);
				if (update_counter == ms_.T2) {
					velocity_stream_.pop();
					ms_.A = 0.0f;
					prof_ptr = nullptr;
				}

				if (capture_mode_ & CAPTURE_POSITION && 
					update_counter % capture_interval_ == 0 && 
					capture_position_.size() < capture_capacity_) {
						capture_position_.push_back(rotor_position);
				}
				if (capture_mode_ & CAPTURE_VELOCITY && 
					update_counter % capture_interval_ == 0 && 
					capture_velocity_.size() < capture_capacity_) {
						capture_velocity_.push_back(drive_->GetRotorVelocity());
				}
				if (capture_mode_ & CAPTURE_VELOCITYSPEC && 
					update_counter % capture_interval_ == 0 && 
					capture_velocityspec_.size() < capture_capacity_) {
						capture_velocityspec_.push_back(ms_.V * drive_->GetUpdateFrequency());
				}
				if (capture_mode_ & CAPTURE_CURRENT && 
					update_counter % capture_interval_ == 0 && 
					capture_current_.size() < capture_capacity_) {
						capture_current_.push_back(lpf_Iq_);
				}
			} else {
				Perr_ = drive_->GetRotorPositionError(rotor_position, ms_.S) * timeslice;
				pid_P_.Input(Perr_, timeslice);
				Werr_ = pid_P_.Output() - drive_->GetRotorVelocityPTS();
				pid_W_.Input(Werr_, timeslice);
			}

			/*
			 *  Park Transform
			 *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
			 *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
			 *
			 *  Idq = std::complex<float>(Id, Iq);
			 */
			std::complex<float> Idq = Iab * std::conj(R);
			lpf_Id_ = Idq.real();
			lpf_Iq_ = Idq.imag();

			/*
			 * Update D/Q PID Regulators.
			 */
			Ierr_ = pid_W_.Output() - lpf_Iq_;
			pid_Id_.Input(0.0f - lpf_Id_, timeslice);
			pid_Iq_.Input(Ierr_, timeslice);

			/*
			 * Inverse Park Transform
			 * Va = Vd * cos(R) - Vq * sin(R)
			 * Vb = Vd * sin(R) + Vq * cos(R)
			 *
			 * Vab = std::complex<float>(Va, Vb)
			 */
			std::complex<float> V_ab = std::complex<float>(pid_Id_.Output(), pid_Iq_.Output()) * R;

			/*
			 * Apply the voltage timings
			 */
			drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

			if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
				foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t_begin_, hrtimer.GetCounter());
				SignalDumpTrajectory();
			}

			return true;
		});
	});
	drive_->AddTaskDisarmMotor();
	drive_->Run();
}

void MotorCtrlFOC::PushStreamPointV(std::vector<int64_t> v)
{
	if (v.size() != 3)
		throw std::runtime_error("Invalid stream point");
	velocity_stream_.push(v);
}

void MotorCtrlFOC::PushStreamPoint(int64_t time, int64_t velocity, int64_t position)
{
	std::vector<int64_t> pt{time, velocity, position};
	velocity_stream_.push(pt);
}


void MotorCtrlFOC::Go()
{
	if (go_)
		return;
	capture_position_.resize(0);
	capture_velocity_.resize(0);
	capture_velocityspec_.resize(0);
	capture_current_.resize(0);
	if (!velocity_stream_.empty())
		target_ = velocity_stream_.get_read_ptr_last()->at(2);
	go_ = true;
}


/** Set movement velocity.
 *
 * @param revpersec movement velocity as revolutions per seconds
 * @return return the internal representation of the velocity as encoder counts per second
 */
float MotorCtrlFOC::VelocityRPS(float revpersec)
{
	velocity_ = revpersec * drive_->GetEncoderCPR();
	return velocity_;
}

/**
 * @brief Return the current target position
 * 
 * @return int64_t Target position
 */
int64_t MotorCtrlFOC::GetTarget() const
{
	return target_;
}

/**
 * @brief Set the current target position
 * 
 * @param position New target position
 */
void MotorCtrlFOC::SetTarget(const int64_t position)
{
	target_ = position;
}

uint64_t MotorCtrlFOC::MoveToPositionParams(uint64_t target, uint32_t v, uint32_t acc, uint32_t dec)
{
	if (velocity_stream_.write_size() < 4)
		throw std::range_error("Velocity profiler queue is full.");
	if (target >= drive_->GetEncoderMaxPosition())
		throw std::range_error("Invalid position");
	uint64_t oldpos = target_;

	std::vector<std::vector<int64_t>> points = CalculateTrapezoidPoints(oldpos, target, 0, 0, v, acc, dec, drive_->GetUpdateFrequency());
	__disable_irq();
	for (auto& pt : points)
		velocity_stream_.push({pt[0], pt[1], pt[2]});
	__enable_irq();
	SetTarget(target);
	Go();
	return target_;
}


/** Set target position
 *
 * @param target new position in encoder counts
 * @return the new target position in encoder counts
 */
uint64_t MotorCtrlFOC::MoveToPosition(uint64_t target)
{
	return MoveToPositionParams(target, velocity_, acceleration_, deceleration_);
}

uint64_t MotorCtrlFOC::MoveRelativeParams(int64_t relative, uint32_t v, uint32_t acc, uint32_t dec)
{
	return MoveToPositionParams(target_ + relative, v, acc, dec);
}

uint64_t MotorCtrlFOC::MoveRelative(int64_t relative)
{
	return MoveRelativeParams(relative, velocity_, acceleration_, deceleration_);
}


uint64_t MotorCtrlFOC::SimpleMoveToPosition(uint64_t target)
{
	if (target >= drive_->GetEncoderMaxPosition())
		throw std::range_error("Invalid position");
	target_ = target;
	go_ = true;
	return target_;
}

uint64_t MotorCtrlFOC::SimpleMoveRelative(int64_t relative)
{
	return SimpleMoveToPosition(target_ + relative);
}


void MotorCtrlFOC::RunCalibrationSequence(bool reset_rotor)
{
	drive_->AddTaskCalibrationSequence(reset_rotor);
	drive_->sched_.AddTask([&](){
#if 1
/*
		R = 4.45
		L = 0.0128
		tau = L/R
		Kp = 1/R
		Tratio = 3 # Tcl/Tp

		# Open loop system
		G = cn.tf(Kp, [tau, 1])

		# Pid controller
		Kc = 1/(Kp*Tratio)
		Ti = L/R
		Td = 0
		Gc = cn.tf([Td*Kc, Kc, Kc/Ti], [1, 0])
		CL = cn.feedback(Gc*G,1)

		cn.pzmap(G)
		cn.pzmap(Gc)
		cn.pzmap(CL)

		plt.figure()
		t = np.linspace(0, 0.025, 100)
		x,y=cn.step_response(G,t)
		plt.plot(x,y)
		x,y=cn.step_response(CL,t)
		plt.plot(x,y)
*/
		float tau = drive_->config_.inductance_ / drive_->config_.resistance_;
		config_.pid_current_kp_ = drive_->config_.resistance_ / config_.tau_ratio_;
		config_.pid_current_ki_ = config_.pid_current_kp_ / tau;
		pid_Id_.SetGainP(config_.pid_current_kp_);
		pid_Id_.SetGainI(config_.pid_current_ki_);
		pid_Iq_.SetGainP(config_.pid_current_kp_);
		pid_Iq_.SetGainI(config_.pid_current_ki_);
#endif
	});
	drive_->RunWaitForCompletion();
}

uint32_t MotorCtrlFOC::PulseCallback()
{
	if (pulse_counter_ > 0) {
		pulse_counter_--;
		target_ += (pulse_direction_) ? -config_.pulse_enc_counts_ : config_.pulse_enc_counts_;
	}
	return pulse_counter_;
}

void MotorCtrlFOC::UpdatePulseTimerPeriod()
{
	__HAL_TIM_SET_AUTORELOAD(htim_pulse_, HAL_RCC_GetPCLK1Freq()*2/pulses_per_sec_);
}
