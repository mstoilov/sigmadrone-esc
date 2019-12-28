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
	, lpf_speed_(config_.speed_disp_alpha_)
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
					lpf_speed_.SetAlpha(config_.speed_disp_alpha_);
				})},

		{"control_bandwith", &config_.control_bandwidth_},
		{"iq_setpoint", &config_.iq_setpoint_},
		{"w_setpoint", &config_.w_setpoint_},
		{"i_trip", &config_.i_trip_},
		{"vab_advance_factor", &config_.vab_advance_factor_},
		{"ri_angle", &config_.ri_angle_},
		{"display", &config_.display_},
	});

	rpc_server.add("foc.speed", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Speed, "void MotorCtrlFOC::Speed()"));
	rpc_server.add("foc.torque", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Torque, "void MotorCtrlFOC::Torque()"));
	rpc_server.add("foc.stop", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Stop, "void MotorCtrlFOC::Stop()"));
	rpc_server.add("foc.calibration", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::RunCalibrationSequence, "void MotorCtrlFOC::RunCalibrationSequence()"));
}

void MotorCtrlFOC::Stop()
{
	drive_->sched_.Abort();
}

void MotorCtrlFOC::Torque()
{
	drive_->AddTaskArmMotor();

	drive_->sched_.AddTask([&](){
		bool ret = false;
		float rotor_speed_factor = 1.0f / (M_PI * 2 * drive_->GetPolePairs());
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
				float phase_speed = drive_->GetPhaseSpeed();
				float rotor_speed = phase_speed * rotor_speed_factor;

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

				if (std::abs(Idq) > config_.i_trip_)
					return false;

				pid_Vd_.Input(0.0f - lpf_Id_.Output(), update_period);
				pid_Vq_.Input(config_.iq_setpoint_ - lpf_Iq_.Output(), update_period);
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
				lpf_speed_.DoFilter(rotor_speed);

				/*
				 * Apply the voltage timings
				 */
				drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag(), drive_->GetBusVoltage());

				uint32_t upd_time = hrtimer.GetTimeElapsedMicroSec(drive_->t2_, drive_->t3_);
				uint32_t foc_time = hrtimer.GetTimeElapsedMicroSec(drive_->t2_, hrtimer.GetCounter());
				if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
					fprintf(stderr, "Speed: %6.2f Rev/sec, Idq_d: %+5.3f, Idq_q: %+5.3f, PID_Vd: %+8.3f, PID_Vq: %+8.3f, PID_Vqp: %+8.3f, PID_Vqi: %+8.3f, PID_Vqb: %+8.3f, UpdT: %4lu, FocT: %4lu \n",
							lpf_speed_.Output(),
							lpf_Id_.Output(),
							lpf_Iq_.Output(),
							pid_Vd_.Output(),
							pid_Vq_.Output(),
							pid_Vq_.OutputP(),
							pid_Vq_.OutputI(),
							pid_Vq_.OutputB(),
							upd_time,
							foc_time
					);
				}
				return true;
			});
		} while (ret);
	});
	drive_->AddTaskDisarmMotor();
	drive_->sched_.Run();
}

void MotorCtrlFOC::Speed()
{
	drive_->AddTaskArmMotor();

	drive_->sched_.AddTask([&](){
		bool ret = false;
		float rotor_speed_factor = 1.0f / (M_PI * 2 * drive_->GetPolePairs());
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
				float phase_speed = drive_->GetPhaseSpeed();
				float rotor_speed = phase_speed * rotor_speed_factor;

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
				lpf_speed_.DoFilter(rotor_speed);

				if (std::abs(Idq) > config_.i_trip_)
					return false;

				float Werr = config_.w_setpoint_ - rotor_speed;
				float Iq_out = pid_W_.Input(Werr, update_period);
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

				uint32_t upd_time = hrtimer.GetTimeElapsedMicroSec(drive_->t2_, drive_->t3_);
				uint32_t foc_time = hrtimer.GetTimeElapsedMicroSec(drive_->t2_, hrtimer.GetCounter());
				if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
					fprintf(stderr,
							"Speed: %6.2f Rev/sec, Idq_d: %+5.3f, Idq_q: %+6.3f, PID_Vd: %+6.3f, PID_Vq: %+6.3f, PID_VqP: %+6.3f, PID_VqI: %+6.3f, "
							"Werr: %+9.3f, PID_Wout: %+9.3f, PID_Wp: %+9.3f,  PID_Wi: %+9.3f,  \n",
							lpf_speed_.Output(),
							lpf_Id_.Output(),
							lpf_Iq_.Output(),
							pid_Vd_.Output(),
							pid_Vq_.Output(),
							pid_Vq_.OutputP(),
							pid_Vq_.OutputI(),
							Werr,
							Iq_out,
							pid_W_.OutputP(),
							pid_W_.OutputI()
					);
				}
				return true;
			});
		} while (ret);
	});
	drive_->AddTaskDisarmMotor();
	drive_->sched_.Run();
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

		config_.pid_w_kp_ = drive_->config_.inductance_;
		config_.pid_w_ki_ = drive_->config_.resistance_;
		pid_W_.SetGainP(config_.pid_w_kp_);
		pid_W_.SetGainI(config_.pid_w_ki_);
	});
	drive_->RunWaitForCompletion();
}
