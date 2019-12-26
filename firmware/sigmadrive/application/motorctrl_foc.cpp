#include "motorctrl_foc.h"
#include "sdmath.h"
#include "uartrpcserver.h"

extern UartRpcServer rpc_server;

MotorCtrlFOC::MotorCtrlFOC(MotorDrive* drive)
	: drive_(drive)
	, lpf_Idq_(config_.idq_alpha_)
	, pid_Vd_(config_.pid_current_kp_, config_.pid_current_ki_, 0, config_.pid_current_leak_)
	, pid_Vq_(config_.pid_current_kp_, config_.pid_current_ki_, 0, config_.pid_current_leak_)
{
	props_= rexjson::property_map({
		{"idq_alpha", rexjson::property(
				&config_.idq_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_Idq_.SetAlpha(config_.idq_alpha_);
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
		{"pid_current_leak", rexjson::property(
				&config_.pid_current_leak_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_Vd_.SetLeakRate(config_.pid_current_leak_);
					pid_Vq_.SetLeakRate(config_.pid_current_leak_);
		})},
		{"pid_current_maxout", rexjson::property(
				&config_.pid_current_maxout_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_Vd_.SetMaxIntegralOutput(config_.pid_current_maxout_);
					pid_Vq_.SetMaxIntegralOutput(config_.pid_current_maxout_);
		})},
		{"iq_set", &config_.iq_set_},
		{"i_trip", &config_.i_trip_},
		{"vab_advance_factor", &config_.vab_advance_factor_},
	});

	rpc_server.add("foc.torque", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Torque, "void MotorCtrlFOC::Torque()"));
	rpc_server.add("foc.stop", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Stop, "void MotorCtrlFOC::Stop()"));
	rpc_server.add("foc.calibration", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::RunCalibrationSequence, "void MotorCtrlFOC::RunCalibrationSequence()"));


	pid_Vd_.SetMaxIntegralOutput(config_.pid_current_maxout_);
	pid_Vq_.SetMaxIntegralOutput(config_.pid_current_maxout_);
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
		uint32_t display_counter = 0;
		drive_->data_.update_counter_ = 0;
		do {
			ret = drive_->RunUpdateHandler([&]()->bool {
				std::complex<float> Iab = drive_->GetPhaseCurrent();
				std::complex<float> R = drive_->GetElecRotation();

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
				lpf_Idq_.DoFilter(Idq);

				if (std::abs(Idq) > config_.i_trip_)
					return false;

				pid_Vd_.Input(0.0f - lpf_Idq_.Output().real(), 1.0f / drive_->GetUpdateFrequency());
				pid_Vq_.Input(config_.iq_set_ - lpf_Idq_.Output().imag(), 1.0f / drive_->GetUpdateFrequency());


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
				V_ab *= std::polar<float>(1.0f, config_.vab_advance_factor_ * drive_->GetPhaseSpeed()/drive_->GetUpdateFrequency());

				/*
				 * Apply the voltage timings
				 */
				drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag(), drive_->GetBusVoltage());

				uint32_t upd_time = hrtimer.GetTimeElapsedMicroSec(drive_->t2_, drive_->t3_);
				uint32_t foc_time = hrtimer.GetTimeElapsedMicroSec(drive_->t2_, hrtimer.GetCounter());
				if (display_counter++ % 73 == 0) {
					fprintf(stderr, "Speed: %6.2f Rev/sec, Idq_d: %+5.3f, Idq_q: %+5.3f, PID_Vd: %+5.3f, PID_Vq: %+5.3f, UpdT: %4lu, FocT: %4lu \n",
							drive_->GetPhaseSpeed() / (M_PI * 2 * drive_->GetPolePairs()),
							lpf_Idq_.Output().real(),
							lpf_Idq_.Output().imag(),
							pid_Vd_.Output(),
							pid_Vq_.Output(),
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

void MotorCtrlFOC::RunCalibrationSequence()
{
	drive_->AddTaskCalibrationSequence();
	drive_->sched_.AddTask([&](){
		config_.pid_current_kp_ = config_.control_bandwidth_ * drive_->config_.inductance_;
		config_.pid_current_ki_ = config_.control_bandwidth_ * drive_->config_.resistance_;
	});
	drive_->RunWaitForCompletion();
}
