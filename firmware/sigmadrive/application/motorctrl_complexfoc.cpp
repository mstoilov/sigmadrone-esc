#include "motorctrl_complexfoc.h"
#include "sdmath.h"
#include "uartrpcserver.h"

extern UartRpcServer rpc_server;

MotorCtrlComplexFOC::MotorCtrlComplexFOC(MotorDrive* drive)
	: drive_(drive)
	, lpf_Iabs_disp_(config_.iabs_alpha_disp_)
	, lpf_RIdot_(config_.ridot_alpha_)
	, lpf_RIdot_disp_(config_.ridot_alpha_disp_)
	, pid_current_arg_(config_.pid_current_kp_, config_.pid_current_ki_, 0, config_.pid_current_leak_)
{
	props_= rexjson::property_map({
		{"iabs_alpha", rexjson::property(
				&config_.iabs_alpha_disp_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_Iabs_disp_.SetAlpha(config_.iabs_alpha_disp_);
				})},
		{"ridot_alpha", rexjson::property(
				&config_.ridot_alpha_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_RIdot_.SetAlpha(config_.ridot_alpha_);
		})},
		{"ridot_disp_alpha", rexjson::property(
				&config_.ridot_alpha_disp_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
				[&](void*)->void {
					lpf_RIdot_disp_.SetAlpha(config_.ridot_alpha_disp_);
		})},

		{"pid_current_kp", rexjson::property(
				&config_.pid_current_kp_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_current_arg_.SetGainP(config_.pid_current_kp_);
		})},
		{"pid_current_ki", rexjson::property(
				&config_.pid_current_ki_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_current_arg_.SetGainI(config_.pid_current_ki_);
		})},
		{"pid_current_leak", rexjson::property(
				&config_.pid_current_leak_,
				rexjson::property_access::readwrite,
				[](const rexjson::value& v){},
				[&](void*)->void {
					pid_current_arg_.SetLeakRate(config_.pid_current_leak_);
		})},
		{"ri_angle", &config_.ri_angle_},
		{"spin_voltage", &config_.spin_voltage_},
		{"drive", rexjson::property({drive_->props_})},
	});

	rpc_server.add("cfoc.start", rexjson::make_rpc_wrapper(this, &MotorCtrlComplexFOC::RunSpinTasks, "void ServoDrive::RunSpinTasks()"));
	rpc_server.add("cfoc.stop", rexjson::make_rpc_wrapper(this, &MotorCtrlComplexFOC::Stop, "void ServoDrive::Stop()"));
}

void MotorCtrlComplexFOC::Stop()
{
	drive_->sched_.Abort();
}

void MotorCtrlComplexFOC::DumpDebugData(float Rarg, float Iarg, float Iabs)
{
	float diff = acosf(lpf_RIdot_disp_.Output());

	if (Rarg < 0.0f)
		Rarg += M_PI * 2.0f;
	if (Iarg < 0.0f)
		Iarg += M_PI * 2.0f;
	float speed = (asinf(drive_->lpf_speed_.Output()) * drive_->GetUpdateFrequency())/(M_PI*2.0 * drive_->GetPolePairs());
//	fprintf(stderr, "Vbus: %4.2f, RPM: %6.1f, arg(R): %6.1f, arg(I): %6.1f, abs(I): %6.3f, DIFF: %+7.1f (%+4.2f), Pid.Out: %8.5f (%5.2f)\n",
//			drive_->GetBusVoltage(),
//			speed, Rarg / M_PI * 180.0f, Iarg / M_PI * 180.0f, lpf_Iabs_disp_.Output(),
//			diff / M_PI * 180.0f, lpf_RIdot_disp_.Output(), pid_current_arg_.Output(), pid_current_arg_.Output() / M_PI * 180.0f);
}

void MotorCtrlComplexFOC::RunSpinTasks()
{
	drive_->AddTaskArmMotor();
	drive_->AddTaskResetRotor();
	if (drive_->GetEncoderDir() == 0)
		drive_->AddTaskDetectEncoderDir();
	drive_->sched_.AddTask([&](){
		bool ret = false;
		uint32_t display_counter = 0;
		do {
			ret = drive_->RunUpdateHandler([&]()->bool {
				std::complex<float> I = drive_->GetPhaseCurrent();
				std::complex<float> rotor = drive_->GetElecRotation();
				float Iarg = std::arg(I);
				float Iabs = std::abs(I);
				std::complex<float> Inorm = std::polar<float>(1.0f, Iarg);
				float ri_dot = sdmath::dot(rotor, Inorm);
				lpf_RIdot_.DoFilter(ri_dot);
				lpf_RIdot_disp_.DoFilter(ri_dot);
				lpf_Iabs_disp_.DoFilter(Iabs);
				pid_current_arg_.Input(lpf_RIdot_.Output(), 1.0f / drive_->GetUpdateFrequency());
				std::complex<float> ri_vec = std::polar<float>(1.0f, config_.ri_angle_ + pid_current_arg_.Output());
				drive_->ApplyPhaseVoltage(config_.spin_voltage_, rotor * ri_vec, drive_->GetBusVoltage());
				if ((display_counter++ % 13) == 0) {
					DumpDebugData(std::arg(rotor), Iarg, Iabs);
				}
				return true;
			});
		} while (ret);
	});

	drive_->sched_.Run();
}
