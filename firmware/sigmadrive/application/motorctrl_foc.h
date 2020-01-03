/*
 * servo_drive.h
 *
 *  Created on: Oct 15, 2019
 *      Author: mstoilov
 */

#ifndef _MOTORCTRL_FOC_H_
#define _MOTORCTRL_FOC_H_

#include "motordrive.h"

class MotorCtrlFOC
{
public:
	struct Config {
		float pid_current_kp_ = 0.5;
		float pid_current_ki_ = 300;
		float pid_current_decay_ = 0.01;
		float pid_current_maxout_ = 45;

		float pid_w_kp_ = 0.5;
		float pid_w_ki_ = 300;
		float pid_w_decay_ = 0.01;
		float pid_w_maxout_ = 5;

		float control_bandwidth_ = 700; // Rad/Sec
		float vab_advance_factor_ = 12000; //1.35;
		float vq_bias_ = 0;
		float w_bias_ = 0;
		float id_alpha_ = 0.001;
		float iq_alpha_ = 0.001;
		float speed_disp_alpha_ = 0.005;
		float i_trip_ = 8.0;
		float iq_setpoint_ = 0.08;
		float w_setpoint_ = 0.0013;
		float ri_angle_ = 1.57;
		float spin_voltage_ = 3.5f;
		bool display_ = true;
		uint32_t enc_skip_updates = 2;
	};


public:
	MotorCtrlFOC(MotorDrive* drive);
	void Stop();
	void Torque();
	void Velocity();
	void Spin();
	void RunCalibrationSequence();
	float VelocitySetPoint(float revpersec);
	std::complex<float> GetElecRotation();
	float GetPhaseSpeedVector();
	float GetEncoderPeriod();

protected:
	void UpdateRotor();
	void RunDebugLoop();
	void StartDebugThread();
	bool WaitDebugDump();
	void SignalDumpTorque();
	void SignalDumpSpin();
	static void RunDebugLoopWrapper(void *ctx);

public:
	rexjson::property props_;

protected:
    enum Signals {
        SIGNAL_DEBUG_DUMP_TORQUE = 1u << 0,
        SIGNAL_DEBUG_DUMP_SPIN = 1u << 1,
    };


	osThreadId_t debug_thread_;
	Config config_;
	MotorDrive *drive_;
	LowPassFilter<float, float> lpf_Id_;
	LowPassFilter<float, float> lpf_Iq_;


	PidController<float> pid_Vd_;
	PidController<float> pid_Vq_;
	PidController<float> pid_W_;
	LowPassFilter<float, float> lpf_speed_disp_;
	std::complex<float> R_;
	float W_;
	uint32_t foc_time_ = 0;
	float Werr_ = 0;

};


#endif // _MOTORCTRL_COMPLEXFOC_H_
