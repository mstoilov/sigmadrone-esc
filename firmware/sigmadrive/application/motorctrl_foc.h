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
		float pid_current_kp_ = 12;
		float pid_current_ki_ = 4000;
		float pid_current_decay_ = 0.01;
		float pid_current_maxout_ = 45;

		float pid_w_kp_ = 12;
		float pid_w_ki_ = 4000;
		float pid_w_decay_ = 0.01;
		float pid_w_maxout_ = 1.1;

		float pid_p_kp_ = 2;
		float pid_p_ki_ = 0.0;
		float pid_p_decay_ = 0.01;
		float pid_p_maxout_ = 1;


		float control_bandwidth_ = 700; // Rad/Sec
		float vab_advance_factor_ = 12000;
		float vq_bias_ = 0;
		float w_bias_ = 0;
		float idq_disp_alpha_ = 0.01;
		float idq_alpha_ = 0.01;
		float speed_disp_alpha_ = 1;
		float i_trip_ = 8.0;
		float spin_voltage_ = 3.5f;
		bool display_ = true;
	};


public:
	MotorCtrlFOC(MotorDrive* drive);
	void Stop();
	void Torque();
	void Velocity();
	void Position();
	void Spin();
	void RunCalibrationSequence();
	float VelocitySetPoint(float revpersec);

protected:
	void UpdateRotor();
	void RunDebugLoop();
	void StartDebugThread();
	void SignalDumpTorque();
	void SignalDumpVelocity();
	void SignalDumpPosition();
	void SignalDumpSpin();
	static void RunDebugLoopWrapper(void *ctx);

public:
	rexjson::property props_;

protected:
    enum Signals {
		SIGNAL_DEBUG_DUMP_SPIN = 1u << 1,
		SIGNAL_DEBUG_DUMP_TORQUE = 1u << 2,
		SIGNAL_DEBUG_DUMP_VELOCITY = 1u << 3,
		SIGNAL_DEBUG_DUMP_POSITION = 1u << 4,
    };


	osThreadId_t debug_thread_;
	Config config_;
	MotorDrive *drive_;
	LowPassFilter<float, float> lpf_Id_;
	LowPassFilter<float, float> lpf_Iq_;
	LowPassFilter<float, float> lpf_Id_disp_;
	LowPassFilter<float, float> lpf_Iq_disp_;


	PidController<float> pid_Vd_;
	PidController<float> pid_Vq_;
	PidController<float> pid_W_;
	PidController<float> pid_P_;
	LowPassFilter<float, float> lpf_speed_disp_;
	std::complex<float> R_;
	uint32_t foc_time_ = 0;
	uint64_t enc_position_ = 0;
	uint64_t p_setpoint_ = 0;

	float Werr_ = 0;
	float Ierr_ = 0;
	float Rerr_ = 0;
	float iq_setpoint_ = 0.08;
	float w_setpoint_ = 0.0013;


};


#endif // _MOTORCTRL_COMPLEXFOC_H_
