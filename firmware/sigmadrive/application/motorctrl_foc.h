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
		float pid_current_kp_ = 21;
		float pid_current_ki_ = 3000;
		float pid_current_decay_ = 0.01;
		float pid_current_maxout_ = 45;

		float pid_w_kp_ = 0.0005;
		float pid_w_ki_ = 0.1;
		float pid_w_decay_ = 0.01;
		float pid_w_maxout_ = 5;

		float control_bandwidth_ = 1200; // Rad/Sec
		float vab_advance_factor_ = 1.35;
		float vq_bias_ = 0;
		float w_bias_ = 0;
		float id_alpha_ = 0.3;
		float iq_alpha_ = 0.01;
		float speed_disp_alpha_ = 0.1;
		float i_trip_ = 8.0;
		float iq_setpoint_ = 0.08;
		float w_setpoint_ = 2; // Rev/Sec
		float ri_angle_ = 1.57;
		bool display_ = true;
	};


public:
	MotorCtrlFOC(MotorDrive* drive);
	void Stop();
	void Torque();
	void Speed();
	void RunCalibrationSequence();

public:
	rexjson::property props_;
	Config config_;
	MotorDrive *drive_;
	LowPassFilter<float, float> lpf_Id_;
	LowPassFilter<float, float> lpf_Iq_;


	PidController<float> pid_Vd_;
	PidController<float> pid_Vq_;
	PidController<float> pid_W_;
	LowPassFilter<float, float> lpf_speed_;

};


#endif // _MOTORCTRL_COMPLEXFOC_H_
