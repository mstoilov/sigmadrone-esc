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
		float pid_current_ki_ = 5200;
		float pid_current_leak_ = 0.999;
		float pid_current_maxout_ = 40;
		float control_bandwidth_ = 1200; // Rad/Sec
		float vab_advance_factor_ = 1.35;
		float idq_alpha_ = 0.01;
		float i_trip_ = 5.0;
		float iq_set_ = 0.2;
	};


public:
	MotorCtrlFOC(MotorDrive* drive);
	void Stop();
	void Torque();
	void RunCalibrationSequence();

public:
	rexjson::property props_;
	Config config_;
	MotorDrive *drive_;
	LowPassFilter<std::complex<float>, float> lpf_Idq_;


	PidController<float> pid_Vd_;
	PidController<float> pid_Vq_;

};


#endif // _MOTORCTRL_COMPLEXFOC_H_
