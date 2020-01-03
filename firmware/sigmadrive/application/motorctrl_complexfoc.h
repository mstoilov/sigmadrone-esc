/*
 * servo_drive.h
 *
 *  Created on: Oct 15, 2019
 *      Author: mstoilov
 */

#ifndef _MOTORCTRL_COMPLEXFOC_H_
#define _MOTORCTRL_COMPLEXFOC_H_

#include "motordrive.h"

class MotorCtrlComplexFOC
{
public:
	struct Config {
		float spin_voltage_ = 0.4f;
		float ridot_alpha_ = 0.15f;
		float ridot_alpha_disp_ = 0.01f;
		float iabs_alpha_disp_ = 0.001f;
		float ri_angle_ = 1.57;
		float pid_current_kp_ = 0.4;
		float pid_current_ki_ = 500;
		float pid_current_leak_ = 0.99;
		uint32_t enc_skip_updates = 2;
	};


public:
	MotorCtrlComplexFOC(MotorDrive* drive);
	void Stop();
	void RunSpinTasks();
	std::complex<float> GetElecRotation();
	float GetPhaseSpeedVector();
	float GetEncoderPeriod();
	void UpdateRotor();

public:
	rexjson::property props_;
	Config config_;
	MotorDrive *drive_;
	LowPassFilter<float, float> lpf_Iabs_disp_;
	LowPassFilter<float, float> lpf_RIdot_;
	LowPassFilter<float, float> lpf_RIdot_disp_;
	PidController<float> pid_current_arg_;
	std::complex<float> R_;
	float W_;

};


#endif // _MOTORCTRL_COMPLEXFOC_H_
