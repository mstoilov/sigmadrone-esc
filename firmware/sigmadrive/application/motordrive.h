/*
 * servo_drive.h
 *
 *  Created on: Oct 15, 2019
 *      Author: mstoilov
 */

#ifndef _MOTOR_DRIVE_H_
#define _MOTOR_DRIVE_H_

#include <complex>
#include "iencoder.h"
#include "ipwmgenerator.h"
#include "lowpassfilter.h"
#include "property.h"
#include "scheduler.h"
#include "pidcontroller.h"

#include "hrtimer.h"
extern HRTimer hrtimer;

class MotorDrive {
public:
	struct SampledData {
		int32_t injdata_[3];
		float phase_current_a_;
		float phase_current_b_;
		float phase_current_c_;
		int32_t vbus_ = 0;
		float theta_ = 0.0f;
		uint32_t update_counter_ = 0;
	};

	struct Config {
		int32_t encoder_dir_ = 0;				// (1) - encoder increasing or (-1) encoder decreasing
		uint32_t reset_hz_ = 35;
		uint32_t pole_pairs = 7;
		uint32_t adc_full_scale = (1<<12);
		bool svm_saddle_ = false;
		float Vref_ = 3.3;
		float Vbus_resistor_ratio_ = (47.0 + 3.3) / 3.3;
		float reset_voltage_ = 0.4f;
		float spin_voltage_ = 0.4f;
		float csa_gain_ = 10.0f;
		float Rsense_ = 0.010f;
		float resistance_ = 0.05f;
		float inductance_ = 5.56e-05f;
		float bias_alpha_ = 0.00035f;
		float abc_alpha_ = 0.25f;
		float i_alpha_ = 0.25f;
		float rotor_alpha_ = 1.0f;
		float speed_alpha_ = 0.3f;
		float ridot_alpha_ = 0.15f;
		float ridot_disp_alpha_ = 0.01f;
		float iabs_alpha_ = 0.001f;
		float ri_angle_ = 1.54;
	};

	void Start();
	void Stop();
	bool IsStarted();

	MotorDrive(IEncoder* encoder, IPwmGenerator *pwm, uint32_t update_hz);
	virtual ~MotorDrive();

	void Attach();
	void IrqUpdateCallback();

	float CalculatePhaseCurrent(float adc_val, float adc_bias);
	float VoltageToDuty(float voltage, float v_bus);

	void UpdateRotor();
	void UpdateVbus();
	void UpdateCurrent();

	void SineSVM(float duty, const std::complex<float>& v_theta, float& duty_a, float& duty_b, float& duty_c);
	void SaddleSVM(float duty, const std::complex<float>& v_theta, float& duty_a, float& duty_b, float& duty_c);
	bool GetDutyTimings(float duty_a, float duty_b, float duty_c, uint32_t timing_period, uint32_t& timing_a, uint32_t& timing_b, uint32_t& timing_c);
	bool ApplyPhaseVoltage(float v_abs, const std::complex<float>& v_theta, float vbus);
	bool ApplyPhaseDuty(float duty_a, float duty_b, float duty_c);
	bool RunUpdateHandler(const std::function<bool(void)>& update_handler);
	void RunSimpleTasks();
	void RunSpinTasks();
	float RunResistanceMeasurement(float seconds, float test_voltage, float max_current);
	float RunResistanceMeasurementOD(float seconds, float test_current, float max_voltage);
	float RunInductanceMeasurementOD(int num_cycles, float voltage_low, float voltage_high);
	IEncoder* GetEncoder() const;
	void SetEncoder(IEncoder* encoder);
	int32_t GetEncoderDir() const;
	uint32_t GetUpdateFrequency() const;
	uint32_t GetPolePairs() const;
	float GetBusVoltage() const;
	std::complex<float> GetPhaseCurrent() const;
	std::complex<float> GetElecRotation() const;

	/*
	 * UpdateHandlers
	 */
	bool RunUpdateHandlerRotateMotor(float angle, float speed, float voltage, bool dir);


	/*
	 * Scheduler Tasks
	 */
	void AddTaskRotateMotor(float angle, float speed, float voltage, bool dir);
	void AddTaskArmMotor();
	void AddTaskDisarmMotor();
	void AddTaskResetRotorWithParams(float reset_voltage, uint32_t reset_hz, bool reset_encoder = true);
	void AddTaskResetRotor();
	void AddTaskDetectEncoderDir();
	void RunTaskAphaPoleSearch();
	void RunTaskRotateMotor(float angle, float speed, float voltage, bool dir);

	void SchedulerRun();
	void SchedulerAbort();

public:
	Scheduler sched_;


public:
	rexjson::property props_;

public:
	std::complex<float> Pa_;
	std::complex<float> Pb_;
	std::complex<float> Pc_;

	uint32_t update_hz_;
	uint32_t wait_timeout_ = (uint32_t)-1;	// Forever
	uint32_t t1_ = 0;
	uint32_t signal_time_ms_ = 0;
	bool run_simple_tasks_ = false;
	bool update_handler_active_ = false;
	IEncoder *encoder_ = nullptr;
	IPwmGenerator *pwm_ = nullptr;
	SampledData data_;
	Config config_;

/*
 * Derived Data
 */
	LowPassFilter<float, float> lpf_bias_a;
	LowPassFilter<float, float> lpf_bias_b;
	LowPassFilter<float, float> lpf_bias_c;
	LowPassFilter<float, float> lpf_a;
	LowPassFilter<float, float> lpf_b;
	LowPassFilter<float, float> lpf_c;
	LowPassFilter<std::complex<float>, float> lpf_e_rotor_;
	LowPassFilter<std::complex<float>, float> lpf_Iab_;
	LowPassFilter<std::complex<float>, float> lpf_Idq_;
	LowPassFilter<float, float> lpf_Iabs_;
	LowPassFilter<float, float> lpf_RIdot_;
	LowPassFilter<float, float> lpf_RIdot_disp_;
	LowPassFilter<float, float> lpf_vbus_;
	LowPassFilter<float, float> lpf_speed_;
	PidController<float> pid_current_arg_;
};

#endif /* _MOTOR_DRIVE_H_ */
