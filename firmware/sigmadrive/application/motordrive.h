/*
 * servo_drive.h
 *
 *  Created on: Oct 15, 2019
 *      Author: mstoilov
 */

#ifndef _MOTOR_DRIVE_H_
#define _MOTOR_DRIVE_H_

#include <complex>
#include <math.h>
#include "adc.h"
#include "drv8323.h"
#include "iencoder.h"
#include "ipwmgenerator.h"
#include "lowpassfilter.h"
#include "rexjson/rexjsonproperty.h"
#include "scheduler.h"
#include "errorinfo.h"

#include "hrtimer.h"
extern HRTimer hrtimer;

/** The MotorDrive class is important class responsible
 * for driving the motor (applying the SVM voltage to the motor phases)
 *
 */
class MotorDrive {
public:
	struct Config {
		int32_t encoder_dir_ = 1;                           /**< Encoder direction: (1) - encoder increasing or (-1) encoder decreasing */
		uint32_t reset_hz_ = 35;                            /**< Rotor reset rate. The rotor is oscilated with this rate during the reset process */
		uint32_t pole_pairs_ = 7;                           /**< Motor pole pairs */
		int32_t pole_offset_ = 0;                           /**< The mechanical offset of the magnetic pole in encoder counts */
		uint32_t adc_full_scale_ = (1 << 12);               /**< ADC converter full scale */
		uint32_t display_div_ = 2999;
		uint32_t enc_skip_updates_ = 1;                     /**< How many update interrupts to skip, before initiating a new encoder update */
		uint32_t csa_gain_ = 10;                            /**< Current sensing amplifier gain */
		uint64_t pos_offset_ = 0;                           /**< Encoder position offset */
		bool svm_saddle_ = false;                           /**< Use space vector modulation (SVM) saddle form */
		float Vref_ = 3.3;                                  /**< ADC reference voltage */
		float max_modulation_duty_ = 0.95;                  /**< Maximum modulation duty */
		float Vbus_resistor_ratio_ = (47.0 + 3.3) / 3.3;    /**< Vbus divider ratio used to measure the Vbus voltage  */
		float reset_voltage_ = 3.5f;                        /**< Voltage used during the rotor reset process. @see reset_hz_ */
		float Rsense_ = 0.010f;                             /**< Shunt resistor value */
		float calib_v_ = 12;                                /**< Calibration voltage. This voltage is used for the calibration process */
		float calib_max_i_ = 4;                             /**< Calibration max allowed current. */
		float resistance_ = 3.12f;                          /**< Phase resistance */
		float inductance_ = 0.0033293f;                     /**< Phase inductance */
		float bias_alpha_ = 0.00035f;                       /**< RC filter alpha coefficient */
		float vbus_alpha_ = 0.2f;                           /**< Vbus filter alpha coefficient */
		float wenc_alpha_ = 0.85;                           /**< rotor velocity filter alpha coefficient */
		float trip_i_ = 8.0f;                               /**< Trip current, the max phase allowed current. */
		float trip_v_ = 60.0f;                              /**< Trip voltage, the max allowed voltage */
	};

public:
	void Run();
	void RunWaitForCompletion();
	void Abort();
	bool IsStarted();

	MotorDrive(uint32_t axis_idx, Drv8323* drv, Adc* adc, Adc* dma_adc, IEncoder *encoder, IPwmGenerator *pwm, uint32_t update_hz);
	virtual ~MotorDrive();

	void Attach();

	float CalculatePhaseCurrent(float adc_val, float adc_bias);
	float VoltageToDuty(float voltage, float v_bus);

	void UpdateRotor();
	void EstimateRotor();
	void UpdateCurrent();
	void UpdateBias();

	void SineSVM(float duty, const std::complex<float> &v_theta, float &duty_a, float &duty_b, float &duty_c);
	void SaddleSVM(float duty, const std::complex<float> &v_theta, float &duty_a, float &duty_b, float &duty_c);
	bool GetDutyTimings(float duty_a, float duty_b, float duty_c, uint32_t timing_period, uint32_t &timing_a, uint32_t &timing_b,
			uint32_t &timing_c);
	bool ApplyPhaseModulation(float v_duty, const std::complex<float> &v_theta);
	bool ApplyPhaseVoltage(float v_abs, const std::complex<float> &v_theta);
	bool ApplyPhaseVoltage(float v_alpha, float v_beta);
	bool ApplyPhaseDuty(float duty_a, float duty_b, float duty_c);
	bool RunUpdateHandler(const std::function<bool(void)> &update_handler);
	IEncoder* GetEncoder() const;
	uint32_t GetEncoderCPR() const;
	void SetEncoder(IEncoder *encoder);
	void SetResolutionBits(uint32_t resolution_bits = 16);
	int32_t GetEncoderDir() const;

protected:
	uint64_t GetEncoderPosition() const;

public:
	uint64_t GetEncoderMaxPosition() const;
	uint32_t GetEncoderPositionBits() const;
	uint32_t GetUpdateFrequency() const;
	float GetTimeSlice() const;
	uint32_t GetPolePairs() const;
	float GetBusVoltage() const;
	std::complex<float> GetRotorElecRotation();
	float GetRotorVelocity();
	float GetRotorVelocityPTS();
	float GetRotorElecVelocityPTS();
	int64_t GetRotorPositionError(uint64_t position, uint64_t target);
	uint64_t GetRotorPosition() const;
	uint32_t GetUpdateCounter() const                                   { return update_counter_; }
	void ResetUpdateCounter()                                           { update_counter_ = 0; }

	std::complex<float> GetPhaseCurrent() const;
	float GetPhaseCurrentMagnetude() const;
	void DefaultIdleTask();
	bool AbortOnPhaseCurrentViolation(float current);
	bool AbortOnBusVoltageViolation(float voltage);
	void RegisterRpcMethods(const std::string& prefix);
	rexjson::property GetPropertyMap();
	rexjson::property GetConfigPropertyMap();


	/*
	 * Temporary
	 */
	float GetMechanicalAngle(uint64_t enc_orientation) const;
	float GetElectricAngle(uint64_t enc_orientation) const;



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
	void AddTaskMeasureResistance(float seconds, float test_voltage);
	void AddTaskMeasureInductance(float seconds, float test_voltage, uint32_t test_hz);
	void AddTaskDetectEncoderDir();
	void RunTaskAlphaPoleSearch();
	void RunTaskRotateMotor(float angle, float speed, float voltage, bool dir);
	void RunSimpleTasks();
	void AddTaskCalibrationSequence(bool reset_rotor);
	void RunTaskResetRotorAndHold();
	float RunResistanceMeasurement(float seconds, float test_voltage);
	float RunInductanceMeasurement(float seconds, float test_voltage, uint32_t test_hz);
	void RunEncoderDisplayDebugInfo();
	void SchedulerRun();
	void SchedulerAbort();

	bool IsPrimary() { return (axis_idx_ == 1UL) ? true : false; }

public:
	Scheduler sched_;
	Config config_;
	uint32_t t_begin_ = 0;

protected:
	uint32_t update_counter_ = 0;
	uint32_t enc_cpr_ = 0;
	uint32_t enc_revolution_bits_ = 0;
	uint32_t enc_resolution_bits_ = 0;
	uint32_t enc_resolution_mask_ = 0;
	uint64_t enc_position_mask_ = 0;
	uint64_t enc_position_size_ = 0;
	uint64_t enc_position_size_half_ = 0;

	uint32_t update_hz_ = 0;
	uint32_t tim1_cnt_ = 0;
	uint32_t tim8_cnt_ = 0;
	int32_t tim1_tim8_offset_ = 0;
	int32_t tim8_tim1_offset_ = 0;

	float time_slice_;
	uint32_t enc_position_shiftright_ = 6;      /**< Shift right the encoder position value */
	uint32_t enc_position_shiftleft_ = 0;       /**< Shift right the encoder position value */

	bool run_simple_tasks_ = false;
	uint32_t axis_idx_ = 0;
	Adc* adc_ = nullptr;
	Adc* dma_adc_ = nullptr;
	IPwmGenerator *pwm_ = nullptr;
	ErrorInfo error_info_;
	IEncoder *encoder_ = nullptr;
	Drv8323 *drv_ = nullptr;

	/*
	 * Derived Data
	 */
	float phase_current_a_ = 0.0f;
	float phase_current_b_ = 0.0f;
	float phase_current_c_ = 0.0f;
	uint64_t Renc_ = 0;                         /**< Position of the rotor in enc counts */
	uint64_t Rencest_ = 0;                      /**< Estimated position of the rotor in enc counts */
	uint64_t Rencpred_ = 0;                     /**< Predicted position of the rotor in enc counts */
	LowPassFilter<float, float> lpf_bias_a;     /**< Low pass filter for phase A current bias */
	LowPassFilter<float, float> lpf_bias_b;     /**< Low pass filter for phase B current bias */
	LowPassFilter<float, float> lpf_bias_c;     /**< Low pass filter for phase C current bias */
	LowPassFilter<float, float> lpf_vbus_;      /**< Low pass filter for the Vbus voltage */
	LowPassFilter<float, float> lpf_Wenc_;      /**< Low pass filter for rotor velocity in enc counts per encoder_time_slice */
	std::complex<float> Iab_;                   /**< Phase current represented as a complex vector, where the real value is alpha current and the imag value is the beta current */
	float Iab_m_;                               /**< Phase current magnetude */
	std::complex<float> E_;                     /**< Orientation of the rotor in electrical radians converted to complex vector. */
	std::complex<float> Et_;                    /**< Orientation of the rotor in electrical radians converted to complex vector. */
	std::complex<float> Pa_ = std::polar<float>(1.0f, 0.0f);
	std::complex<float> Pb_ = std::polar<float>(1.0f, M_PI / 3.0 * 4.0);
	std::complex<float> Pc_ = std::polar<float>(1.0f, M_PI / 3.0 * 2.0);
};

#endif /* _MOTOR_DRIVE_H_ */
