/*
 * servo_drive.h
 *
 *  Created on: Oct 15, 2019
 *      Author: mstoilov
 */

#ifndef _MOTORCTRL_FOC_H_
#define _MOTORCTRL_FOC_H_

#include <vector>
#include "rexjson/rexjsonproperty.h"
#include "motordrive.h"
#include "pidcontroller.h"
#include "picontroller.h"
#include "pcontroller.h"
#include "velocityprofiler/trapezoidprofile.h"
#include "ring.h"



class MotorCtrlFOC
{
protected:
	struct Config {
		float pid_current_kp_ = 1.5;            /**< Current PID regulator proportional gain */
		float pid_current_ki_ = 300;            /**< Current PID regulator integral gain */
		float pid_current_maxout_ = 60;         /**< Current PID regulator output limit */

		float pid_w_kp_ = 0.70;                 /**< Velocity PID regulator proportional gain */
		float pid_w_ki_ = 220;                  /**< Velocity PID regulator integral gain */
		float pid_w_maxout_ = 15.0;             /**< Velocity PID regulator output limit */

		float pid_p_kp_ = 20;                   /**< Position PID regulator proportional gain */
		float pid_p_maxout_ = 240;              /**< Position PID regulator output limit */

		float tau_ratio_ = 2.5;                 /**< Constant used in the calculations of the closed loop PID regulator gains. Tratio = Tcl/Tp as per https://www.youtube.com/watch?v=3viD5ij60EI */
		float vab_advance_factor_ = 1.5;        /**< Magnetic field advance factor. The V_ab will be advanced proportional to the rotor variable speed and this constant  */
		float vq_bias_ = 0;                     /**< Bias for the q-voltage (Vq) PID regulator */
		float w_bias_ = 0;                      /**< Bias for the velocity PID (W) regulator */
		float crash_current_ = 25;              /**< If the current is above this value the crash detection will kick in */
		uint32_t crash_backup_ = 100000;        /**< If crash is detected, backup from that point by the given encoder counts */
		uint32_t crash_backup_speed_ = 250000;  /**< How fast to backup from the crash point */
		uint32_t pulse_enc_counts_ = 256;       /**< Encoder counts per pulse */
		bool display_ = false;                  /**< Display mode on/off */
	};

	struct MotionStats {
		float V1;
		float V2;
		float V;
		float A;
		float T1;
		float T2;
		float T;
		float S2;
		float S;
		int32_t lpf_V;
	};

public:
	MotorCtrlFOC(MotorDrive* drive, std::string axis_id, TIM_HandleTypeDef* htim_pulse);
	void Stop();
	void ModeClosedLoopTorque();
	void ModeClosedLoopVelocity();
	void SimpleModeClosedLoopPosition();
	void ModeClosedLoopPositionStream();
	void ModeClosedLoopPositionTrajectory();
	void StopMove();
	void ModeSpin();

	int64_t GetTarget() const;						// Get the current target position. Relevant only in Closed Loop Position mode.
	void SetTarget(const int64_t position);			// Set the current target position. Relevant only in Closed Loop Position mode.
	uint64_t MoveToPosition(uint64_t position);
	uint64_t MoveToPositionParams(uint64_t target, uint32_t v, uint32_t acc, uint32_t dec);
	uint64_t MoveRelative(int64_t relative);
	void MoveTrapezoid(int64_t relative, uint32_t v, uint32_t acc, uint32_t dec);
	uint64_t MoveRelativeParams(int64_t relateive, uint32_t v, uint32_t acc, uint32_t dec);

	uint64_t SimpleMoveToPosition(uint64_t position);
	uint64_t SimpleMoveRelative(int64_t relative);


	void UpdatePulseTimerPeriod();
	void RunCalibrationSequence(bool reset_rotor);
	float VelocityRPS(float revpersec);

	rexjson::property GetPropertyMap();
	rexjson::property GetConfigPropertyMap();
	void RegisterRpcMethods();
	/**
	 * @brief Push a velocity spec
	 * 
	 * Important: The velocity needs to be converted to [ec/timeslice]
	 * by the closed loop algorithm like this: 
	 * velocity_per_timeslice = velocity / update_hz
	 * 
	 * @param time 			in timeslices count
	 * @param velocity 		[ec/s]
	 * @param position 		[ec]
	 */
	void PushStreamPoint(int64_t time, int64_t velocity, int64_t position);
	void PushStreamPointV(std::vector<int64_t> v);
	void MoveRelativePulseStream(int64_t relative);
	void PulseStreamPush(uint32_t dir, uint32_t pulse, uint32_t seq);
	void PulseStreamFlush();
	void Go();
	uint32_t PulseCallback();

	/**
	 * @brief Specify a pulse stream to be executed in the control loop
	 * 
	 * The pulse stream increases or decreases the target position.
	 * Every byte holds 4 pulses, 2bits per pulse
	 * Pulse spec:
	 * Bit 0 - Pulse
	 * Bit 1 - Direction
	 * Example: 
	 *    00 - No pulse
	 *    01 - Pulse forward
	 *    10 - No pulse
	 *    11 - Pulse backward
	 * 
	 * @param v 
	 */
	void PulseStream(std::vector<uint8_t> v);

	void UpdateRotor();
	void RunMonitorLoop();
	void StartMonitorThread();
	void SignalDumpTorque();
	void SignalDumpVelocity();
	void SignalDumpPosition();
	void SignalDumpTrajectory();
	void SignalDumpSpin();
	void SignalCrashDetected();
	void SignalRelatedCrashDetected();
	void Capture();

	static void RunDebugLoopWrapper(void *ctx);

	rexjson::array GetCapturedPosition();
	rexjson::array GetCapturedVelocity();
	rexjson::array GetCapturedVelocitySpec();
	rexjson::array GetCapturedCurrent();

	enum Signals {
		SIGNAL_DEBUG_DUMP_SPIN = 1u << 1,       /**< Signal the debug display thread to run and dump spin mode info */
		SIGNAL_DEBUG_DUMP_TORQUE = 1u << 2,     /**< Signal the debug display thread to run and dump closed loop torque mode info */
		SIGNAL_DEBUG_DUMP_VELOCITY = 1u << 3,   /**< Signal the debug display thread to run and dump closed loop velocity mode info */
		SIGNAL_DEBUG_DUMP_POSITION = 1u << 4,   /**< Signal the debug display thread to run and dump closed loop position mode info */
		SIGNAL_DEBUG_DUMP_TRAJECTORY = 1u << 5, /**< Signal the debug display thread to run and dump closed loop position mode trajectory info */
		SIGNAL_CRASH_DETECTED = 1u <<  6,       /**< Signal the monitoring loop crash detection */
		SIGNAL_RELATEDCRASH_DETECTED = 1u <<  7 /**< Signal the monitoring loop crash detection */
	};

	enum CaptureMode {
		CAPTURE_NONE = 0,
		CAPTURE_POSITION = 1,
		CAPTURE_VELOCITY = 2,
		CAPTURE_VELOCITYSPEC = 4,
		CAPTURE_CURRENT = 8,
	};

	Config config_;                             /**< Structure holding all configuration parameters */
	MotorDrive *drive_;                         /**< Pointer to the MotorDrive structure */
	std::string axis_id_;                       /**< Axis identifying this motor control */

protected:
	osThreadId_t monitor_thread_;                 /**< Debug display info thread */

	/*
	 * Filters
	 */
	float lpf_Id_;                              /**< d-current (Id) */
	float lpf_Iq_;                              /**< q-current (Iq) */

	/*
	 * PID regulators
	 */
	PIController<float> pid_Id_;                /**< PID regulator controlling the d-current (Id) */
	PIController<float> pid_Iq_;                /**< PID regulator controlling the q-current (Iq) */
	PIDController<float> pid_W_;                /**< PID regulator controlling the rotor velocity (W) */
	PController<float> pid_P_;                  /**< PID regulator controlling the target position */
	float Ierr_ = 0;                            /**< Q-current error. Used as input for the Iq PID regulator */
	float Werr_ = 0;                            /**< Velocity error. Used as input for the velocity PID regulator */
	float Wraderr_ = 0;                         /**< Velocity error in rads. Used as input for the velocity PID regulator */
	float Perr_ = 0;                            /**< Rotor position error. Used as input for the position PID regulator */
	uint64_t target_ = 0;                       /**< Target position used in closed loop position mode */
	float velocity_ = 1200000;                  /**< Movement velocity in encoder counts per second used in velocity loop and position loop modes */
	float acceleration_ = 6000000;              /**< Movement acceleration [counts/s^2] */
	float deceleration_ = 2000000;              /**< Movement deceleration [counts/s^2] */
	float q_current_ = 0.075;                   /**< Q-current used for torque loop mode */
	float spin_voltage_ = 3.0f;                 /**< Voltage used for the spin mode */
	uint32_t foc_time_ = 0;                     /**< The time it takes to run the FOC calculations in micro-seconds */
	uint32_t pulse_counter_ = 0;                /**< Number of pulse interrupts. This var is decremented at each period until it reaches 0 and then the pulse timer is deactivated */
	uint32_t pulse_direction_ = 0;              /**< 0 - Move forward, 1 - Move backward */
	uint32_t pulses_per_sec_ = 256;             /**< The pulse trains speed, i.e. how many pulses to generate per second */
	TIM_HandleTypeDef* htim_pulse_;             /**< The timer responsible for generating pulses */
	MotionStats ms_;                            /**< Holds the motion progress values, like time, velocity, position */

	Ring<std::vector<int64_t>, 512> velocity_stream_; /**< (T, V, P) T in counts of update periods, V in enc. counts per sec, P in enc. counts */
	using PulseStreamType = uint8_t;
	Ring<PulseStreamType, 2500> pulse_stream_;                /**< Two bits per pulse. Bit 0 is pulse, Bit 1 is dir. Values: 0 - No pulse, 1 - Pulse Forward, 2 - Not used, 3 - Pulse Backward */
	std::vector<float> capture_position_;
	std::vector<float> capture_velocity_;
	std::vector<float> capture_current_;
	std::vector<float> capture_velocityspec_;
	size_t capture_interval_;
	size_t capture_mode_;
	size_t capture_capacity_;
	bool go_;

private:
	PulseStreamType scratch_;
};


#endif // _MOTORCTRL_FOC_H_
