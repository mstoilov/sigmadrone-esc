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
        float pid_current_kp_ = 3;    /* 3 */   /**< Current PID regulator proportional gain */
        float pid_current_ki_ = 90;   /* 3120*/ /**< Current PID regulator integral gain */
        float pid_current_maxout_ = 45;         /**< Current PID regulator integral output limit */

        float pid_w_kp_ = 0.01;                 /**< Velocity PID regulator proportional gain */
        float pid_w_ki_ = 0.7;                  /**< Velocity PID regulator integral gain */
        float pid_w_maxout_ = 25.0;             /**< Velocity PID regulator integral output limit */

        float pid_p_kp_ = 4;                    /**< Position PID regulator proportional gain */
        float pid_p_ki_ = 10;                   /**< Position PID regulator integral gain */
        float pid_p_kd_ = 0.05;                 /**< Position PID regulator differential gain */
        float pid_p_maxout_ = 5;                /**< Position PID regulator integral output limit */

        float control_bandwidth_ = 700;         /**< Constant used in the calculations of the closed loop PID regulators gains. Rad/Sec */
        float vab_advance_factor_ = 1.5;        /**< Magnetic field advance factor. The V_ab will be advanced proportional to the rotor variable speed and this constant  */
        float vq_bias_ = 0;                     /**< Bias for the q-voltage (Vq) PID regulator */
        float w_bias_ = 0;                      /**< Bias for the velocity PID (W) regulator */
        float idq_alpha_ = 0.75;                /**< DQ-current filter alpha coefficient */
        float i_trip_ = 8.0;                    /**< Max allowed current. If the current gets bigger than that it will cause trip violation */
        bool display_ = true;                   /**< Display mode on/off */
    };


public:
    MotorCtrlFOC(MotorDrive* drive);
    void Stop();
    void ModeClosedLoopTorque();
    void ModeClosedLoopVelocity();
    void ModeClosedLoopPosition();
    void ModeClosedLoopPosition2();
    void ModeSpin();
    uint64_t MoveToPosition(uint64_t position);
    uint64_t MoveRelative(int64_t position);
    void RunCalibrationSequence(bool reset_rotor);
    float VelocityRPS(float revpersec);
    rexjson::property GetPropertyMap();
    void RegisterRpcMethods();

protected:
    void UpdateRotor();
    void RunDebugLoop();
    void StartDebugThread();
    void SignalDumpTorque();
    void SignalDumpVelocity();
    void SignalDumpPosition();
    void SignalDumpSpin();
    static void RunDebugLoopWrapper(void *ctx);

protected:
    enum Signals {
        SIGNAL_DEBUG_DUMP_SPIN = 1u << 1,       /**< Signal the debug display thread to run and dump spin mode info */
        SIGNAL_DEBUG_DUMP_TORQUE = 1u << 2,     /**< Signal the debug display thread to run and dump closed loop torque mode info */
        SIGNAL_DEBUG_DUMP_VELOCITY = 1u << 3,   /**< Signal the debug display thread to run and dump closed loop velocity mode info */
        SIGNAL_DEBUG_DUMP_POSITION = 1u << 4,   /**< Signal the debug display thread to run and dump closed loop position mode info */
    };


    osThreadId_t debug_thread_;                 /**< Debug display info thread */
    Config config_;                             /**< Structure holding all configuration parameters */
    MotorDrive *drive_;                         /**< Pointer to the MotorDrive structure */

    /*
     * Filters
     */
    LowPassFilter<float, float> lpf_Id_;        /**< Low-pass filter for d-current (Id) */
    LowPassFilter<float, float> lpf_Iq_;        /**< Low-pass filter for q-current (Iq) */

    /*
     * PID regulators
     */
    PidController<float> pid_Id_;               /**< PID regulator controlling the d-current (Id) */
    PidController<float> pid_Iq_;               /**< PID regulator controlling the q-current (Iq) */
    PidController<float> pid_W_;                /**< PID regulator controlling the rotor velocity (W) */
    PidController<float> pid_P_;                /**< PID regulator controlling the target position */
    float Ierr_ = 0;                            /**< Q-current error. Used as input for the Iq PID regulator */
    float Werr_ = 0;                            /**< Velocity error. Used as input for the velocity PID regulator */
    float Perr_ = 0;                            /**< Rotor position error. Used as input for the position PID regulator */

    uint64_t target_ = 0;                       /**< Target position used in closed loop position mode */
    float velocity_ = 65535;                    /**< Movement velocity in encoder counts per second used in velocity loop and position loop modes */
    float q_current_ = 0.075;                   /**< Q-current used for torque loop mode */
    float spin_voltage_ = 3.5f;                 /**< Voltage used for the spin mode */
    uint32_t foc_time_ = 0;                     /**< The time it takes to run the FOC calculations in micro-seconds */
};


#endif // _MOTORCTRL_FOC_H_
