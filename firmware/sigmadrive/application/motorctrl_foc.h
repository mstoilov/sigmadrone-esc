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
        float pid_current_kp_ = 1.5;
        float pid_current_ki_ = 500;
        float pid_current_decay_ = 0.01;
        float pid_current_maxout_ = 45;

        float pid_w_kp_ = 0.0075;
        float pid_w_ki_ = 3;
        float pid_w_decay_ = 0.01;
        float pid_w_maxout_ = 3.0;


        float pid_p_kp_ = 2;
        float pid_p_ki_ = 0.0;
        float pid_p_decay_ = 0.01;
        float pid_p_maxout_ = 1;


        float control_bandwidth_ = 700; // Rad/Sec
        float vab_advance_factor_ = 0;//1.5;
        float vq_bias_ = 0;
        float w_bias_ = 0;
        float idq_alpha_ = 0.75;
        float i_trip_ = 8.0;
        float spin_voltage_ = 3.5f;
        bool display_ = true;
    };


public:
    MotorCtrlFOC(MotorDrive* drive);
    void Stop();
    void ModeClosedLoopTorque();
    void ModeClosedLoopVelocity();
    void ModeClosedLoopPosition();
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

    PidController<float> pid_Vd_;
    PidController<float> pid_Vq_;
    PidController<float> pid_W_;
    PidController<float> pid_P_;
    uint32_t foc_time_ = 0;
    uint64_t enc_position_ = 0;
    uint64_t target_ = 0;

    float Werr_ = 0;
    float Ierr_ = 0;
    float Rerr_ = 0;
    float iq_setpoint_ = 0.055;
    float velocity_ = 65535;            /**< Movement velocity as encoder counts per second */
};


#endif // _MOTORCTRL_COMPLEXFOC_H_
