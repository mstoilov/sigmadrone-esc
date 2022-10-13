
#include <string.h>

#include "motorctrl_foc.h"
#include "sdmath.h"
#include "uartrpcserver.h"
#include "minasa4encoder.h"

extern UartRpcServer rpc_server;

MotorCtrlFOC::MotorCtrlFOC(MotorDrive* drive, std::string axis_id)
    : drive_(drive)
    , axis_id_(axis_id)
    , pid_Id_(config_.pid_current_kp_, config_.pid_current_ki_, config_.pid_current_maxout_)
    , pid_Iq_(config_.pid_current_kp_, config_.pid_current_ki_, config_.pid_current_maxout_)
    , pid_W_(config_.pid_w_kp_, config_.pid_w_ki_, 0, 1, config_.pid_w_maxout_, 0)
    , pid_P_(config_.pid_p_kp_, config_.pid_p_maxout_)
{
    StartDebugThread();
    RegisterRpcMethods();
}

void MotorCtrlFOC::RegisterRpcMethods()
{
    std::string prefix = axis_id_ + ".";
    rpc_server.add(prefix, "modeclp", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeClosedLoopPositionTrajectory, "void MotorCtrlFOC::ModeClosedLoopPositionTrajectory()"));
    rpc_server.add(prefix, "modeclps", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeClosedLoopPositionSimple, "void MotorCtrlFOC::ModeClosedLoopPositionSimple()"));
    rpc_server.add(prefix, "modeclv", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeClosedLoopVelocity, "void MotorCtrlFOC::ModeClosedLoopVelocity()"));
    rpc_server.add(prefix, "modeclt", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeClosedLoopTorque, "void MotorCtrlFOC::ModeClosedLoopTorque()"));
    rpc_server.add(prefix, "modespin", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeSpin, "void MotorCtrlFOC::ModeSpin()"));
    rpc_server.add(prefix, "stop", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Stop, "void MotorCtrlFOC::Stop()"));
    rpc_server.add(prefix, "calibration", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::RunCalibrationSequence, "void MotorCtrlFOC::RunCalibrationSequence(bool reset_rotor)"));
    rpc_server.add(prefix, "velocity_rps", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::VelocityRPS, "void MotorCtrlFOC::VelocityRPS(float revpersec)"));
    rpc_server.add(prefix, "mvp", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::MoveToPosition, "void MotorCtrlFOC::MoveToPosition(uint64_t position)"));
    rpc_server.add(prefix, "mvr", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::MoveRelative, "void MotorCtrlFOC::MoveRelative(int64_t relative)"));

    rpc_server.add(prefix, "push", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::PushStreamPoint, "void PushStreamPoint(uint32_t time, float velocity)"));
    rpc_server.add(prefix, "go", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Go, "void Go()"));

    drive_->RegisterRpcMethods(prefix + "drive.");
}

rexjson::property MotorCtrlFOC::GetPropertyMap()
{
    rexjson::property props = rexjson::property_map({
        {"drive", rexjson::property({drive_->GetPropertyMap()})},
        {"q_current", &q_current_},
        {"velocity", &velocity_},
        {"acceleration", &acceleration_},
        {"deceleration", &deceleration_},
        {"target", &target_},
        {"spin_voltage", &spin_voltage_},
    });
    return props;
}


rexjson::property MotorCtrlFOC::GetConfigPropertyMap()
{
    rexjson::property props = rexjson::property_map({
        {"drive", rexjson::property({drive_->GetConfigPropertyMap()})},
        {"pid_current_kp", rexjson::property(
                &config_.pid_current_kp_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_Iq_.SetGainP(config_.pid_current_kp_);
                    pid_Id_.SetGainP(config_.pid_current_kp_);
                })},
        {"pid_current_ki", rexjson::property(
                &config_.pid_current_ki_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_Iq_.SetGainI(config_.pid_current_ki_);
                    pid_Id_.SetGainI(config_.pid_current_ki_);
                })},
        {"pid_current_maxout", rexjson::property(
                &config_.pid_current_maxout_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_Iq_.SetMaxOutput(config_.pid_current_maxout_);
                    pid_Id_.SetMaxOutput(config_.pid_current_maxout_);
                })},
        {"w_bias", rexjson::property(
                &config_.w_bias_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_W_.SetBias(config_.w_bias_);
                })},
        {"pid_w_kp", rexjson::property(
                &config_.pid_w_kp_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_W_.SetGainP(config_.pid_w_kp_);
                })},
        {"pid_w_ki", rexjson::property(
                &config_.pid_w_ki_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_W_.SetGainI(config_.pid_w_ki_);
                })},
        {"pid_w_maxout", rexjson::property(
                &config_.pid_w_maxout_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_W_.SetMaxOutput(config_.pid_w_maxout_);
                })},

        {"pid_p_kp", rexjson::property(
                &config_.pid_p_kp_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_P_.SetGainP(config_.pid_p_kp_);
                })},
        {"pid_p_maxout", rexjson::property(
                &config_.pid_p_maxout_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_P_.SetMaxOutput(config_.pid_p_maxout_);
                })},
        {"tau_ratio", &config_.tau_ratio_},
        {"vab_advance_factor", &config_.vab_advance_factor_},
        {"display", &config_.display_},
        {"max_poserr_factor", &config_.max_poserr_factor_},
    });
    return props;
}


void MotorCtrlFOC::Stop()
{
    drive_->Abort();
}

void MotorCtrlFOC::RunDebugLoop()
{
    for (;;) {
        uint32_t status = osThreadFlagsWait(SIGNAL_DEBUG_DUMP_POSITION | SIGNAL_DEBUG_DUMP_TRAJECTORY | SIGNAL_DEBUG_DUMP_VELOCITY | SIGNAL_DEBUG_DUMP_TORQUE | SIGNAL_DEBUG_DUMP_SPIN, osFlagsWaitAny, -1);
        if (status & osFlagsError) {
            continue;
        } else if (status & SIGNAL_DEBUG_DUMP_TORQUE) {
            fprintf(stderr,
                    "Sp: %+6.2f (%+9.2f) I_d: %+6.3f I_q: %+6.3f PVd: %+5.2f PVq: %+7.2f PVqP: %+7.2f PVqI: %+7.2f "
                    "Ierr: %+7.3f T: %3lu\r\n",
                    drive_->GetRotorVelocity() / drive_->GetEncoderCPR(),
                    drive_->GetRotorVelocityPTS(),
                    lpf_Id_,
                    lpf_Iq_,
                    pid_Id_.Output(),
                    pid_Iq_.Output(),
                    pid_Iq_.OutputP(),
                    pid_Iq_.OutputI(),
                    Ierr_,
                    foc_time_
            );
        } else if (status & SIGNAL_DEBUG_DUMP_VELOCITY) {
            fprintf(stderr,
                    "PR: %10llu CV: %+6.2f (%+9.2f) I_d: %+6.3f I_q: %+6.3f PVd: %+5.2f PVq: %+7.2f PVqP: %+7.2f PVqI: %+7.2f "
                    "Ierr: %+7.3f Werr: %+7.3f PID_W: %+6.3f PID_WP: %+6.3f PID_WI: %+6.3f T: %3lu\r\n",
                    drive_->GetRotorPosition(),
                    drive_->GetRotorVelocity() / drive_->GetEncoderCPR(),
                    drive_->GetRotorVelocityPTS(),
                    lpf_Id_,
                    lpf_Iq_,
                    pid_Id_.Output(),
                    pid_Iq_.Output(),
                    pid_Iq_.OutputP(),
                    pid_Iq_.OutputI(),
                    Ierr_,
                    Werr_,
                    pid_W_.Output(),
                    pid_W_.OutputP(),
                    pid_W_.OutputI(),
                    foc_time_
            );

        } else if (status & SIGNAL_DEBUG_DUMP_POSITION) {
            fprintf(stderr,
                    "P: %10llu (%10llu) I_q: %+6.3f PVd: %+5.2f PVq: %+7.2f PVqP: %+7.2f PVqI: %+7.2f "
                    "Werr: %+7.3f PID_W: %+6.3f Perr: %+6.1f PID_PP: %+6.1f V_PEP: %+6.1f, T: %3lu\r\n",
                    drive_->GetEncoderPosition(),
                    target_,
                    lpf_Iq_,
                    pid_Id_.Output(),
                    pid_Iq_.Output(),
                    pid_Iq_.OutputP(),
                    pid_Iq_.OutputI(),
                    Werr_,
                    pid_W_.Output(),
                    Perr_,
                    pid_P_.Output(),
                    drive_->GetRotorVelocityPTS(),
                    foc_time_
            );

        } else if (status & SIGNAL_DEBUG_DUMP_TRAJECTORY) {
            fprintf(stderr,
                    "%s: %10llu PR: %10.0f (%10llu) [%10.0f] I_q: %+6.3f PVq: %+7.2f "
                    "Werr: %+7.3f PID_W: %+6.3f Perr: %+6.1f PID_PP: %+6.1f Pd: %+5.0f, V_PTS: %+8.3f, T: %3lu\r\n",
                    axis_id_.c_str(),
                    drive_->GetRotorPosition(),
                    (float)profile_target_.P,
                    target_,
                    (float)profile_target_.P - drive_->GetEncoderPosition(),
                    lpf_Iq_,
                    pid_Iq_.Output(),
                    Werr_,
                    pid_W_.Output(),
                    Perr_,
                    pid_P_.Output(),
                    (float)profile_target_.Pd,
                    drive_->GetRotorVelocityPTS(),
                    foc_time_
            );
        } else if (status & SIGNAL_DEBUG_DUMP_SPIN) {
            fprintf(stderr,
                    "VBus: %5.2f Speed: %9.2f (%9.2f), I_d: %+5.3f, I_q: %+6.3f, T: %4lu, Adv1: %+5.3f\r\n",
                    drive_->GetBusVoltage(),
                    drive_->GetRotorVelocity() / drive_->GetEncoderCPR(),
                    drive_->GetRotorVelocityPTS(),
                    lpf_Id_,
                    lpf_Iq_,
                    foc_time_,
                    config_.vab_advance_factor_ * drive_->GetRotorElecVelocityPTS()/drive_->GetEncoderCPR() * 2.0f * M_PI
            );
        }
    }
}

void MotorCtrlFOC::RunDebugLoopWrapper(void* ctx)
{
    reinterpret_cast<MotorCtrlFOC*>(const_cast<void*>(ctx))->RunDebugLoop();
}

void MotorCtrlFOC::SignalDumpTrajectory()
{
    if (debug_thread_)
        osThreadFlagsSet(debug_thread_, SIGNAL_DEBUG_DUMP_TRAJECTORY);
}

void MotorCtrlFOC::SignalDumpPosition()
{
    if (debug_thread_)
        osThreadFlagsSet(debug_thread_, SIGNAL_DEBUG_DUMP_POSITION);
}

void MotorCtrlFOC::SignalDumpVelocity()
{
    if (debug_thread_)
        osThreadFlagsSet(debug_thread_, SIGNAL_DEBUG_DUMP_VELOCITY);
}

void MotorCtrlFOC::SignalDumpTorque()
{
    if (debug_thread_)
        osThreadFlagsSet(debug_thread_, SIGNAL_DEBUG_DUMP_TORQUE);
}

void MotorCtrlFOC::SignalDumpSpin()
{
    if (debug_thread_)
        osThreadFlagsSet(debug_thread_, SIGNAL_DEBUG_DUMP_SPIN);
}


void MotorCtrlFOC::StartDebugThread()
{
    osThreadAttr_t task_attributes;
    memset(&task_attributes, 0, sizeof(osThreadAttr_t));
    task_attributes.name = "DebugFOC";
    task_attributes.priority = (osPriority_t) osPriorityNormal;
    task_attributes.stack_size = 2048;
    debug_thread_ = osThreadNew(RunDebugLoopWrapper, this, &task_attributes);
}

void MotorCtrlFOC::ModeSpin()
{
    drive_->AddTaskArmMotor();

    drive_->sched_.AddTask([&](){
        uint32_t display_counter = 0;
        drive_->ResetUpdateCounter();
        pid_Id_.Reset();
        pid_Iq_.Reset();
        pid_W_.Reset();
        drive_->sched_.RunUpdateHandler([&]()->bool {

            std::complex<float> Iab = drive_->GetPhaseCurrent();
            std::complex<float> E = drive_->GetRotorElecRotation();

            /*
             *  Park Transform
             *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
             *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
             *
             *  Idq = std::complex<float>(Id, Iq);
             */
            std::complex<float> Idq = Iab * std::conj(E);
            lpf_Id_ = Idq.real();
            lpf_Iq_ = Idq.imag();

            /*
             * Apply advance
             */
            float advance = config_.vab_advance_factor_ * drive_->GetRotorElecVelocityPTS()/drive_->GetEncoderCPR() * 2.0f * M_PI;
            std::complex<float> Eadv = E * std::complex<float>(cosf(advance), sinf(advance));

            /*
             * Inverse Park Transform
             * Va = Vd * cos(R) - Vq * sin(R)
             * Vb = Vd * sin(R) + Vq * cos(R)
             *
             * Vab = std::complex<float>(Va, Vb)
             */
            std::complex<float> V_ab = std::complex<float>(0, spin_voltage_) * Eadv;

            /*
             * Apply the voltage timings
             */
            drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

            if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
                foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t_begin_, hrtimer.GetCounter());
                SignalDumpSpin();
            }

            return true;
        });
    });
    drive_->AddTaskDisarmMotor();
    drive_->Run();
}

void MotorCtrlFOC::ModeClosedLoopTorque()
{
    drive_->AddTaskArmMotor();

    drive_->sched_.AddTask([&](){
        float timeslice = drive_->GetTimeSlice();
        uint32_t display_counter = 0;
        drive_->ResetUpdateCounter();
        pid_Id_.Reset();
        pid_Iq_.Reset();
        pid_W_.Reset();
        drive_->sched_.RunUpdateHandler([&]()->bool {

            std::complex<float> Iab = drive_->GetPhaseCurrent();
            std::complex<float> R = drive_->GetRotorElecRotation();

            /*
             *  Park Transform
             *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
             *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
             *
             *  Idq = std::complex<float>(Id, Iq);
             */
            std::complex<float> Idq = Iab * std::conj(R);
            lpf_Id_ = Idq.real();
            lpf_Iq_ = Idq.imag();

            /*
             * Update D/Q PID Regulators.
             */
            Ierr_ = q_current_ - lpf_Iq_;
            pid_Id_.Input(0.0f - lpf_Id_, timeslice);
            pid_Iq_.Input(Ierr_, timeslice);

            /*
             * Inverse Park Transform
             * Va = Vd * cos(R) - Vq * sin(R)
             * Vb = Vd * sin(R) + Vq * cos(R)
             *
             * Vab = std::complex<float>(Va, Vb)
             */
            std::complex<float> V_ab = std::complex<float>(pid_Id_.Output(), pid_Iq_.Output()) * R;

            /*
             * Apply the voltage timings
             */
            drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

            if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
                foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t_begin_, hrtimer.GetCounter());
                SignalDumpTorque();
            }

            return true;
        });

    });
    drive_->AddTaskDisarmMotor();
    drive_->Run();
}

void MotorCtrlFOC::ModeClosedLoopVelocity()
{
    drive_->AddTaskArmMotor();

    drive_->sched_.AddTask([&](){
        float timeslice = drive_->GetTimeSlice();
        uint32_t display_counter = 0;
        drive_->ResetUpdateCounter();
        pid_Id_.Reset();
        pid_Iq_.Reset();
        pid_W_.Reset();
        drive_->sched_.RunUpdateHandler([&]()->bool {
            std::complex<float> Iab = drive_->GetPhaseCurrent();
            std::complex<float> R = drive_->GetRotorElecRotation();
            float velocity_ecp = velocity_ * timeslice;

            Werr_ = velocity_ecp - drive_->GetRotorVelocityPTS();
            pid_W_.Input(Werr_, timeslice);

            /*
             *  Park Transform
             *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
             *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
             *
             *  Idq = std::complex<float>(Id, Iq);
             */
            std::complex<float> Idq = Iab * std::conj(R);
            lpf_Id_ = Idq.real();
            lpf_Iq_ = Idq.imag();

            /*
             * Update D/Q PID Regulators.
             */
            Ierr_ = pid_W_.Output() - lpf_Iq_;
            pid_Id_.Input(0.0f - lpf_Id_, timeslice);
            pid_Iq_.Input(Ierr_, timeslice);

            /*
             * Inverse Park Transform
             * Va = Vd * cos(R) - Vq * sin(R)
             * Vb = Vd * sin(R) + Vq * cos(R)
             *
             * Vab = std::complex<float>(Va, Vb)
             */
            std::complex<float> V_ab = std::complex<float>(pid_Id_.Output(), pid_Iq_.Output()) * R;

            /*
             * Apply the voltage timings
             */
            drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

            if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
                foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t_begin_, hrtimer.GetCounter());
                SignalDumpVelocity();
            }

            return true;
        });
    });
    drive_->AddTaskDisarmMotor();
    drive_->Run();
}

void MotorCtrlFOC::ModeClosedLoopPositionSimple()
{
    drive_->AddTaskArmMotor();

    drive_->sched_.AddTask([&](){
        float timeslice = drive_->GetTimeSlice();
        uint32_t display_counter = 0;
        drive_->ResetUpdateCounter();
        pid_Id_.Reset();
        pid_Iq_.Reset();
        pid_W_.Reset();
        pid_P_.Reset();
        target_ = drive_->GetEncoderPosition();

        drive_->sched_.RunUpdateHandler([&]()->bool {
            std::complex<float> Iab = drive_->GetPhaseCurrent();
            std::complex<float> R = drive_->GetRotorElecRotation();

            uint64_t enc_position = drive_->GetRotorPosition();
            Perr_ = drive_->GetRotorPositionError(enc_position, target_) * timeslice;
            pid_P_.Input(Perr_, timeslice);
            Werr_ = pid_P_.Output() - drive_->GetRotorVelocityPTS();
            pid_W_.Input(Werr_, timeslice);

            /*
             *  Park Transform
             *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
             *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
             *
             *  Idq = std::complex<float>(Id, Iq);
             */
            std::complex<float> Idq = Iab * std::conj(R);
            lpf_Id_ = Idq.real();
            lpf_Iq_ = Idq.imag();

            /*
             * Update D/Q PID Regulators.
             */
            Ierr_ = pid_W_.Output() - lpf_Iq_;
            pid_Id_.Input(0.0f - lpf_Id_, timeslice);
            pid_Iq_.Input(Ierr_, timeslice);

            /*
             * Inverse Park Transform
             * Va = Vd * cos(R) - Vq * sin(R)
             * Vb = Vd * sin(R) + Vq * cos(R)
             *
             * Vab = std::complex<float>(Va, Vb)
             */
            std::complex<float> V_ab = std::complex<float>(pid_Id_.Output(), pid_Iq_.Output()) * R;

            /*
             * Apply the voltage timings
             */
            drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

            if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
                foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t_begin_, hrtimer.GetCounter());
                SignalDumpPosition();
            }

            return true;
        });
    });
    drive_->AddTaskDisarmMotor();
    drive_->Run();
}


void MotorCtrlFOC::ModeClosedLoopPositionTrajectory()
{
    drive_->AddTaskArmMotor();

    drive_->sched_.AddTask([&](){
        float timeslice = drive_->GetTimeSlice();
        uint32_t display_counter = 0;
        drive_->ResetUpdateCounter();
        pid_Id_.Reset();
        pid_Iq_.Reset();
        pid_W_.Reset();
        pid_P_.Reset();
        TrajectoryPoint* prof_ptr = nullptr;
        float V1 = 0.0f;
        float V2 = 0.0f;
        float V = 0.0f;
        float S2 = target_ = drive_->GetEncoderPosition();
        float S = 0.0f;
        float A = 0.0f;
        uint32_t T1 = 0;
        uint32_t T2 = 0;

        drive_->sched_.RunUpdateHandler([&]()->bool {
            std::complex<float> Iab = drive_->GetPhaseCurrent();
            std::complex<float> R = drive_->GetRotorElecRotation();
            uint64_t enc_position = drive_->GetRotorPosition();

again:
            if (!prof_ptr && !velocity_stream_.empty()) {
                prof_ptr = velocity_stream_.get_read_ptr();
                T1 = drive_->GetUpdateCounter();
                T2 = T1 + prof_ptr->time_;
                V1 = V2;
                V2 = prof_ptr->velocity_;
                S2 = prof_ptr->position_;
                if (T1 == T2) {
                    prof_ptr = nullptr;
                    velocity_stream_.pop();
                    goto again;
                }
                A = (V2 - V1) / (T2 - T1);
            }

            if (prof_ptr) {
                uint32_t T = (drive_->GetUpdateCounter() - T1);
                V = V1  + A * T;
                S = S2 - (V + V2) * (T2 - drive_->GetUpdateCounter()) / 2;
                Perr_ = drive_->GetRotorPositionError(enc_position, S) * timeslice;
                pid_P_.Input(Perr_, timeslice);
                Werr_ = pid_P_.Output() + V - drive_->GetRotorVelocityPTS();
                pid_W_.Input(Werr_, timeslice);
                if (drive_->GetUpdateCounter() == T2) {
                    velocity_stream_.pop();
                    prof_ptr = nullptr;
                }
            } else {
                Perr_ = drive_->GetRotorPositionError(enc_position, S2) * timeslice;
                pid_P_.Input(Perr_, timeslice);
                Werr_ = pid_P_.Output() - drive_->GetRotorVelocityPTS();
                pid_W_.Input(Werr_, timeslice);
            }

            /*
             *  Park Transform
             *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
             *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
             *
             *  Idq = std::complex<float>(Id, Iq);
             */
            std::complex<float> Idq = Iab * std::conj(R);
            lpf_Id_ = Idq.real();
            lpf_Iq_ = Idq.imag();

            /*
             * Update D/Q PID Regulators.
             */
            Ierr_ = pid_W_.Output() - lpf_Iq_;
            pid_Id_.Input(0.0f - lpf_Id_, timeslice);
            pid_Iq_.Input(Ierr_, timeslice);

            /*
             * Inverse Park Transform
             * Va = Vd * cos(R) - Vq * sin(R)
             * Vb = Vd * sin(R) + Vq * cos(R)
             *
             * Vab = std::complex<float>(Va, Vb)
             */
            std::complex<float> V_ab = std::complex<float>(pid_Id_.Output(), pid_Iq_.Output()) * R;

            /*
             * Apply the voltage timings
             */
            drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

            if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
                foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t_begin_, hrtimer.GetCounter());
                SignalDumpTrajectory();
            }

            return true;
        });
    });
    drive_->AddTaskDisarmMotor();
    drive_->Run();
}

void MotorCtrlFOC::PushStreamPoint(uint32_t time, float velocity)
{
    TrajectoryPoint pt(time, velocity);
    velocity_stream_.push(pt);
}

void MotorCtrlFOC::Go()
{
    go_ = true;
}


/** Set movement velocity.
 *
 * @param revpersec movement velocity as revolutions per seconds
 * @return return the internal representation of the velocity as encoder counts per second
 */
float MotorCtrlFOC::VelocityRPS(float revpersec)
{
    velocity_ = revpersec * drive_->GetEncoderCPR();
    return velocity_;
}

/** Set target position
 *
 * @param target new position in encoder counts
 * @return the new target position in encoder counts
 */
uint64_t MotorCtrlFOC::MoveToPosition(uint64_t target)
{
    if (target >= drive_->GetEncoderMaxPosition())
        throw std::range_error("Invalid position");
    int64_t oldpos = target_;
    target_ = target;
    trap_profiler_.Init(target_, drive_->GetEncoderPosition(), drive_->GetRotorVelocity(), velocity_, acceleration_, deceleration_, drive_->GetUpdateFrequency());
    trap_profiler_ptr_ = &trap_profiler_;

    TrajectoryPoint pt0, pt1, pt2, pt3;
    trap_profiler_.CalcTrapezoidPoints(target_, oldpos, 0, velocity_, acceleration_, deceleration_, drive_->GetUpdateFrequency(), pt0, pt1, pt2, pt3);
    __disable_irq();
    velocity_stream_.push(pt0);
    velocity_stream_.push(pt1);
    velocity_stream_.push(pt2);
    velocity_stream_.push(pt3);
    __enable_irq();
    Go();
    return target_;
}

uint64_t MotorCtrlFOC::MoveRelative(int64_t relative)
{
    int64_t oldpos = target_;
    int64_t newpos = relative + target_;
    if (newpos < 0 || newpos >= (int64_t)drive_->GetEncoderMaxPosition())
        throw std::range_error("Invalid position");
    if (velocity_stream_.write_size() < 4) {
        throw std::range_error("Velocity profiler queue is full.");
    }

    target_ = newpos;
    trap_profiler_.Init(target_, drive_->GetEncoderPosition(), drive_->GetRotorVelocity(), velocity_, acceleration_, deceleration_, drive_->GetUpdateFrequency());
    trap_profiler_ptr_ = &trap_profiler_;

    TrajectoryPoint pt0, pt1, pt2, pt3;
    trap_profiler_.CalcTrapezoidPoints(target_, oldpos, 0, velocity_, acceleration_, deceleration_, drive_->GetUpdateFrequency(), pt0, pt1, pt2, pt3);
    __disable_irq();
    velocity_stream_.push(pt0);
    velocity_stream_.push(pt1);
    velocity_stream_.push(pt2);
    velocity_stream_.push(pt3);
    __enable_irq();
    Go();
    return target_;
}

void MotorCtrlFOC::RunCalibrationSequence(bool reset_rotor)
{
    drive_->AddTaskCalibrationSequence(reset_rotor);
    drive_->sched_.AddTask([&](){
#if 1
/*
        R = 4.45
        L = 0.0128
        tau = L/R
        Kp = 1/R
        Tratio = 3 # Tcl/Tp

        # Open loop system
        G = cn.tf(Kp, [tau, 1])

        # Pid controller
        Kc = 1/(Kp*Tratio)
        Ti = L/R
        Td = 0
        Gc = cn.tf([Td*Kc, Kc, Kc/Ti], [1, 0])
        CL = cn.feedback(Gc*G,1)

        cn.pzmap(G)
        cn.pzmap(Gc)
        cn.pzmap(CL)

        plt.figure()
        t = np.linspace(0, 0.025, 100)
        x,y=cn.step_response(G,t)
        plt.plot(x,y)
        x,y=cn.step_response(CL,t)
        plt.plot(x,y)
*/
        float tau = drive_->config_.inductance_ / drive_->config_.resistance_;
        config_.pid_current_kp_ = drive_->config_.resistance_ / config_.tau_ratio_;
        config_.pid_current_ki_ = config_.pid_current_kp_ / tau;
        pid_Id_.SetGainP(config_.pid_current_kp_);
        pid_Id_.SetGainI(config_.pid_current_ki_);
        pid_Iq_.SetGainP(config_.pid_current_kp_);
        pid_Iq_.SetGainI(config_.pid_current_ki_);
#endif
    });
    drive_->RunWaitForCompletion();
}

