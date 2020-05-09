
#include <string.h>

#include "motorctrl_foc.h"
#include "sdmath.h"
#include "uartrpcserver.h"
#include "minasa4encoder.h"

extern MinasA4Encoder ma4_abs_encoder;
extern UartRpcServer rpc_server;

MotorCtrlFOC::MotorCtrlFOC(MotorDrive* drive)
    : drive_(drive)
    , lpf_Id_(config_.idq_alpha_)
    , lpf_Id_disp_(config_.idq_disp_alpha_)
    , lpf_Iq_disp_(config_.idq_disp_alpha_)
    , pid_Vd_(config_.pid_current_kp_, config_.pid_current_ki_, 0, config_.pid_current_decay_, config_.pid_current_maxout_)
    , pid_Vq_(config_.pid_current_kp_, config_.pid_current_ki_, 0, config_.pid_current_decay_, config_.pid_current_maxout_)
    , pid_W_(config_.pid_w_kp_, config_.pid_w_ki_, 0, config_.pid_w_decay_, config_.pid_w_maxout_)
    , pid_P_(config_.pid_p_kp_, config_.pid_p_ki_, 0, config_.pid_p_decay_, config_.pid_p_maxout_)
    , lpf_speed_disp_(config_.speed_disp_alpha_)
{
    StartDebugThread();
    RegisterRpcMethods();
}

void MotorCtrlFOC::RegisterRpcMethods()
{
    rpc_server.add("foc.modeclp", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeClosedLoopPosition, "void MotorCtrlFOC::ModeClosedLoopPosition()"));
    rpc_server.add("foc.modeclv", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeClosedLoopVelocity, "void MotorCtrlFOC::ModeClosedLoopVelocity()"));
    rpc_server.add("foc.modeclt", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeClosedLoopTorque, "void MotorCtrlFOC::ModeClosedLoopTorque()"));
    rpc_server.add("foc.modespin", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::ModeSpin, "void MotorCtrlFOC::ModeSpin()"));
    rpc_server.add("foc.stop", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Stop, "void MotorCtrlFOC::Stop()"));
    rpc_server.add("foc.calibration", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::RunCalibrationSequence, "void MotorCtrlFOC::RunCalibrationSequence(bool reset_rotor)"));
    rpc_server.add("foc.velocity", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::Velocity, "void MotorCtrlFOC::Velocity(float revpersec)"));
    rpc_server.add("foc.mvp", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::MoveToPosition, "void MotorCtrlFOC::MoveToPosition(uint64_t position)"));
    rpc_server.add("foc.mvr", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::MoveRelative, "void MotorCtrlFOC::MoveRelative(int64_t relative)"));

}

rexjson::property MotorCtrlFOC::GetPropertyMap()
{
    rexjson::property props = rexjson::property_map({
        {"idq_alpha", rexjson::property(
                &config_.idq_alpha_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
                [&](void*)->void {
                    lpf_Id_.SetAlpha(config_.idq_alpha_);
                    lpf_Iq_.SetAlpha(config_.idq_alpha_);
                })},

        {"idq_disp_alpha", rexjson::property(
                &config_.idq_disp_alpha_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
                [&](void*)->void {
                    lpf_Id_disp_.SetAlpha(config_.idq_disp_alpha_);
                    lpf_Iq_disp_.SetAlpha(config_.idq_disp_alpha_);
                })},

        {"pid_current_kp", rexjson::property(
                &config_.pid_current_kp_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_Vq_.SetGainP(config_.pid_current_kp_);
                    pid_Vd_.SetGainP(config_.pid_current_kp_);
                })},
        {"pid_current_ki", rexjson::property(
                &config_.pid_current_ki_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_Vq_.SetGainI(config_.pid_current_ki_);
                    pid_Vd_.SetGainI(config_.pid_current_ki_);
                })},
        {"pid_current_decay", rexjson::property(
                &config_.pid_current_decay_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_Vq_.SetDecayRate(config_.pid_current_decay_);
                    pid_Vd_.SetDecayRate(config_.pid_current_decay_);
                })},
        {"pid_current_maxout", rexjson::property(
                &config_.pid_current_maxout_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_Vq_.SetMaxIntegralOutput(config_.pid_current_maxout_);
                    pid_Vd_.SetMaxIntegralOutput(config_.pid_current_maxout_);
                })},
        {"vq_bias", rexjson::property(
                &config_.vq_bias_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_Vq_.SetBias(config_.vq_bias_);
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
        {"pid_w_decay", rexjson::property(
                &config_.pid_w_decay_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_W_.SetDecayRate(config_.pid_w_decay_);
                })},
        {"pid_w_maxout", rexjson::property(
                &config_.pid_w_maxout_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_W_.SetMaxIntegralOutput(config_.pid_w_maxout_);
                })},

        {"pid_p_kp", rexjson::property(
                &config_.pid_p_kp_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_P_.SetGainP(config_.pid_p_kp_);
                })},
        {"pid_p_ki", rexjson::property(
                &config_.pid_p_ki_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_P_.SetGainI(config_.pid_p_ki_);
                })},
        {"pid_p_decay", rexjson::property(
                &config_.pid_p_decay_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_P_.SetDecayRate(config_.pid_p_decay_);
                })},
        {"pid_p_maxout", rexjson::property(
                &config_.pid_p_maxout_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_P_.SetMaxIntegralOutput(config_.pid_p_maxout_);
                })},

        {"speed_disp_alpha", rexjson::property(
                &config_.speed_disp_alpha_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){float t = v.get_real(); if (t < 0 || t > 1.0) throw std::range_error("Invalid value");},
                [&](void*)->void {
                    lpf_speed_disp_.SetAlpha(config_.speed_disp_alpha_);
                })},

        {"control_bandwith", &config_.control_bandwidth_},
        {"iq_setpoint", &iq_setpoint_},
        {"velocity", &velocity_},
        {"position", &position_},
        {"i_trip", &config_.i_trip_},
        {"vab_advance_factor", &config_.vab_advance_factor_},
        {"display", &config_.display_},
        {"spin_voltage", &config_.spin_voltage_},
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
        uint32_t status = osThreadFlagsWait(SIGNAL_DEBUG_DUMP_POSITION | SIGNAL_DEBUG_DUMP_VELOCITY | SIGNAL_DEBUG_DUMP_TORQUE | SIGNAL_DEBUG_DUMP_SPIN, osFlagsWaitAny, -1);
        if (status & osFlagsError) {
            continue;
        } else if (status & SIGNAL_DEBUG_DUMP_TORQUE) {
            fprintf(stderr,
                    "Speed: %+12.9f (%+6.2f), I_d: %+14.9f, I_q: %+14.9f, PVd: %+14.9f, PVq: %+14.9f, PVqP: %+14.9f, PVqI: %+14.9f, "
                    "Ierr: %+14.9f, T: %4lu\n",
                    lpf_speed_disp_.Output(),
                    asinf(lpf_speed_disp_.Output()) / (drive_->GetEncoderTimeSlice() * M_PI * 2 * drive_->GetPolePairs()),
                    lpf_Id_disp_.Output(),
                    lpf_Iq_disp_.Output(),
                    pid_Vd_.Output(),
                    pid_Vq_.Output(),
                    pid_Vq_.OutputP(),
                    pid_Vq_.OutputI(),
                    Ierr_,
                    foc_time_
            );
        } else if (status & SIGNAL_DEBUG_DUMP_VELOCITY) {
            fprintf(stderr,
                    "Speed: %+12.9f (%+6.2f), I_d: %+6.3f, I_q: %+6.3f, PVd: %+7.3f, PVq: %+7.3f, PVqP: %+7.3f, PVqI: %+7.3f, "
                    "Werr: %+12.9f, PID_W: %+12.9f, PID_WP: %+12.9f, PID_WI: %+12.9f, T: %4lu\n",
                    lpf_speed_disp_.Output(),
                    asinf(lpf_speed_disp_.Output()) / (drive_->GetEncoderTimeSlice() * M_PI * 2 * drive_->GetPolePairs()),
                    lpf_Id_disp_.Output(),
                    lpf_Iq_disp_.Output(),
                    pid_Vd_.Output(),
                    pid_Vq_.Output(),
                    pid_Vq_.OutputP(),
                    pid_Vq_.OutputI(),
                    Werr_,
                    pid_W_.Output(),
                    pid_W_.OutputP(),
                    pid_W_.OutputI(),
                    foc_time_
            );
        } else if (status & SIGNAL_DEBUG_DUMP_POSITION) {
            float Rarg = std::arg(R_);
            if (Rarg < 0)
                Rarg += M_PI * 2;
            fprintf(stderr,
                    "Position: %+12.9f ( %13llu ), I_q: %+6.3f, PVq: %+7.3f, PVqP: %+7.3f, PVqI: %+7.3f, "
                    "Rerr: %+12.9f, Werr: %+12.9f, PID_P: %+12.9f, PID_W: %+12.9f, T: %4lu\n",
                    Rarg,
                    enc_position_,
                    lpf_Iq_disp_.Output(),
                    pid_Vq_.Output(),
                    pid_Vq_.OutputP(),
                    pid_Vq_.OutputI(),
                    Rerr_,
                    Werr_,
                    pid_P_.Output(),
                    pid_W_.Output(),
                    foc_time_
            );
        } else if (status & SIGNAL_DEBUG_DUMP_SPIN) {
            fprintf(stderr,
                    "Speed: %13.9f (%5.2f), I_d: %+5.3f, I_q: %+6.3f, t1_span: %4lu, t2_span: %4lu, t2_t2: %4lu, T: %4lu, EncT: %4lu\n",
                    lpf_speed_disp_.Output(),
                    asinf(lpf_speed_disp_.Output()) / (drive_->GetEncoderTimeSlice() * M_PI * 2 * drive_->GetPolePairs()),
                    lpf_Id_disp_.Output(),
                    lpf_Iq_disp_.Output(),
                    drive_->t1_span_,
                    drive_->t2_span_,
                    drive_->t2_to_t2_,
                    foc_time_,
                    ma4_abs_encoder.update_time_ms_
            );
        }
    }
}

void MotorCtrlFOC::RunDebugLoopWrapper(void* ctx)
{
    reinterpret_cast<MotorCtrlFOC*>(const_cast<void*>(ctx))->RunDebugLoop();
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

void MotorCtrlFOC::ModeClosedLoopTorque()
{
    drive_->AddTaskArmMotor();

    drive_->sched_.AddTask([&](){
        float update_period = drive_->GetTimeSlice();
        float enc_update_period = drive_->GetEncoderTimeSlice();
        uint32_t display_counter = 0;
        drive_->data_.update_counter_ = 0;
        pid_Vd_.Reset();
        pid_Vq_.Reset();
        pid_W_.Reset();
        lpf_Id_.Reset();
        lpf_Iq_.Reset();
        lpf_Id_disp_.Reset();
        lpf_Iq_disp_.Reset();
        pid_Vq_.SetMaxIntegralOutput(0.9 * drive_->GetBusVoltage());
        pid_Vd_.SetMaxIntegralOutput(0.9 * drive_->GetBusVoltage());
        drive_->sched_.RunUpdateHandler([&]()->bool {

            std::complex<float> Iab = drive_->GetPhaseCurrent();
            std::complex<float> E = drive_->GetElecRotation();
            float phase_speed = drive_->GetPhaseSpeedVector();

            /*
             *  Park Transform
             *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
             *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
             *
             *  Idq = std::complex<float>(Id, Iq);
             */
            std::complex<float> Idq = Iab * std::conj(E);

            /*
             * Apply filters
             */
            lpf_Id_.DoFilter(Idq.real());
            lpf_Iq_.DoFilter(Idq.imag());
            lpf_Id_disp_.DoFilter(Idq.real());
            lpf_Iq_disp_.DoFilter(Idq.imag());
            lpf_speed_disp_.DoFilter(phase_speed);

            /*
             * Update D/Q PID Regulators.
             */
            Ierr_ = iq_setpoint_ - lpf_Iq_.Output();
            pid_Vd_.Input(0.0f - lpf_Id_.Output(), update_period);
            pid_Vq_.Input(Ierr_, update_period);

            /*
             * Inverse Park Transform
             * Va = Vd * cos(R) - Vq * sin(R)
             * Vb = Vd * sin(R) + Vq * cos(R)
             *
             * Vab = std::complex<float>(Va, Vb)
             */
            std::complex<float> V_ab = std::complex<float>(pid_Vd_.Output(), pid_Vq_.Output()) * E;

            /*
             * Apply advance
             */
            float advance = config_.vab_advance_factor_ * phase_speed * enc_update_period;
            V_ab *= std::complex<float>(cosf(advance), sinf(advance));

            /*
             * Apply the voltage timings
             */
            drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

            foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t2_begin_, hrtimer.GetCounter());
            if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
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
        float update_period = drive_->GetTimeSlice();
        float enc_update_period = drive_->GetEncoderTimeSlice();
        uint32_t display_counter = 0;
        drive_->data_.update_counter_ = 0;
        pid_Vd_.Reset();
        pid_Vq_.Reset();
        pid_W_.Reset();
        lpf_Id_.Reset();
        lpf_Iq_.Reset();
        lpf_speed_disp_.Reset();
        pid_Vq_.SetMaxIntegralOutput(0.9 * drive_->GetBusVoltage());
        pid_Vd_.SetMaxIntegralOutput(0.9 * drive_->GetBusVoltage());
        drive_->sched_.RunUpdateHandler([&]()->bool {
            std::complex<float> Iab = drive_->GetPhaseCurrent();
            std::complex<float> E = drive_->GetElecRotation();
            float phase_speed = drive_->GetPhaseSpeedVector();

            if (drive_->data_.update_counter_ % (drive_->config_.enc_skip_updates_ + 1) == 0) {
                Werr_ = velocity_ - phase_speed;
                pid_W_.Input(Werr_, enc_update_period);
            }

            /*
             *  Park Transform
             *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
             *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
             *
             *  Idq = std::complex<float>(Id, Iq);
             */
            std::complex<float> Idq = Iab * std::conj(E);

            /*
             * Apply filters
             */
            lpf_Id_.DoFilter(Idq.real());
            lpf_Iq_.DoFilter(Idq.imag());
            lpf_Id_disp_.DoFilter(Idq.real());
            lpf_Iq_disp_.DoFilter(Idq.imag());
            lpf_speed_disp_.DoFilter(drive_->GetPhaseSpeedVector());

            /*
             * Update D/Q PID Regulators.
             */
            Ierr_ = pid_W_.Output() - lpf_Iq_.Output();
            pid_Vd_.Input(0.0f - lpf_Id_.Output(), update_period);
            pid_Vq_.Input(Ierr_, update_period);

            /*
             * Inverse Park Transform
             * Va = Vd * cos(R) - Vq * sin(R)
             * Vb = Vd * sin(R) + Vq * cos(R)
             *
             * Vab = std::complex<float>(Va, Vb)
             */
            std::complex<float> V_ab = std::complex<float>(pid_Vd_.Output(), pid_Vq_.Output()) * E;

            /*
             * Apply advance
             */
            float advance = config_.vab_advance_factor_ * phase_speed * enc_update_period;
            V_ab *= std::complex<float>(cosf(advance), sinf(advance));

            /*
             * Apply the voltage timings
             */
            drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

            foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t2_begin_, hrtimer.GetCounter());
            if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
                SignalDumpVelocity();
            }

            return true;
        });
    });
    drive_->AddTaskDisarmMotor();
    drive_->Run();
}

void MotorCtrlFOC::ModeClosedLoopPosition()
{
    drive_->AddTaskArmMotor();

    drive_->sched_.AddTask([&](){
        float update_period = drive_->GetTimeSlice();
        float enc_update_period = drive_->GetEncoderTimeSlice();
        uint32_t display_counter = 0;
        drive_->data_.update_counter_ = 0;
        pid_Vd_.Reset();
        pid_Vq_.Reset();
        pid_W_.Reset();
        lpf_Id_.Reset();
        lpf_Iq_.Reset();
        lpf_speed_disp_.Reset();
        pid_Vq_.SetMaxIntegralOutput(0.9 * drive_->GetBusVoltage());
        pid_Vd_.SetMaxIntegralOutput(0.9 * drive_->GetBusVoltage());
        position_ = drive_->encoder_->GetAbsolutePosition();

        drive_->sched_.RunUpdateHandler([&]()->bool {
            std::complex<float> Iab = drive_->GetPhaseCurrent();
            std::complex<float> E = drive_->GetElecRotation();
            R_ = drive_->GetMechRotation();
            float phase_speed = drive_->GetPhaseSpeedVector();

            if (drive_->data_.update_counter_ % (drive_->config_.enc_skip_updates_ + 1) == 0) {
                enc_position_ = drive_->encoder_->GetAbsolutePosition();
                float target_angle = drive_->encoder_->GetMechanicalPosition(position_);

                int64_t Eerr = enc_position_ - position_;
                float p_velocity = (velocity_ > 0) ? velocity_ : -velocity_;
                if (std::abs(Eerr) < ( 1 << (drive_->encoder_->GetResolutionBits() - 2))) {
                    Rerr_ = sdmath::cross(R_, std::complex<float>(cosf(target_angle), sinf(target_angle)));
                    Werr_ = Rerr_ * p_velocity - phase_speed;
                } else {
                    if (Eerr < 0)
                        Werr_ = p_velocity - phase_speed;
                    else
                        Werr_ = -p_velocity - phase_speed;
                    Rerr_ = 0;
                }


                pid_P_.Input(Rerr_, enc_update_period);
                pid_W_.Input(Werr_, enc_update_period);

            }

            /*
             *  Park Transform
             *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
             *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
             *
             *  Idq = std::complex<float>(Id, Iq);
             */
            std::complex<float> Idq = Iab * std::conj(E);

            /*
             * Apply filters
             */
            lpf_Id_.DoFilter(Idq.real());
            lpf_Iq_.DoFilter(Idq.imag());
            lpf_Id_disp_.DoFilter(Idq.real());
            lpf_Iq_disp_.DoFilter(Idq.imag());
            lpf_speed_disp_.DoFilter(drive_->GetPhaseSpeedVector());

            /*
             * Update D/Q PID Regulators.
             */
            Ierr_ = pid_W_.Output() - lpf_Iq_.Output();
            pid_Vd_.Input(0.0f - lpf_Id_.Output(), update_period);
            pid_Vq_.Input(Ierr_, update_period);

            /*
             * Inverse Park Transform
             * Va = Vd * cos(R) - Vq * sin(R)
             * Vb = Vd * sin(R) + Vq * cos(R)
             *
             * Vab = std::complex<float>(Va, Vb)
             */
            std::complex<float> V_ab = std::complex<float>(pid_Vd_.Output(), pid_Vq_.Output() + pid_P_.Output()) * E;

            /*
             * Apply advance
             */
            float advance = config_.vab_advance_factor_ * phase_speed * enc_update_period;
            V_ab *= std::complex<float>(cosf(advance), sinf(advance));

            /*
             * Apply the voltage timings
             */
            drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

            foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t2_begin_, hrtimer.GetCounter());
            if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
                SignalDumpPosition();
            }

            return true;
        });
    });
    drive_->AddTaskDisarmMotor();
    drive_->Run();
}


void MotorCtrlFOC::ModeSpin()
{
    drive_->AddTaskArmMotor();

    drive_->sched_.AddTask([&](){
        float enc_update_period = drive_->GetEncoderTimeSlice();
        uint32_t display_counter = 0;
        drive_->data_.update_counter_ = 0;
        pid_Vd_.Reset();
        pid_Vq_.Reset();
        pid_W_.Reset();
        lpf_Id_.Reset();
        lpf_Iq_.Reset();
        drive_->sched_.RunUpdateHandler([&]()->bool {

            std::complex<float> Iab = drive_->GetPhaseCurrent();
            std::complex<float> E = drive_->GetElecRotation();
            float phase_speed = drive_->GetPhaseSpeedVector();

            /*
             *  Park Transform
             *  Id = Ialpha * cos(R) + Ibeta  * sin(R)
             *  Iq = Ibeta  * cos(R) - Ialpha * sin(R)
             *
             *  Idq = std::complex<float>(Id, Iq);
             */
            std::complex<float> Idq = Iab * std::conj(E);

            /*
             * Apply filters
             */
            lpf_Id_.DoFilter(Idq.real());
            lpf_Iq_.DoFilter(Idq.imag());
            lpf_speed_disp_.DoFilter(phase_speed);

            /*
             * Inverse Park Transform
             * Va = Vd * cos(R) - Vq * sin(R)
             * Vb = Vd * sin(R) + Vq * cos(R)
             *
             * Vab = std::complex<float>(Va, Vb)
             */
            std::complex<float> V_ab = std::complex<float>(0, config_.spin_voltage_) * E;

            /*
             * Apply advance
             */
            float advance = config_.vab_advance_factor_ * phase_speed * enc_update_period;
            V_ab *= std::complex<float>(cosf(advance), sinf(advance));

            /*
             * Apply the voltage timings
             */
            drive_->ApplyPhaseVoltage(V_ab.real(), V_ab.imag());

            foc_time_ = hrtimer.GetTimeElapsedMicroSec(drive_->t2_begin_, hrtimer.GetCounter());
            if (config_.display_ &&  display_counter++ % drive_->config_.display_div_ == 0) {
                SignalDumpSpin();
            }

            return true;
        });
    });
    drive_->AddTaskDisarmMotor();
    drive_->Run();
}

float MotorCtrlFOC::Velocity(float revpersec)
{
    velocity_ = std::sin(M_PI * 2 * revpersec * drive_->GetPolePairs() * drive_->GetEncoderTimeSlice());
    return velocity_;
}

uint64_t MotorCtrlFOC::MoveToPosition(uint64_t position)
{
    if (position >= drive_->encoder_->GetAbsolutePositionMax())
        throw std::range_error("Invalid position");
    position_ = position;
    return position_;
}

uint64_t MotorCtrlFOC::MoveRelative(int64_t relative)
{
    int64_t newpos = relative + drive_->encoder_->GetAbsolutePosition();
    if (newpos < 0 || newpos >= (int64_t)drive_->encoder_->GetAbsolutePositionMax())
        throw std::range_error("Invalid position");
    position_ = newpos;
    return position_;
}

void MotorCtrlFOC::RunCalibrationSequence(bool reset_rotor)
{
    drive_->AddTaskCalibrationSequence(reset_rotor);
    drive_->sched_.AddTask([&](){

#if 0
        config_.pid_dcurrent_kp_ = config_.control_bandwidth_ * drive_->config_.inductance_;
        config_.pid_dcurrent_ki_ = config_.control_bandwidth_ * drive_->config_.resistance_;
        pid_Vd_.SetGainP(config_.pid_dcurrent_kp_);
        pid_Vq_.SetGainP(config_.pid_dcurrent_kp_);
        pid_Vd_.SetGainI(config_.pid_dcurrent_ki_);
        pid_Vq_.SetGainI(config_.pid_dcurrent_ki_);

        config_.pid_w_kp_ = drive_->config_.inductance_ * config_.control_bandwidth_;
        config_.pid_w_ki_ = drive_->config_.resistance_ * config_.control_bandwidth_;
        pid_W_.SetGainP(config_.pid_w_kp_);
        pid_W_.SetGainI(config_.pid_w_ki_);
#endif
    });
    drive_->RunWaitForCompletion();
}
