
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
    , lpf_Iq_(config_.idq_alpha_)
    , pid_Id_(config_.pid_current_kp_, config_.pid_current_ki_, 0, config_.pid_current_decay_, config_.pid_current_maxout_)
    , pid_Iq_(config_.pid_current_kp_, config_.pid_current_ki_, 0, config_.pid_current_decay_, config_.pid_current_maxout_)
    , pid_W_(config_.pid_w_kp_, config_.pid_w_ki_, 0, config_.pid_w_decay_, config_.pid_w_maxout_)
    , pid_P_(config_.pid_p_kp_, config_.pid_p_ki_, 0, config_.pid_p_decay_, config_.pid_p_maxout_)
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
    rpc_server.add("foc.velocity_rps", rexjson::make_rpc_wrapper(this, &MotorCtrlFOC::VelocityRPS, "void MotorCtrlFOC::VelocityRPS(float revpersec)"));
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
        {"pid_current_decay", rexjson::property(
                &config_.pid_current_decay_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_Iq_.SetDecayRate(config_.pid_current_decay_);
                    pid_Id_.SetDecayRate(config_.pid_current_decay_);
                })},
        {"pid_current_maxout", rexjson::property(
                &config_.pid_current_maxout_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_Iq_.SetMaxIntegralOutput(config_.pid_current_maxout_);
                    pid_Id_.SetMaxIntegralOutput(config_.pid_current_maxout_);
                })},
        {"vq_bias", rexjson::property(
                &config_.vq_bias_,
                rexjson::property_access::readwrite,
                [](const rexjson::value& v){},
                [&](void*)->void {
                    pid_Iq_.SetBias(config_.vq_bias_);
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
        {"control_bandwith", &config_.control_bandwidth_},
        {"q_current", &q_current_},
        {"velocity", &velocity_},
        {"target", &target_},
        {"i_trip", &config_.i_trip_},
        {"vab_advance_factor", &config_.vab_advance_factor_},
        {"display", &config_.display_},
        {"spin_voltage", &spin_voltage_},
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
                    "Sp: %+6.2f (%+9.2f) I_d: %+6.3f I_q: %+6.3f PVd: %+5.2f PVq: %+7.2f PVqP: %+7.2f PVqI: %+7.2f "
                    "Ierr: %+7.3f T: %3lu\n",
                    drive_->GetRotorVelocity() * drive_->enc_update_hz_ / drive_->enc_cpr_,
                    drive_->GetRotorVelocity(),
                    lpf_Id_.Output(),
                    lpf_Iq_.Output(),
                    pid_Id_.Output(),
                    pid_Iq_.Output(),
                    pid_Iq_.OutputP(),
                    pid_Iq_.OutputI(),
                    Ierr_,
                    foc_time_
            );
        } else if (status & SIGNAL_DEBUG_DUMP_VELOCITY) {
            fprintf(stderr,
                    "Sp: %+6.2f (%+9.2f) I_d: %+6.3f I_q: %+6.3f PVd: %+5.2f PVq: %+7.2f PVqP: %+7.2f PVqI: %+7.2f "
                    "Ierr: %+7.3f Werr: %+7.3f PID_W: %+6.3f PID_WP: %+6.3f PID_WI: %+6.3f T: %3lu\n",
                    drive_->GetRotorVelocity() * drive_->enc_update_hz_ / drive_->enc_cpr_,
                    drive_->GetRotorVelocity(),
                    lpf_Id_.Output(),
                    lpf_Iq_.Output(),
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
                    "Tg: %12llu (%12llu) I_d: %+6.3f I_q: %+6.3f PVd: %+5.2f PVq: %+7.2f PVqP: %+7.2f PVqI: %+7.2f "
                    "Werr: %+7.3f PID_W: %+6.3f PID_WP: %+6.3f PID_WI: %+6.3f Eerr: %+12lld PID_P: %+12lld T: %3lu\n",
                    drive_->GetEncoderPosition(),
                    target_,
                    lpf_Id_.Output(),
                    lpf_Iq_.Output(),
                    pid_Id_.Output(),
                    pid_Iq_.Output(),
                    pid_Iq_.OutputP(),
                    pid_Iq_.OutputI(),
                    Werr_,
                    pid_W_.Output(),
                    pid_W_.OutputP(),
                    pid_W_.OutputI(),
                    Eerr_,
                    pid_P_.Output(),
                    foc_time_
            );

        } else if (status & SIGNAL_DEBUG_DUMP_SPIN) {
            fprintf(stderr,
                    "Speed: %9.2f (%9.2f), I_d: %+5.3f, I_q: %+6.3f, t1_span: %4lu, t2_span: %4lu, t2_t2: %4lu, T: %4lu, Adv1: %+5.3f, EncT: %4lu\n",
                    drive_->GetRotorVelocity() * drive_->enc_update_hz_ / drive_->enc_cpr_,
                    drive_->GetRotorVelocity(),
                    lpf_Id_.Output(),
                    lpf_Iq_.Output(),
                    drive_->t1_span_,
                    drive_->t2_span_,
                    drive_->t2_to_t2_,
                    foc_time_,
                    config_.vab_advance_factor_ * drive_->GetRotorElecVelocity()/drive_->enc_cpr_ * 2.0f * M_PI,
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

void MotorCtrlFOC::ModeSpin()
{
    drive_->AddTaskArmMotor();

    drive_->sched_.AddTask([&](){
        uint32_t display_counter = 0;
        drive_->data_.update_counter_ = 0;
        pid_Id_.Reset();
        pid_Iq_.Reset();
        pid_W_.Reset();
        lpf_Id_.Reset();
        lpf_Iq_.Reset();
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

            /*
             * Apply filters
             */
            lpf_Id_.DoFilter(Idq.real());
            lpf_Iq_.DoFilter(Idq.imag());

            /*
             * Apply advance
             */
            float advance = config_.vab_advance_factor_ * drive_->GetRotorElecVelocity()/drive_->enc_cpr_ * 2.0f * M_PI;
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

void MotorCtrlFOC::ModeClosedLoopTorque()
{
    drive_->AddTaskArmMotor();

    drive_->sched_.AddTask([&](){
        float update_period = drive_->GetTimeSlice();
        uint32_t display_counter = 0;
        drive_->data_.update_counter_ = 0;
        pid_Id_.Reset();
        pid_Iq_.Reset();
        pid_W_.Reset();
        lpf_Id_.Reset();
        lpf_Iq_.Reset();
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

            /*
             * Apply filters
             */
            lpf_Id_.DoFilter(Idq.real());
            lpf_Iq_.DoFilter(Idq.imag());

            /*
             * Update D/Q PID Regulators.
             */
            Ierr_ = q_current_ - lpf_Iq_.Output();
            pid_Id_.Input(0.0f - lpf_Id_.Output(), update_period);
            pid_Iq_.Input(Ierr_, update_period);

            /*
             * Inverse Park Transform
             * Va = Vd * cos(R) - Vq * sin(R)
             * Vb = Vd * sin(R) + Vq * cos(R)
             *
             * Vab = std::complex<float>(Va, Vb)
             */
            std::complex<float> V_ab = std::complex<float>(pid_Id_.Output(), pid_Iq_.Output()) * E;

            /*
             * Apply advance
             */
            float advance = config_.vab_advance_factor_ * drive_->GetRotorElecVelocity()/drive_->enc_cpr_ * 2.0f * M_PI;
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
        pid_Id_.Reset();
        pid_Iq_.Reset();
        pid_W_.Reset();
        lpf_Id_.Reset();
        lpf_Iq_.Reset();
        drive_->sched_.RunUpdateHandler([&]()->bool {
            std::complex<float> Iab = drive_->GetPhaseCurrent();
            std::complex<float> E = drive_->GetRotorElecRotation();
            float velocity_ecp = velocity_ * enc_update_period;

            if (drive_->data_.update_counter_ % (drive_->config_.enc_skip_updates_ + 1) == 0) {
                Werr_ = velocity_ecp - drive_->GetRotorVelocity();
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

            /*
             * Update D/Q PID Regulators.
             */
            Ierr_ = pid_W_.Output() - lpf_Iq_.Output();
            pid_Id_.Input(0.0f - lpf_Id_.Output(), update_period);
            pid_Iq_.Input(Ierr_, update_period);

            /*
             * Inverse Park Transform
             * Va = Vd * cos(R) - Vq * sin(R)
             * Vb = Vd * sin(R) + Vq * cos(R)
             *
             * Vab = std::complex<float>(Va, Vb)
             */
            std::complex<float> V_ab = std::complex<float>(pid_Id_.Output(), pid_Iq_.Output()) * E;

            /*
             * Apply advance
             */
            float advance = config_.vab_advance_factor_ * drive_->GetRotorElecVelocity()/drive_->enc_cpr_ * 2.0f * M_PI;
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
        pid_Id_.Reset();
        pid_Iq_.Reset();
        pid_W_.Reset();
        lpf_Id_.Reset();
        lpf_Iq_.Reset();
        target_ = drive_->GetEncoderPosition();

        drive_->sched_.RunUpdateHandler([&]()->bool {
            std::complex<float> Iab = drive_->GetPhaseCurrent();
            std::complex<float> E = drive_->GetRotorElecRotation();

            if (drive_->data_.update_counter_ % (drive_->config_.enc_skip_updates_ + 1) == 0) {
                float velocity_ecp = std::abs(velocity_) * enc_update_period;
                Eerr_ = (int64_t)(target_ - drive_->GetEncoderPosition());
                float output_ecp = (float) pid_P_.Input(Eerr_, enc_update_period);
                output_ecp = std::min(output_ecp, velocity_ecp);
                output_ecp = std::max(output_ecp, -velocity_ecp);
                Werr_ = output_ecp - drive_->GetRotorVelocity();
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

            /*
             * Update D/Q PID Regulators.
             */
            Ierr_ = pid_W_.Output() - lpf_Iq_.Output();
            pid_Id_.Input(0.0f - lpf_Id_.Output(), update_period);
            pid_Iq_.Input(Ierr_, update_period);

            /*
             * Inverse Park Transform
             * Va = Vd * cos(R) - Vq * sin(R)
             * Vb = Vd * sin(R) + Vq * cos(R)
             *
             * Vab = std::complex<float>(Va, Vb)
             */
            std::complex<float> V_ab = std::complex<float>(pid_Id_.Output(), pid_Iq_.Output()) * E;

            /*
             * Apply advance
             */
            float advance = config_.vab_advance_factor_ * drive_->GetRotorElecVelocity()/drive_->enc_cpr_ * 2.0f * M_PI;
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

/** Set movement velocity.
 *
 * @param revpersec movement velocity as revolutions per seconds
 * @return return the internal representation of the velocity as encoder counts per second
 */
float MotorCtrlFOC::VelocityRPS(float revpersec)
{
    velocity_ = revpersec * drive_->enc_cpr_;
    return velocity_;
}

/** Set target position
 *
 * @param target new position in encoder counts
 * @return the new target position in encoder counts
 */
uint64_t MotorCtrlFOC::MoveToPosition(uint64_t target)
{
    if (target >= (1ULL << (drive_->enc_resolution_bits_ + drive_->enc_revolution_bits_)))
        throw std::range_error("Invalid position");
    target_ = target;
    return target_;
}

uint64_t MotorCtrlFOC::MoveRelative(int64_t relative)
{
    int64_t newpos = relative + drive_->GetEncoderPosition();
    if (newpos < 0 || newpos >= (int64_t)(1ULL << (drive_->enc_resolution_bits_ + drive_->enc_resolution_bits_)))
        throw std::range_error("Invalid position");
    target_ = newpos;
    return target_;
}

void MotorCtrlFOC::RunCalibrationSequence(bool reset_rotor)
{
    drive_->AddTaskCalibrationSequence(reset_rotor);
    drive_->sched_.AddTask([&](){

#if 0
        config_.pid_dcurrent_kp_ = config_.control_bandwidth_ * drive_->config_.inductance_;
        config_.pid_dcurrent_ki_ = config_.control_bandwidth_ * drive_->config_.resistance_;
        pid_Id_.SetGainP(config_.pid_dcurrent_kp_);
        pid_Iq_.SetGainP(config_.pid_dcurrent_kp_);
        pid_Id_.SetGainI(config_.pid_dcurrent_ki_);
        pid_Iq_.SetGainI(config_.pid_dcurrent_ki_);

        config_.pid_w_kp_ = drive_->config_.inductance_ * config_.control_bandwidth_;
        config_.pid_w_ki_ = drive_->config_.resistance_ * config_.control_bandwidth_;
        pid_W_.SetGainP(config_.pid_w_kp_);
        pid_W_.SetGainI(config_.pid_w_ki_);
#endif
    });
    drive_->RunWaitForCompletion();
}
