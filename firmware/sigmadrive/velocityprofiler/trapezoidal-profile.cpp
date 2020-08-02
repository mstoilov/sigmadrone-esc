
#include "trapezoidal-profile.h"
#include "trapTraj.h"
#include <math.h>
#include "stm32f745xx.h"

#define SQ(x) ((x) * (x))

void TrapezoidalProfile::Init(float Xf, float Xi, float Vin, float Vmax, float Amax, float Dmax)
{
    float s = (Xf >= Xi) ? 1.0 : -1.0;
    float Vr = abs(Vmax);
    float Ar = abs(Amax);
    float Dr = abs(Dmax);
    float dX = abs(Xf - Xi);
    float Vi = s * Vin;

    if (Vi > Vr)
        Ar = -Ar;
    float Da = 0.5 * (Vr + Vi) * (Vr - Vi) / Ar;
    float Dd = 0.5 * SQ(Vr) / Dr;
    float Dc = dX - (Da + Dd);

    if (Dc < 0) {
        // Find Vr by solving:
        // Da + Dd = dX
        // 0.5 * (Vr + Vi) * (Vr - Vi) / Ar + 0.5 *(Vr * Vr) / Dr = dX
        float Vr_sq = Dr * (2.0 * Ar * dX + SQ(Vi))/(Dr + Ar);
        if (Vr_sq < 0 or Da == 0) {
            // The distence to the requested position is too short
            // for the specified decelaration.
            // Calculate the required decelaration to reach the position
            Vr = Vi;
            Dr = 0.5 * SQ(Vi) / dX;
        } else {
            Vr = sqrtf(Vr_sq);
        }
        Dc = 0;
        Da = 0.5 * (Vr + Vi) * (Vr - Vi) / Ar;
        Dd = 0.5 * SQ(Vr) / Dr;
    }

    float Ta = (Vr - Vi) / Ar;
    float Td = Vr / Dr;
    float Tc = Dc / Vr;

    __disable_irq();
    Xf_ = Xf;
    Xi_ = Xi;
    Vr_ = Vr;
    Ar_ = Ar;
    Dr_ = Dr;
    Vi_ = Vi;
    Ta_ = Ta;
    Td_ = Td;
    Tc_ = Tc;
    T_ = Ta + Tc + Td;
    s_ = s;
    __enable_irq();

}

ProfileData TrapezoidalProfile::Step(float t)
{
	ProfileData data(0, 0, 0);

	if (t < 0) {
        data.Pdd = 0;
        data.Pd = Vi_;
        data.P = Xi_;
	} else if (t < Ta_) {
        data.Pdd = s_ * Ar_;
        data.Pd = s_ * (Vi_ + Ar_ * t);
        data.P = Xi_ + s_ * (Vi_ * t + 0.5 * Ar_ * SQ(t));
    } else if (t < (Ta_ + Tc_)) {
        data.Pdd = 0;
        data.Pd = s_ * Vr_;
        data.P   = Xi_ + s_ * (Vi_ * Ta_ + 0.5 * Ar_ * SQ(Ta_) + Vr_ * (t - Ta_));
    } else if (t < T_) {
        float tc = t - (Ta_ + Tc_);
        data.Pdd = - s_ * Dr_;
        data.Pd = s_ * (Vr_ - Dr_ * tc);
        data.P = Xi_ + s_ * (Vi_ * Ta_ + 0.5 * Ar_ * SQ(Ta_) + Vr_ * Tc_ + Vr_ * tc - 0.5 * Dr_ * SQ(tc));
    } else {
        data.Pdd = 0;
        data.Pd = 0;
        data.P = Xf_;
    }

	return data;
}


#ifdef _USE_PYBIND_
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(trapezoidprofile, m) {
    py::class_<ProfileData>(m, "ProfileData")
        .def(py::init<float, float, float>())
		.def_readwrite("P", &ProfileData::P)
		.def_readwrite("Pd", &ProfileData::Pd)
		.def_readwrite("Pdd", &ProfileData::Pdd);
    py::class_<TrapezoidalProfile>(m, "TrapezoidProfile")
        .def(py::init<>())
		.def("Init", &TrapezoidalProfile::Init)
		.def("Step", &TrapezoidalProfile::Step);
    py::class_<TrapezoidalTrajectory::Step_t>(m, "Step_t")
        .def(py::init<float, float, float>())
        .def_readwrite("P", &TrapezoidalTrajectory::Step_t::Y)
        .def_readwrite("Pd", &TrapezoidalTrajectory::Step_t::Yd)
        .def_readwrite("Pdd", &TrapezoidalTrajectory::Step_t::Ydd);
    py::class_<TrapezoidalTrajectory>(m, "TrapezoidalTrajectory")
        .def(py::init<>())
        .def("Init", &TrapezoidalTrajectory::Init)
        .def("Step", &TrapezoidalTrajectory::Step);
}

#endif

