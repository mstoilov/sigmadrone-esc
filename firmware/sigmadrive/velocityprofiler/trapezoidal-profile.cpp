
#include "trapezoidal-profile.h"

#include <math.h>

#define SQ(x) ((x) * (x))

void TrapezoidalProfile::Init(float Xf, float Xi, float Vin, float Vmax, float Amax, float Dmax)
{
    s_ = (Xf >= Xi) ? 1.0 : -1.0;
    Xf_ = Xf;
    Xi_ = Xi;
    Vr_ = abs(Vmax);
    Ar_ = abs(Amax);
    Dr_ = abs(Dmax);
    dX_ = abs(Xf - Xi);
    Vi_ = s_ * Vin;

    if (Vi_ > Vr_)
        Ar_ = -Ar_;
    Da_ = 0.5 * (Vr_ + Vi_) * (Vr_ - Vi_) / Ar_;
    Dd_ = 0.5 * SQ(Vr_) / Dr_;
    Dc_ = dX_ - (Da_ + Dd_);

    if (Dc_ < 0) {
        // Find Vr by solving:
        // Da + Dd = dX
        // 0.5 * (Vr + Vi) * (Vr - Vi) / Ar + 0.5 *(Vr * Vr) / Dr = dX
        float Vr_sq = Dr_ * (2.0 * Ar_ * dX_ + SQ(Vi_))/(Dr_ + Ar_);
        if (Vr_sq < 0 or Da_ == 0) {
            // The distence to the requested position is too short
            // for the specified decelaration.
            // Calculate the required decelaration to reach the position
            Vr_ = Vi_;
            Dr_ = 0.5 * SQ(Vi_) / dX_;
        } else {
            Vr_ = sqrtf(Vr_sq);
        }
        Dc_ = 0;
        Da_ = 0.5 * (Vr_ + Vi_) * (Vr_ - Vi_) / Ar_;
        Dd_ = 0.5 * SQ(Vr_) / Dr_;
    }

    Ta_ = (Vr_ - Vi_) / Ar_;
    Td_ = Vr_ / Dr_;
    Tc_ = Dc_ / Vr_;
    T_ = Ta_ + Tc_ + Td_;
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
}

#endif

