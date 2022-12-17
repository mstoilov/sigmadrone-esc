
#include "trapezoidal-profile.h"
#include "trapezoidal-profile-int.h"

#include <math.h>

#ifdef STM32F745xx
#include "stm32f745xx.h"
#endif

#define SQ(x) ((x) * (x))


void TrapezoidalProfile::CalcTrapezoidPoints(float Xf, float Xi, float Vin, float Vmax, float Amax, float Dmax, float Hz, TrajectoryPoint& pt0, TrajectoryPoint& pt1, TrajectoryPoint& pt2, TrajectoryPoint& pt3)
{
	float s = (Xf >= Xi) ? 1.0f : -1.0f;
	float Vr = abs(Vmax) / Hz;
	float Ar = abs(Amax) / SQ(Hz);
	float Dr = abs(Dmax) / SQ(Hz);
	float dX = abs(Xf - Xi);
	float Vi = s * Vin  / Hz;


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
			// The distance to the requested position is too short
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
	float Tr = (Vr != 0.0f) ? Dc / Vr : 0.0f;
	float Td = Vr / Dr;

	pt0.time_ = 0;
	pt0.velocity_ = Vi;
	pt0.position_ = Xi;

	pt1.time_ = Ta;
	pt1.velocity_ = s * Vr;
	pt1.position_ = Xi + (Vi + pt1.velocity_) * Ta * 0.5;

	pt2.time_ = Tr;
	pt2.velocity_ = s * Vr;
	pt2.position_ = pt1.position_ + (pt1.velocity_ + pt2.velocity_) * Tr * 0.5;

	pt3.time_ = Td;
	pt3.velocity_ = 0;
	pt3.position_ = pt2.position_ + (pt2.velocity_ + pt3.velocity_) * Td * 0.5;
}

void TrapezoidalProfile::Init(float Xf, float Xi, float Vin, float Vmax, float Amax, float Dmax, float Hz)
{
	float s = (Xf >= Xi) ? 1.0 : -1.0;
	float Vr = abs(Vmax) / Hz;
	float Ar = abs(Amax) / SQ(Hz);
	float Dr = abs(Dmax) / SQ(Hz);
	float dX = abs(Xf - Xi);
	float Vi = s * Vin  / Hz;

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
			// The distance to the requested position is too short
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
	float Tr = (Vr != 0.0f) ? Dc / Vr : 0.0f;

#ifdef STM32F745xx
	__disable_irq();
#endif
	Xf_ = Xf;
	Xi_ = Xi;
	Vr_ = Vr;
	Ar_ = Ar;
	Dr_ = Dr;
	Vi_ = Vi;
	Ta_ = Ta;
	Td_ = Td;
	Tr_ = Tr;
	T_ = Ta + Tr + Td;
	s_ = s;

	Ar_2_ = 0.5f * Ar_;
	Dr_2_ = 0.5f * Dr_;
	Tar_ = Ta_ + Tr_;
	Sa_ = Vi_ * Ta_ + 0.5 * Ar_ * SQ(Ta_);
	Sar_ = Sa_ + Vr_ * Tr_;


#ifdef STM32F745xx
	__enable_irq();
#endif

}


void TrapezoidalProfile::CalcProfileData(float t, ProfileData<float>& data)
{
	if (t < 0) {
		data.Pd = Vi_;
		data.P = Xi_;
	} else if (t < Ta_) {
		data.Pd = s_ * (Vi_ + Ar_ * t);
		data.P = Xi_ + s_ * (Vi_ * t + 0.5 * Ar_ * SQ(t));
	} else if (t < (Ta_ + Tr_)) {
		data.Pd = s_ * Vr_;
		data.P   = Xi_ + s_ * (Vi_ * Ta_ + 0.5 * Ar_ * SQ(Ta_) + Vr_ * (t - Ta_));
	} else if (t < T_) {
		float tc = t - (Ta_ + Tr_);
		data.Pd = s_ * (Vr_ - Dr_ * tc);
		data.P = Xi_ + s_ * (Vi_ * Ta_ + 0.5 * Ar_ * SQ(Ta_) + Vr_ * Tr_ + Vr_ * tc - 0.5 * Dr_ * SQ(tc));
	} else {
		data.Pd = 0;
		data.P = Xf_;
	}
}


void TrapezoidalProfile::CalcProfileData2(float t, ProfileData<float>& data)
{
	if (t < 0) {
		data.Pd = Vi_;
		data.P = Xi_;
	} else if (t < Ta_) {
		data.Pd = s_ * (Vi_ + Ar_ * t);
		data.P = Xi_ + s_ * (Vi_ * t + Ar_2_ * SQ(t));
	} else if (t < (Ta_ + Tr_)) {
		data.Pd = s_ * Vr_;
		data.P   = Xi_ + s_ * (Sa_ + Vr_ * (t - Ta_));
	} else if (t < T_) {
		float tc = t - Tar_;
		data.Pd = s_ * (Vr_ - Dr_ * tc);
		data.P = Xi_ + s_ * (Sar_ + Vr_ * tc - Dr_2_ * SQ(tc));
	} else {
		data.Pd = 0;
		data.P = Xf_;
	}
}


ProfileData<float> TrapezoidalProfile::Step(float t)
{
	ProfileData<float> data(0, 0);
	CalcProfileData2(t, data);

	return data;
}


std::vector<TrajectoryPoint> TrapezoidalProfile::CalcTrapPoints(float Xf, float Xi, float Vin, float Vmax, float Amax, float Dmax, float Hz)
{
	std::vector<TrajectoryPoint> ret;
	TrajectoryPoint pt0;
	TrajectoryPoint pt1;
	TrajectoryPoint pt2;
	TrajectoryPoint pt3;

	CalcTrapezoidPoints(Xf, Xi, Vin, Vmax, Amax, Dmax, Hz, pt0, pt1, pt2, pt3);
	ret.push_back(pt0);
	ret.push_back(pt1);
	ret.push_back(pt2);
	ret.push_back(pt3);
	return ret;
}

float TrapezoidalProfile::CalcVelocity(float t)
{
	if (t < 0) {
		return Vi_;
	} else if (t < Ta_) {
		return s_ * (Vi_ + Ar_ * t);
	} else if (t < (Ta_ + Tr_)) {
		return s_ * Vr_;
	} else if (t < T_) {
		float tc = t - (Ta_ + Tr_);
		return s_ * (Vr_ - Dr_ * tc);
	}
	return 0.0f;
}


#ifdef _USE_PYBIND_
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(trapezoidprofile, m) {
	py::class_<ProfileData<float>>(m, "ProfileData")
		.def(py::init<float, float>())
		.def_readwrite("P", &ProfileData<float>::P)
		.def_readwrite("Pd", &ProfileData<float>::Pd);

	py::class_<TrajectoryPoint>(m, "TrajectoryPoint")
		.def(py::init<uint32_t, float>())
		.def_readwrite("time", &TrajectoryPoint::time_)
		.def_readwrite("velocity", &TrajectoryPoint::velocity_)
		.def_readwrite("position", &TrajectoryPoint::position_);

	py::class_<ProfileData<int64_t>>(m, "ProfileDataInt")
		.def(py::init<int64_t, int64_t>())
		.def_readwrite("P", &ProfileData<int64_t>::P)
		.def_readwrite("Pd", &ProfileData<int64_t>::Pd);
	py::class_<TrapezoidalProfile>(m, "TrapezoidProfile")
		.def(py::init<>())
		.def("Init", &TrapezoidalProfile::Init)
		.def("Step", &TrapezoidalProfile::Step)
		.def("CalcTrapPoints", &TrapezoidalProfile::CalcTrapPoints)
		.def("CalcVelocity", &TrapezoidalProfile::CalcVelocity)
		.def_readwrite("T", &TrapezoidalProfile::T_)
		.def_readwrite("Ta", &TrapezoidalProfile::Ta_)
		.def_readwrite("Tr", &TrapezoidalProfile::Tr_)
		.def_readwrite("Td", &TrapezoidalProfile::Td_);
	py::class_<TrapezoidalProfileInt>(m, "TrapezoidProfileInt")
		.def(py::init<>())
		.def("Init", &TrapezoidalProfileInt::Init)
		.def("Step", &TrapezoidalProfileInt::Step)
		.def_readwrite("T", &TrapezoidalProfileInt::T_)
		.def_readwrite("Ta", &TrapezoidalProfileInt::Ta_)
		.def_readwrite("Tr", &TrapezoidalProfileInt::Tr_)
		.def_readwrite("Td", &TrapezoidalProfileInt::Td_);
}

#endif

