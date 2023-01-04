
#include "trapezoidal-profile.h"

#include <math.h>

#ifdef STM32F745xx
#include "stm32f745xx.h"
#endif

#define SQ(x) ((x) * (x))


std::vector<std::vector<int64_t>> CalculateTrapezoidPoints(float Xf, float Xi, float Vin, float Vmax, float Amax, float Dmax, float Hz)
{
	float s = (Xf >= Xi) ? 1.0f : -1.0f;
	float Vr = abs(Vmax) / Hz;				// Requested Velocity
	float Ar = abs(Amax) / SQ(Hz);			// Requested Acceleration
	float Dr = abs(Dmax) / SQ(Hz);			// Requested Deceleration
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

	float pt0time = 0;
	float pt0velocity = Vi;
	float pt0position = Xi;

	float pt1time = Ta;
	float pt1velocity = s * Vr;
	float pt1position = Xi + (Vi + pt1velocity) * Ta * 0.5;

	float pt2time = Tr;
	float pt2velocity = s * Vr;
	float pt2position = pt1position + (pt1velocity + pt2velocity) * Tr * 0.5;

	float pt3time = Td;
	float pt3velocity = 0;
	float pt3position = pt2position + (pt2velocity + pt3velocity) * Td * 0.5;

	std::vector<std::vector<int64_t>> ret = {
		{(int64_t)pt0time, (int64_t)pt0velocity, (int64_t)pt0position},
		{(int64_t)pt1time, (int64_t)pt1velocity, (int64_t)pt1position},
		{(int64_t)pt2time, (int64_t)pt2velocity, (int64_t)pt2position},
		{(int64_t)pt3time, (int64_t)pt3velocity, (int64_t)pt3position}};
	return ret;
}


#ifdef _USE_PYBIND_
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(trapezoidprofile, m) {
	m.def("CalculateTrapezoidPoints", &CalculateTrapezoidPoints, "Function calculating trapezoid profile points");
}

#endif

