#include <math.h>
#include "trapezoidprofile.h"


#define SQ(x) ((x) * (x))

// Pin   - initial position              [ec]
// Pfin  - final position                [ec]
// Vin   - initial velocity              [ec/sec]
// Vfin  - final velocity                [ec/sec]
// Vmax  - max velocity allowed          [ec/sec]
// Accel - acceleration                  [ec/(sec*sec)]
// Decel - deceleration                  [ec/(sec*sec)]
// Hz    - Closed loop update frequency  [Hz]
std::vector<std::vector<int64_t>> CalculateTrapezoidPoints(float Pin, float Pfin, float Vin, float Vfin, float Vmax, float Accel, float Decel, float Hz)
{
	float s = (Pfin >= Pin) ? 1.0f : -1.0f;
	float Vr = abs(Vmax);								// Requested Velocity
	float Ar = abs(Accel);								// Requested Acceleration
	float Dr = abs(Decel);								// Requested Deceleration
	float dP = abs(Pfin - Pin);							// Total displacement
	float Vi = s * Vin;									// Initial speed in encoder
	float Vf = s * Vfin;								// Final speed in encoder


	if (Vi > Vr)
		Ar = -Ar;
	float Da = 0.5 * (Vr + Vi) * (Vr - Vi) / Ar;		// Displacement during acceleration
	float Dd = 0.5 * (Vr + Vf) * (Vr - Vf) / Dr;		// Displacement during deceleration
	float Dc = dP - (Da + Dd);							// Displacement during const velocity

	if (Dc < 0) {
		// Find Vr by solving:
		// Da + Dd = dP
		// 0.5 * (Vr + Vi) * (Vr - Vi) / Ar + 0.5 *(Vr * Vr) / Dr = dP
		float Vr_sq = Dr * (2.0 * Ar * dP + SQ(Vi))/(Dr + Ar);
		if (Vr_sq < 0 or Da == 0) {
			// The distance to the requested position is too short
			// for the specified decelaration.
			// Calculate the required decelaration to reach the position
			Vr = Vi;
			Dr = 0.5 * SQ(Vi) / dP;
		} else {
			Vr = sqrtf(Vr_sq);
		}
		Dc = 0;
		Da = 0.5 * (Vr + Vi) * (Vr - Vi) / Ar;
		Dd = 0.5 * (Vr + Vf) * (Vr - Vf) / Dr;
	}

	float Ta = (Vr - Vi) / Ar;
	float Tr = (Vr != 0.0f) ? Dc / Vr : 0.0f;
	float Td = (Vr - Vf) / Dr;

	float pt0t = 0;
	float pt0v = Vi;
	float pt0p = Pin;

	float pt1t = Ta;
	float pt1v = s * Vr;
	float pt1p = Pin + (Vi + pt1v) * Ta * 0.5;

	float pt2t = Tr;
	float pt2v = s * Vr;
	float pt2p = pt1p + (pt1v + pt2v) * Tr * 0.5;

	float pt3t = Td;
	float pt3v = Vf;
	float pt3p = Pfin;

	std::vector<std::vector<int64_t>> ret = {
		{(int64_t)(pt0t * Hz), (int64_t)pt0v, (int64_t)pt0p},
		{(int64_t)(pt1t * Hz), (int64_t)pt1v, (int64_t)pt1p},
		{(int64_t)(pt2t * Hz), (int64_t)pt2v, (int64_t)pt2p},
		{(int64_t)(pt3t * Hz), (int64_t)pt3v, (int64_t)pt3p}};
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
