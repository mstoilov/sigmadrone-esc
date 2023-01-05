
#ifndef _TRAPEZOID_PROFILE_H_
#define _TRAPEZOID_PROFILE_H_

#include <vector>
#include <stdint.h>

std::vector<std::vector<int64_t>> CalculateTrapezoidPoints(float Xi, float Xf, float Vin, float Vfin, float Vmax, float Amax, float Dmax, float Hz);

#endif
