
#ifndef _TRAPEZOIDPROFILE_H_
#define _TRAPEZOIDPROFILE_H_

#include <vector>
#include <stdint.h>

std::vector<std::vector<int64_t>> 
CalculateTrapezoidPoints(float Xi, float Xf, float Vin, float Vfin, float Vmax, float Amax, float Dmax, float Hz);

std::vector<std::vector<std::vector<int64_t>>> 
CalculateTrapezoidPointsXY(float Xin, float Yin, float Xfin, float Yfin, float Vmax, float Accel, float Decel, float Hz);

#endif
