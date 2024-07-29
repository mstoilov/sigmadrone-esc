
#ifndef _TRAPEZOIDPROFILE_H_
#define _TRAPEZOIDPROFILE_H_

#include <vector>
#include <stdint.h>

std::vector<std::vector<int64_t>> 
CalculateTrapezoidPoints(int64_t Xi, int64_t Xf, int64_t Vin, int64_t Vfin, int64_t Vmax, int64_t Amax, int64_t Dmax, int64_t Hz);

std::vector<std::vector<std::vector<int64_t>>> 
CalculateTrapezoidPointsXY(int64_t Xin, int64_t Yin, int64_t Xfin, int64_t Yfin, int64_t Vmax, int64_t Accel, int64_t Decel, int64_t Hz);

#endif
