
#ifndef _TRAPEZOIDPROFILE_H_
#define _TRAPEZOIDPROFILE_H_

#include <vector>
#include <stdint.h>

std::vector<std::vector<float>> CalculateTrapezoidPoints(float Xi, float Xf, float Vin, float Vfin, float Vmax, float Amax, float Dmax, float Hz);

#endif
