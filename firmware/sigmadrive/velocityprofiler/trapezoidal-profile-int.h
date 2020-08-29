
#ifndef _TRAPEZOID_PROFILE_INT_H_
#define _TRAPEZOID_PROFILE_INT_H_

#include <stdint.h>

#include "trapezoidal-profile.h"

class TrapezoidalProfileInt
{
public:
    TrapezoidalProfileInt() = default;
    void Init(float Xf, float Xi, float Vin, float Vmax, float Amax, float Dmax, float Hz);
    void CalcProfileData(uint32_t t, ProfileData<int64_t>& data);
    void CalcProfileData2(uint32_t t, ProfileData<int64_t>& data);
    ProfileData<int64_t> Step(uint32_t t);


public:
    int64_t Xf_ = 0;
    int64_t Xi_ = 0;
    int64_t Vr_ = 0;
    int64_t Ar_ = 0;
    int64_t Dr_ = 0;
    int64_t Vi_ = 0;
    int64_t dX_ = 0;
    int64_t Ta_ = 0;
    int64_t Td_ = 0;
    int64_t Tr_ = 0;
    int64_t T_ = 0;
    int64_t s_ = 1;

    int64_t Ar_2_ = 0.0f;
    int64_t Dr_2_ = 0.0f;
    int64_t Sa_ = 0.0f;
    int64_t Sar_ = 0.0f;
    int64_t Tar_ = 0.0f;
};

#endif
