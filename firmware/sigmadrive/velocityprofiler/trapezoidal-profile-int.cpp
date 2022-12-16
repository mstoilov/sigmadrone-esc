
#include "trapezoidal-profile-int.h"
#include <cmath>

#ifdef STM32F745xx
#include "stm32f745xx.h"
#endif

#define SQ(x) ((x) * (x))
#define SF (1024 * 1024)

void TrapezoidalProfileInt::Init(int64_t Xf, int64_t Xi, int64_t Vin, int64_t Vmax, int64_t Amax, int64_t Dmax, int64_t Hz)
{
    Xf_ = Xf * SF;                                  // Final position
    Xi_ = Xi * SF;                                  // Initial position
    int64_t s = (Xf >= Xi) ? 1 : -1;
    int64_t Vr = abs(Vmax) * SF / Hz;               // Roaming velocity
    int64_t Ar = abs(Amax) * SF / SQ(Hz);           // Acceleration to roaming velocity
    int64_t Dr = abs(Dmax) * SF / SQ(Hz);           // Deceleration from roaming velocity
    int64_t Vi = s * Vin * SF / Hz;                 // Initial velocity
    int64_t dX = abs(Xf_ - Xi_);                    // Distance delta.

    if (Vi > (int32_t)Vr)
        Ar = -Ar;
    int64_t Da = (Vr + Vi) * (Vr - Vi) / Ar / 2;    // Distance traveled while accelerating
    int64_t Dd = SQ(Vr) / Dr / 2;                   // Distance traveled while  decelerating
    int64_t Dc = dX - (Da + Dd);                    // Distance traveled while the velocity is constant

    if (Dc < 0) {
        // Find Vr by solving:
        // Da + Dd = dX
        // 0.5 * (Vr + Vi) * (Vr - Vi) / Ar + 0.5 *(Vr * Vr) / Dr = dX
        int64_t Vr_sq = Dr * (2 * Ar * dX + SQ(Vi))/(Dr + Ar);
        if (Vr_sq < 0 or Da == 0) {
            // The distence to the requested position is too short
            // for the specified decelaration.
            // Calculate the required decelaration to reach the position
            Vr = Vi;
            Dr = SQ(Vi) / dX / 2;
        } else {
            Vr = std::sqrt(Vr_sq);
        }
        Dc = 0;
        Da = (Vr + Vi) * (Vr - Vi) / Ar / 2;
        Dd = SQ(Vr) / Dr / 2;
    }

    uint32_t Ta = (Vr - Vi) / Ar;                    // Time for acceleration
    uint32_t Td = Vr / Dr;                           // Time for deceleration
    uint32_t Tr = (Vr != 0) ? Dc / Vr : 0;           // Time for roaming (constant velocity)

#ifdef STM32F745xx
    __disable_irq();
#endif
    Vr_ = Vr;
    Ar_ = Ar;
    Dr_ = Dr;
    Vi_ = Vi;
    Ta_ = Ta;
    Td_ = Td;
    Tr_ = Tr;
    T_ = Ta + Tr + Td;
    s_ = s;

    Ar_2_ = Ar_ / 2;
    Dr_2_ = Dr_ / 2;
    Tar_ = Ta_ + Tr_;
    Sa_ = Vi_ * Ta_ + Ar_ * SQ(Ta_) / 2;
    Sar_ = Sa_ + Vr_ * Tr_;

#ifdef STM32F745xx
    __enable_irq();
#endif

}


void TrapezoidalProfileInt::CalcProfileData(uint32_t t, ProfileData<int64_t>& data)
{
    if (t < 0) {
        data.Pd = Vi_ / SF;
        data.P = Xi_ / SF;
    } else if (t < Ta_) {
        data.Pd = (s_ * (Vi_ + Ar_ * t)) / SF;
        data.P = (Xi_ + s_ * (Vi_ * t + Ar_ * SQ(t) / 2)) / SF;
    } else if (t < (Ta_ + Tr_)) {
        data.Pd = (s_ * Vr_) / SF;
        data.P   = (Xi_ + s_ * (Vi_ * Ta_ + Ar_ * SQ(Ta_) / 2 + Vr_ * (t - Ta_))) / SF;
    } else if (t < T_) {
        int32_t tc = t - (Ta_ + Tr_);
        data.Pd = (s_ * (Vr_ - Dr_ * tc)) / SF;
        data.P = (Xi_ + s_ * (Vi_ * Ta_ + Ar_ * SQ(Ta_) / 2 + Vr_ * Tr_ + Vr_ * tc - Dr_ * SQ(tc) / 2)) / SF;
    } else {
        data.Pd = 0;
        data.P = Xf_ / SF;
    }
}

void TrapezoidalProfileInt::CalcProfileData2(uint32_t t, ProfileData<int64_t>& data)
{
    if (t < 0) {
        data.Pd = Vi_ / SF;
        data.P = Xi_ / SF;
    } else if (t < Ta_) {
        data.Pd = (s_ * (Vi_ + Ar_ * t)) / SF;
        data.P = (Xi_ + s_ * (Vi_ * t + Ar_2_ * SQ(t))) / SF;
    } else if (t < (Ta_ + Tr_)) {
        data.Pd = (s_ * Vr_) / SF;
        data.P   = (Xi_ + s_ * (Sa_ + Vr_ * (t - Ta_))) / SF;
    } else if (t < T_) {
        int32_t tc = t - Tar_;
        data.Pd = (s_ * (Vr_ - Dr_ * tc)) / SF;
        data.P = (Xi_ + s_ * (Sar_ + Vr_ * tc - Dr_2_ * SQ(tc))) / SF;
    } else {
        data.Pd = 0;
        data.P = Xf_ / SF;
    }
}

ProfileData<int64_t> TrapezoidalProfileInt::Step(uint32_t t)
{
	ProfileData<int64_t> data(0, 0);
	CalcProfileData(t, data);

	return data;
}


