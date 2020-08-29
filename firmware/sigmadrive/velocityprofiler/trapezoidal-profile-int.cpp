
#include "trapezoidal-profile-int.h"
#include <math.h>

#ifdef STM32F745xx
#include "stm32f745xx.h"
#endif

#define SQ(x) ((x) * (x))
#define SF 1048576

void TrapezoidalProfileInt::Init(float Xf, float Xi, float Vin, float Vmax, float Amax, float Dmax, float Hz)
{
    Xf_ = Xf * SF;
    Xi_ = Xi * SF;
    int64_t s = (Xf >= Xi) ? 1.0 : -1.0;
    int64_t Vr = abs(Vmax) / Hz * SF;
    int64_t Ar = abs(Amax) / SQ(Hz) * SF;
    int64_t Dr = abs(Dmax) / SQ(Hz) * SF;
    int64_t Vi = s * Vin / Hz * SF;
    int64_t dX = abs(Xf_ - Xi_);

    if (Vi > Vr)
        Ar = -Ar;
    int64_t Da = (Vr + Vi) * (Vr - Vi) / Ar / 2;
    int64_t Dd = SQ(Vr) / Dr / 2;
    int64_t Dc = dX - (Da + Dd);

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
            Vr = sqrtf(Vr_sq);
        }
        Dc = 0;
        Da = (Vr + Vi) * (Vr - Vi) / Ar / 2;
        Dd = SQ(Vr) / Dr / 2;
    }

    int64_t Ta = (Vr - Vi) / Ar;
    int64_t Td = Vr / Dr;
    int64_t Tr = (Vr != 0) ? Dc / Vr : 0;

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

    Ar_2_ = 0.5f * Ar_;
    Dr_2_ = 0.5f * Dr_;
    Tar_ = Ta_ + Tr_;
    Sa_ = Vi_ * Ta_ + 0.5 * Ar_ * SQ(Ta_);
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
        int64_t tc = t - (Ta_ + Tr_) / SF;
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
        float tc = t - Tar_;
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
	CalcProfileData2(t, data);

	return data;
}


