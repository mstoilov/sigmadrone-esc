
#ifndef _TRAPEZOID_PROFILE_H_
#define _TRAPEZOID_PROFILE_H_

class ProfileData
{
public:
    ProfileData(float Pos = 0, float Vel = 0)
        : P(Pos), Pd(Vel)
    {

    }
    float P;	// Position
    float Pd;	// First derivative - velocity
};

class TrapezoidalProfile
{
public:
    TrapezoidalProfile() = default;
    void Init(float Xf, float Xi, float Vin, float Vmax, float Amax, float Dmax);
    void CalcProfileData(float t, ProfileData& data);
    ProfileData Step(float t);


public:
    float Xf_ = 0.0f;
    float Xi_ = 0.0f;
    float Vr_ = 0.0f;
    float Ar_ = 0.0f;
    float Dr_ = 0.0f;
    float Vi_ = 0.0f;
    float dX_ = 0.0f;
    float Ta_ = 0.0f;
    float Td_ = 0.0f;
    float Tc_ = 0.0f;
    float T_ = 0.0f;
    float s_ = 1.0f;
};

#endif
