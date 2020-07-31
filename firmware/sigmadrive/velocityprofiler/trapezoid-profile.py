import trapezoidprofile as tp
import matplotlib.pyplot as pp
import numpy as np

def sign_hard(x):
    return 1.0 if (x >= 0) else -1.0

def SQ(x):
    return x*x


def trapezoidalProfile(Xf, Xi, Vin, Vmax, Amax, Dmax, t):
    Yd = 0
    Y = 0
    s = 1.0 if (Xf > Xi) else -1.0
    
    Vr = Vmax
    Ar = Amax
    Dr = Dmax
    Vi = s * Vin
    dX = np.abs(Xf - Xi)
    
    if (Vi > Vr):
        Ar = -Ar
    Da = 0.5 * (Vr + Vi) * (Vr - Vi) / Ar
    Dd = 0.5 * SQ(Vr) / Dr
    Dc = dX - (Da + Dd)
    if (Dc < 0):
        # Find Vr by solving:
        # Da + Dd = dX
        # 0.5 * (Vr + Vi) * (Vr - Vi) / Ar + 0.5 *(Vr * Vr) / Dr = dX
        Vr_sq = Dr*(2.0 * Ar * dX + SQ(Vi))/(Dr + Ar)
        #Vr_sq = (((Vi**2 + 2*Ar*dX)*(Ar + Dr))/Dr)
        if (Vr_sq < 0 or Da == 0):
            # The distence to the requested position is too short 
            # for the specified decelaration.
            # Calculate the required decelaration to reach the position
            Vr = Vi
            Dr = 0.5 * SQ(Vi) / dX
        else:
            Vr = np.sqrt(Vr_sq)
        Dc = 0
        Da = 0.5 * (Vr + Vi) * (Vr - Vi) / Ar
        Dd = 0.5 * SQ(Vr) / Dr

    
    Ta = (Vr - Vi) / Ar
    Td = Vr / Dr
    Tc = Dc / Vr
    T = Ta + Tc + Td

    if (t < Ta):
        Ydd = s * Ar
        Yd = s * (Vi + Ar * t)
        Y   = Xi + s * (Vi * t + 0.5 * Ar * SQ(t))
    elif t < (Ta + Tc):
        Ydd = 0
        Yd = s * Vr
        Y   = Xi + s * (Vi * Ta + 0.5 * Ar * SQ(Ta) + Vr * (t - Ta))
    elif (t < T):
        tc = t - (Ta + Tc)
        Ydd = - s * Dr
        Yd = s * (Vr - Dr * tc)
        Y = Xi + s * (Vi * Ta + 0.5 * Ar * SQ(Ta) + Vr * Tc + Vr * tc - 0.5 * Dr * SQ(tc))
    else:
        Ydd = 0
        Yd = 0
        Y = Xf
        
    return Y, Yd, Ydd


time = np.linspace(0, 70, 200)
Vc = np.zeros_like(time)
Sc = np.zeros_like(time)
Ac = np.zeros_like(time)

trap = tp.TrapezoidProfile();
trap.Init(160, 40, 0, 4, 0.5, 0.15)
for i, t in enumerate(time):
    data = trap.Step(time[1])
    Sc[i], Vc[i], Ac[i] = data.P, data.Pd, data.Pdd
    trap.Init(160, data.P, data.Pd, 4, 0.5, 0.15)

# for i, t in enumerate(time):
#     Sc[i], Vc[i], Ac[i] = trapezoidalProfile(120, 30, 5, 4, 0.5, 0.15, t)

pp.plot(time, Ac * 40)
pp.plot(time, Vc * 20)
pp.plot(time, Sc)
pp.xlabel('Time')
pp.ylabel('P,Pd,Pdd')
pp.show()
