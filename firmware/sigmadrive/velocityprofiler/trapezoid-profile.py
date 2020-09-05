import trapezoidprofile as tp
import matplotlib.pyplot as pp
import numpy as np
import sys

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


Pf = 1000000
Pi = 0
Vi = 0
Vmax = 65535 * 30
Acc = 40000000
Dec = 5000000
HZ = 18000

nargs = len(sys.argv)

Pf = int(sys.argv[1]) if nargs > 1 else Pf
Pi = int(sys.argv[2]) if nargs > 2 else Pi
Vi = int(sys.argv[3]) if nargs > 3 else Vi
Vmax = int(sys.argv[4]) if nargs > 4 else Vmax
Acc = int(sys.argv[5]) if nargs > 5 else Acc
Dec = int(sys.argv[6]) if nargs > 6 else Dec
HZ = int(sys.argv[7]) if nargs > 7 else HZ

prof = tp.TrapezoidProfile();
prof.Init(Pf, Pi, Vi, Vmax, Acc, Dec, HZ)

time = np.arange(0, prof.T + HZ/10)
Vc = np.zeros_like(time)
Sc = np.zeros_like(time)
Ac = np.zeros_like(time)


for i, t in enumerate(time):
    data = prof.Step(time[i])
    Sc[i], Vc[i] = data.P, data.Pd


print("T = ", prof.T)
print("Ta = ", prof.Ta)
print("Tr = ", prof.Tr)
print("Td = ", prof.Td)

pp.plot(time, Vc * HZ)
pp.plot(time, Sc)
pp.xlabel('Time')
pp.ylabel('P,Pd,Pdd')
pp.show()
