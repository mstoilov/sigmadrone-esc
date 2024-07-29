import trapezoidprofile as tp
import matplotlib.pyplot as pp
import numpy as np
import sys

def sign_hard(x):
    return 1.0 if (x >= 0) else -1.0

def SQ(x):
    return x*x

# Xf	Position (final)
# Xi	Position (initial)
# Vi	Velocity (initial) at Pi
# Vmax	Maximum allowed velocity
# Amax	Acceleration
# Dmax	Deceleration
# t		time
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
    Da = 0.5 * (Vr + Vi) * (Vr - Vi) / Ar   # Displacement during acceleration
    Dd = 0.5 * SQ(Vr) / Dr                  # Displacement during deceleration
    Dc = dX - (Da + Dd)                     # Constant displacement
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

    
    Ta = (Vr - Vi) / Ar			# Time for acceleration
    Td = Vr / Dr				# Time for deceleration
    Tc = Dc / Vr				# Time for constant velocity
    T = Ta + Tc + Td			# Total time for movement

    if (t < Ta):
        # Acceleration phase
        Ydd = s * Ar
        Yd = s * (Vi + Ar * t)
        Y   = Xi + s * (Vi * t + 0.5 * Ar * SQ(t))
    elif t < (Ta + Tc):
        # Const velocity phase
        Ydd = 0
        Yd = s * Vr
        Y   = Xi + s * (Vi * Ta + 0.5 * Ar * SQ(Ta) + Vr * (t - Ta))
    elif (t < T):
        # Deceleration phase
        tc = t - (Ta + Tc)
        Ydd = - s * Dr
        Yd = s * (Vr - Dr * tc)
        Y = Xi + s * (Vi * Ta + 0.5 * Ar * SQ(Ta) + Vr * Tc + Vr * tc - 0.5 * Dr * SQ(tc))
    else:
        Ydd = 0
        Yd = 0
        Y = Xf
    return Y, Yd, Ydd


Pf = 3000000
Pi = 0
Vi = 0
Vmax = 2000000
Acc = 4000000
Dec = 1500000
T = 3.0

nargs = len(sys.argv)

Pf = float(sys.argv[1]) if nargs > 1 else Pf				# Position (final)
Pi = float(sys.argv[2]) if nargs > 2 else Pi				# Position (initial)
Vi = float(sys.argv[3]) if nargs > 3 else Vi				# Velocity (initial) at Pi
Vmax = float(sys.argv[4]) if nargs > 4 else Vmax			# Maximum allowed velocity
Acc = float(sys.argv[5]) if nargs > 5 else Acc			# Acceleration
Dec = float(sys.argv[6]) if nargs > 6 else Dec			# Deceleration
T = float(sys.argv[7]) if nargs > 7 else T				# period


time = np.linspace(0, T, 500)
Vc = np.zeros_like(time)
Sc = np.zeros_like(time)
Ac = np.zeros_like(time)


for i, t in enumerate(time):
    Sc[i], Vc[i], Ac[i] = trapezoidalProfile(Pf, Pi, Vi, Vmax, Acc, Dec, time[i])


# print("T = ", prof.T)
# print("Ta = ", prof.Ta)
# print("Tr = ", prof.Tr)
# print("Td = ", prof.Td)

pp.plot(time, Sc, label="Position")
pp.plot(time, Vc, label="Velocity")
pp.plot(time, Ac, label="Acceleration")
pp.legend(loc="upper right")
pp.xlabel('Time')
pp.ylabel('P,V,A')
pp.show()
