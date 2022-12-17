import trapezoidprofile as tp
import matplotlib.pyplot as pp
import numpy as np
import sys

def sign_hard(x):
    return 1.0 if (x >= 0) else -1.0

def SQ(x):
    return x*x

# Pf	Position (final)
# Pi	Position (initial)
# Vi	Velocity (initial) at Pi
# Vmax	Maximum allowed velocity
# Amax	Acceleration
# Dmax	Deceleration
# H		Update frequency
Pf = 1000000
Pi = 0
Vi = 0
Vmax = 65535 * 30
Acc = 40000000
Dec = 5000000
HZ = 18000

nargs = len(sys.argv)

Pf = int(sys.argv[1]) if nargs > 1 else Pf				# Position (final)
Pi = int(sys.argv[2]) if nargs > 2 else Pi				# Position (initial)
Vi = int(sys.argv[3]) if nargs > 3 else Vi				# Velocity (initial) at Pi
Vmax = int(sys.argv[4]) if nargs > 4 else Vmax			# Maximum allowed velocity
Acc = int(sys.argv[5]) if nargs > 5 else Acc			# Acceleration
Dec = int(sys.argv[6]) if nargs > 6 else Dec			# Deceleration
HZ = int(sys.argv[7]) if nargs > 7 else HZ				# Update frequency

prof = tp.TrapezoidProfile()
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
pp.ylabel('P,Pd')
pp.show()
