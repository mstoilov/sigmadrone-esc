import trapezoidprofile as tp
import matplotlib.pyplot as pp
import numpy as np
import sys


S = 1000000
Vi = 0
Vmax = 65535 * 30
Acc = 40000000
Dec = 5000000
HZ = 18000

nargs = len(sys.argv)

S = int(sys.argv[1]) if nargs > 1 else S
Vi = int(sys.argv[2]) if nargs > 2 else Vi
Vmax = int(sys.argv[3]) if nargs > 3 else Vmax
Acc = int(sys.argv[4]) if nargs > 4 else Acc
Dec = int(sys.argv[5]) if nargs > 5 else Dec
HZ = int(sys.argv[6]) if nargs > 6 else HZ

prof = tp.TrapezoidProfile();
points = prof.CalcTrapPoints(S, Vi, Vmax, Acc, Dec, HZ)

time = np.arange(0, 4)
V = np.zeros_like(time)
T = np.zeros_like(time)


for i, t in enumerate(time):
    if (t == 0):
        V[0] = Vi/HZ
        T[0] = 0
    else:
        T[t], V[t] = points[t-1].time + T[t-1], points[t-1].velocity


for i in range(0, 3):
    print("point ", i, " : ", points[i].time, ", ", points[i].velocity)

pp.plot(T, V)
pp.xlabel('Time')
pp.ylabel('Velocity')
pp.show()
