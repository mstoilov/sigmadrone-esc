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

Pf = int(sys.argv[1]) if nargs > 1 else Pf
Pi = int(sys.argv[2]) if nargs > 2 else Pi
Vi = int(sys.argv[3]) if nargs > 3 else Vi
Vmax = int(sys.argv[4]) if nargs > 4 else Vmax
Acc = int(sys.argv[5]) if nargs > 5 else Acc
Dec = int(sys.argv[6]) if nargs > 6 else Dec
HZ = int(sys.argv[7]) if nargs > 7 else HZ

prof = tp.TrapezoidProfile();
points = prof.CalcTrapPoints(Pf, Pi, Vi, Vmax, Acc, Dec, HZ)

time = np.arange(0, 4)
V = np.zeros_like(time)
S = np.zeros_like(time)
T = np.zeros_like(time)


for i, t in enumerate(time):
    if (t == 0):
        V[t] = Vi/HZ
        S[t] = Pi
        T[t] = 0
    else:
        T[t], V[t], S[t] = points[t-1].time, points[t-1].velocity, points[t-1].position


for i in range(0, 3):
    print("point ", i, " : ", points[i].time, ", ", points[i].velocity, ", ", points[i].position)

pp.figure()
pp.subplot(2,1,1)
pp.plot(T, V)
pp.subplot(2,1,2)
pp.plot(T, S, color="orange")
pp.xlabel('Time')
pp.ylabel('Position')
pp.show()
