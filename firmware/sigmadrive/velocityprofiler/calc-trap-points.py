import trapezoidprofile as tp
import matplotlib.pyplot as pp
import numpy as np
import sys


Pf = 1000000
Pi = 0
Vi = 0
Vmax = 2000000
Acc = 30000000
Dec = 10000000
HZ = 18000

nargs = len(sys.argv)

Pf = int(sys.argv[1]) if nargs > 1 else Pf
Pi = int(sys.argv[2]) if nargs > 2 else Pi
Vi = int(sys.argv[3]) if nargs > 3 else Vi
Vmax = int(sys.argv[4]) if nargs > 4 else Vmax
Acc = int(sys.argv[5]) if nargs > 5 else Acc
Dec = int(sys.argv[6]) if nargs > 6 else Dec
HZ = int(sys.argv[7]) if nargs > 7 else HZ

points = tp.CalculateTrapezoidPoints(Pf, Pi, Vi, Vmax, Acc, Dec, HZ)

print("             Time,     Velocity,     Position")
for i in range(0, 4):
    print("point ", i, " : ", points[i][0], ", ", points[i][1], ", ", points[i][2])

totalTime = 0
for i in range(0, 4):
    totalTime += points[i][0]


time = np.arange(0, totalTime)
A = np.zeros_like(time, dtype=float)
V = np.zeros_like(time, dtype=float)
S = np.zeros_like(time, dtype=float)
T = np.zeros_like(time, dtype=float)

V2 = 0
offset = 0

for k in range(0, 4):
    point = points[k]
    V1 = V2
    V2 = point[1]
    S2 = point[2]
    T1 = 0
    T2 = point[0]
    if (T2 > T1):
        Acc = (V2 - V1) / (T2 - T1)
        for i in range(0, int(point[0])) :
            A[i + offset] = Acc
            V[i + offset] = V1  + Acc * i
            v = V[i + offset]
            S[i + offset] = S2 - (v + V2) * (T2 - i) / 2
    offset += int(point[0])

pp.figure()
pp.subplot(3,1,1)
pp.plot(time, V, label="Velocity")
pp.ylabel('Velocity')
pp.subplot(3,1,2)
pp.plot(time, S, color="orange", label="Position")
pp.xlabel('Time')
pp.ylabel('Position')
pp.subplot(3,1,3)
pp.plot(time, A, color="green", label="Acceleration")
pp.xlabel('Time')
pp.ylabel('Acceleration')
#pp.legend(loc="right")
pp.show()
