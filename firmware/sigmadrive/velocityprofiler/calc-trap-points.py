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

prof = tp.TrapezoidProfile()
points = prof.CalcTrapPoints(Pf, Pi, Vi, Vmax, Acc, Dec, HZ)

print("             Time,     Velocity,     Position")
for i in range(0, 4):
    print("point ", i, " : ", points[i].time, ", ", points[i].velocity, ", ", points[i].position)

totalTime = 0
for i in range(0, 4):
    totalTime += points[i].time

time = np.arange(0, totalTime)
V = np.zeros_like(time, dtype=float)
S = np.zeros_like(time, dtype=float)
T = np.zeros_like(time, dtype=float)

V2 = 0
offset = 0

for k in range(0, 4):
    point = points[k]
    V1 = V2
    V2 = point.velocity
    S2 = point.position
    T1 = 0
    T2 = point.time
    if (T2 > T1):
        A = (V2 - V1) / (T2 - T1)
        for i in range(0, point.time) :
            v = V[i + offset] = V1  + A * i
            S[i + offset] = S2 - (v + V2) * (T2 - i) / 2
    offset += point.time
    

pp.figure()
pp.subplot(2,1,1)
pp.plot(time, V, label="Velocity")
pp.ylabel('Velocity')
#pp.legend(loc="upper right")
pp.subplot(2,1,2)
pp.plot(time, S, color="orange", label="Position")
pp.xlabel('Time')
pp.ylabel('Position')
#pp.legend(loc="right")
pp.show()
