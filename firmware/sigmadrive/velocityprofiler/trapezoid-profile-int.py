import trapezoidprofile as tp
import matplotlib.pyplot as pp
import numpy as np


time = np.arange(0, 20000)
Vc = np.zeros_like(time)
Sc = np.zeros_like(time)
Ac = np.zeros_like(time)

HZ = 18000
prof = tp.TrapezoidProfileInt();
prof.Init(1000000, 0, 0, 65535 * 30, 20000000, 20000000, HZ)
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
