import matplotlib.pyplot as pp
import numpy as np
import trapezoidprofile as tp
import sys


# Sin   - initial position              [ec]
# Sfin  - final position                [ec]
# Vin   - initial velocity              [ec/sec]
# Vfin  - final velocity                [ec/sec]
# Vmax  - max velocity allowed          [ec/sec]
# Accel - acceleration                  [ec/(sec*sec)]
# Decel - deceleration                  [ec/(sec*sec)]
# Hz    - Closed loop update frequency  [Hz]
def CalculateTrapezoidPoints(Sin, Sfin, Vin, Vfin, Vmax, Accel, Decel, Hz):
    squareHz = Hz * Hz
    s = float(1.0) if Sfin >= Sin else float(-1.0)  # Direction of the trajectory
    Vr = float(np.abs(Vmax))                        # Requested velocity absolute value
    Ar = float(np.abs(Accel))                       # Requested acceleration absolute value
    Dr = float(np.abs(Decel))                       # Requested deceleration absolute value
    dS = float(np.abs(Sfin - Sin))                  # Total displacement absolute value
    Vi = float(s * Vin )                            # Initial speed
    Vf = float(s * Vfin )                           # Final speed

    if (Vi > Vr):
        Ar = -Ar
    Da = 0.5 * (Vr + Vi) * (Vr - Vi) / Ar if (Ar) else 0.0          # Displacement during acceleration
    Dd = 0.5 * (Vr + Vf) * (Vr - Vf) / Dr if (Dr) else 0.0          # Displacement during deceleration
    Dc = dS - (Da + Dd)                                             # Displacement during const velocity

    if (Dc < 0):
        # Find Vr by solving:
        # Da + Dd = dX
        # 0.5 * (Vr + Vi) * (Vr - Vi) / Ar + 0.5 *(Vr * Vr) / Dr = dX
        Vr_sq = Dr * (2.0 * Ar * dS + np.square(Vi))/(Dr + Ar)
        if (Vr_sq < 0 or Da == 0):
            # The distance to the requested position is too short
            # for the specified decelaration.
            # Calculate the required decelaration to reach the position
            Vr = Vi
            Dr = 0.5 * np.square(Vi) / dS
        else:
            Vr = np.sqrt(Vr_sq)
        Dc = 0
        Da = 0.5 * (Vr + Vi) * (Vr - Vi) / Ar
        Dd = 0.5 * (Vr + Vf) * (Vr - Vf) / Dr

    Ta = (Vr - Vi) / Ar if (Ar) else 0.0
    Tr = Dc / Vr if (Vr) else 0
    Td = (Vr - Vf) / Dr if (Dr) else 0.0

    pt0t = 0
    pt0v = Vi
    pt0p = Sin

    pt1t = Ta
    pt1v = s * Vr
    pt1p = Sin + (Vi + pt1v) * Ta * 0.5

    pt2t = Tr
    pt2v = s * Vr
    pt2p = pt1p + (pt1v + pt2v) * Tr * 0.5

    pt3t = Td
    pt3v = Vf
    pt3p = Sfin

    #
    # Return the 4 points of the trapezoid
    # The time (col 0) is specified in time slices
    # The velocity (col 1) is in encoder counts / second (it needs to be converted to ec/timeslice)
    # The position (col 2) is in encoder counts
    return [[int(pt0t * Hz), int(pt0v), int(pt0p)], 
            [int(pt1t * Hz), int(pt1v), int(pt1p)], 
            [int(pt2t * Hz), int(pt2v), int(pt2p)], 
            [int(pt3t * Hz), int(pt3v), int(pt3p)]]


# Xin   - initial position              [ec]
# Yin   - initial position              [ec]
# Xfin  - final position                [ec]
# Yfin  - final position                [ec]
# Vmax  - max velocity allowed          [ec/sec]
# Accel - acceleration                  [ec/(sec*sec)]
# Decel - deceleration                  [ec/(sec*sec)]
# Hz    - Closed loop update frequency  [Hz]
def CalculateTrapezoidPointsXY(Xin, Yin, Xfin, Yfin, Vmax, Accel, Decel, Hz):
    Vr = float(np.abs(Vmax))                        # Requested velocity absolute value
    Ar = float(np.abs(Accel))                       # Requested acceleration absolute value
    Dr = float(np.abs(Decel))                       # Requested deceleration absolute value
    dX = float(np.abs(Xfin - Xin))
    dY = float(np.abs(Yfin - Yin))
    dS = np.sqrt(dX * dX + dY * dY)                 # Total displacement absolute value
    Vi = 0                                          # Initial speed
    Vf = 0                                          # Final speed
    angle = np.arctan2(Yfin - Yin, Xfin - Xin)

    if (Vi > Vr):
        Ar = -Ar
    Da = 0.5 * (Vr + Vi) * (Vr - Vi) / Ar if (Ar) else 0.0          # Displacement during acceleration
    Dd = 0.5 * (Vr + Vf) * (Vr - Vf) / Dr if (Dr) else 0.0          # Displacement during deceleration
    Dc = dS - (Da + Dd)                                             # Displacement during const velocity

    if (Dc < 0):
        # Find Vr by solving:
        # Da + Dd = dX
        # 0.5 * (Vr + Vi) * (Vr - Vi) / Ar + 0.5 *(Vr * Vr) / Dr = dX
        Vr_sq = Dr * (2.0 * Ar * dS + np.square(Vi))/(Dr + Ar)
        if (Vr_sq < 0 or Da == 0):
            # The distance to the requested position is too short
            # for the specified decelaration.
            # Calculate the required decelaration to reach the position
            Vr = Vi
            Dr = 0.5 * np.square(Vi) / dS
        else:
            Vr = np.sqrt(Vr_sq)
        Dc = 0
        Da = 0.5 * (Vr + Vi) * (Vr - Vi) / Ar
        Dd = 0.5 * (Vr + Vf) * (Vr - Vf) / Dr

    Ta = (Vr - Vi) / Ar if (Ar) else 0.0
    Tr = Dc / Vr if (Vr) else 0
    Td = (Vr - Vf) / Dr if (Dr) else 0.0

    pt0t = 0
    pt0v = 0
    pt0p = 0

    pt1t = Ta
    pt1v = Vr
    pt1p = (Vi + pt1v) * Ta * 0.5

    pt2t = Tr
    pt2v = Vr
    pt2p = pt1p + (pt1v + pt2v) * Tr * 0.5

    pt3t = Td
    pt3v = Vf
    pt3p = dS


    #
    # Return the 4 points of the trapezoid
    # The time (col 0) is specified in time slices
    # The velocity (col 1) is in encoder counts / second (it needs to be converted to ec/timeslice)
    # The position (col 2) is in encoder counts
    ptsX = [[int(pt0t * Hz), int(pt0v * np.cos(angle)), int(pt0p * np.cos(angle) + Xin)], 
            [int(pt1t * Hz), int(pt1v * np.cos(angle)), int(pt1p * np.cos(angle) + Xin)], 
            [int(pt2t * Hz), int(pt2v * np.cos(angle)), int(pt2p * np.cos(angle) + Xin)], 
            [int(pt3t * Hz), int(pt3v * np.cos(angle)), int(pt3p * np.cos(angle) + Xin)]]
    ptsY = [[int(pt0t * Hz), int(pt0v * np.sin(angle)), int(pt0p * np.sin(angle) + Yin)], 
            [int(pt1t * Hz), int(pt1v * np.sin(angle)), int(pt1p * np.sin(angle) + Yin)], 
            [int(pt2t * Hz), int(pt2v * np.sin(angle)), int(pt2p * np.sin(angle) + Yin)], 
            [int(pt3t * Hz), int(pt3v * np.sin(angle)), int(pt3p * np.sin(angle) + Yin)]]
    return (ptsX, ptsY)

if __name__ == "__main__":
    nargs = len(sys.argv)
    Si = int(sys.argv[1]) if nargs > 1 else 0
    Sf = int(sys.argv[2]) if nargs > 2 else 10000000
    Vi = int(sys.argv[3]) if nargs > 3 else 0
    Vf = int(sys.argv[4]) if nargs > 4 else 0
    Vmax = int(sys.argv[5]) if nargs > 5 else 2000000
    Acc = int(sys.argv[6]) if nargs > 6 else 3000000
    Dec = int(sys.argv[7]) if nargs > 7 else 1000000
    HZ = int(sys.argv[8]) if nargs > 8 else 20000
    PULSEN = int(sys.argv[9]) if nargs > 9 else 256

    profile = tp.CalculateTrapezoidPoints(Si, Sf, Vi, Vf, Vmax, Acc, Dec, HZ)
    points = np.array(profile, dtype=int)
    Tcol = points[0:5, 0]
    Vcol = points[0:5, 1]
    Scol = points[0:5, 2]
    time = np.arange(0, np.sum(Tcol))
    A = np.zeros_like(time, dtype=float)
    V = np.zeros_like(time, dtype=float)
    S = np.zeros_like(time, dtype=float)
    P = np.zeros_like(time, dtype=float)

    V2 = 0
    time_offset = 0
    Scur = 0
    for k in range(0, len(Tcol)):
        V1 = V2
        V2 = Vcol[k] / HZ
        S2 = Scol[k]
        T1 = 0
        T2 = Tcol[k]
        if (T2 > T1):
            Acc = (V2 - V1) / (T2 - T1)
            for i in range(0, int(Tcol[k])) :
                A[i + time_offset] = Acc
                V[i + time_offset] = V1  + Acc * i
                v = V[i + time_offset]
                S[i + time_offset] = S2 - (v + V2) * (T2 - i) / 2
                if Si < Sf and S[i + time_offset] - Scur > PULSEN:
                    Scur += PULSEN
                    P[i + time_offset] = PULSEN
                elif Si > Sf and S[i + time_offset] - Scur < -PULSEN:
                    Scur += -PULSEN
                    P[i + time_offset] = -PULSEN
        time_offset += int(Tcol[k])

    print(profile)
    print(points)
    pp.figure()
    pp.subplot(4,1,1)
    pp.plot(time, V, label="Velocity")
    pp.ylabel('Velocity')
    pp.subplot(4,1,2)
    pp.plot(time, S, color="orange", label="Position")
    pp.ylabel('Position')
    pp.subplot(4,1,3)
    pp.plot(time, A, color="green", label="Acceleration")
    pp.ylabel('Acceleration')
    pp.subplot(4,1,4)
    pp.plot(time, P, color="indianred", label="Pulses")
    pp.xlabel('Time')
    pp.ylabel('Pulses')

    pp.show()
