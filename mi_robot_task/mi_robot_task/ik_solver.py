#!/usr/bin/env python3
import math

Z0 = 0.400   # Altura hombro (joint_2): 0.10 + 0.30
L1 = 0.400   # Longitud brazo superior (link_2)
L2 = 0.445   # Longitud antebrazo + muñeca + punta de pinza (0.30 + 0.10 + 0.045)

JOINT_LIMITS = [
    (-3.14, 3.14),   # joint_1
    (-1.57, 1.57),   # joint_2
    (-2.50, 2.50),   # joint_3
    (-3.14, 3.14),   # joint_4
]

def forward_kinematics(q):
    q1, q2, q3, _ = q
    r = L1 * math.sin(q2) + L2 * math.sin(q2 + q3)
    z = Z0 + L1 * math.cos(q2) + L2 * math.cos(q2 + q3)
    x = r * math.cos(q1)
    y = r * math.sin(q1)
    return (x, y, z)

def solve_ik(x, y, z, wrist_yaw=0.0):
    if z < 0.035:
        z = 0.035

    q1 = math.atan2(y, x)
    r = math.hypot(x, y)
    dz = z - Z0

    d_sq = r * r + dz * dz
    d = math.sqrt(d_sq)

    if d > (L1 + L2 - 0.005) or d < abs(L1 - L2) + 0.01:
        return None, None

    cos_q3 = (d_sq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_q3 = max(-1.0, min(1.0, cos_q3))
    q3 = math.acos(cos_q3)

    alpha = math.atan2(r, dz)
    k1 = L1 + L2 * math.cos(q3)
    k2 = L2 * math.sin(q3)
    beta = math.atan2(k2, k1)

    q2 = alpha - beta
    q4 = wrist_yaw

    q = [q1, q2, q3, q4]
    for val, (low, high) in zip(q, JOINT_LIMITS):
        if val < low or val > high:
            return None, None

    fx, fy, fz = forward_kinematics(q)
    err_mm = math.sqrt((fx - x)**2 + (fy - y)**2 + (fz - z)**2) * 1000.0
    return q, err_mm
