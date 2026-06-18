import numpy as np

R_base = 0.2      # Base radius [meters]
R_plat = 0.1      # Platform radius [meters]
h0 = 0.15         # Nominal height

angles_deg = np.array([0, 60, 120, 180, 240, 300])
angles_rad = np.deg2rad(angles_deg)

# Base (fixed, in global frame)
B = np.array([[R_base * np.cos(a), R_base * np.sin(a), 0] for a in angles_rad]).T

# Platform (local frame)
P = np.array([[R_plat * np.cos(a), R_plat * np.sin(a), 0] for a in angles_rad]).T

def leg_lengths(R, p):
    L = []
    for i in range(6):
        Pi_world = R @ P[:, i] + p
        li = Pi_world - B[:, i]
        L.append(np.linalg.norm(li))
    return np.array(L)

