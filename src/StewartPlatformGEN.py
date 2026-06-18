import numpy as np
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from numpy.linalg import svd

def rotate_angles(angles, rotation):
    # angles: list of angles in degrees
    # rotation: scalar angle to add (can be negative)
    shifted = np.roll(angles, -1)
    rotated = (shifted + rotation) % 360
    return rotated

def angle_to_points(angles_deg, r):
    angles_rad = np.deg2rad(angles_deg)
    x = r * np.cos(angles_rad)
    y = r * np.sin(angles_rad)
    return np.vstack((x, y))  # 2xN array

def compute_condition_number(B, P):
    # Assume vertical actuators (simplified Jacobian estimation)
    legs = P - B
    J = legs / np.linalg.norm(legs, axis=0)
    _, s, _ = svd(J)
    return s[0] / s[-1] if s[-1] > 1e-12 else np.inf

# Parameters
alpha = 20 # degrees
beta = 120 - alpha

thetaB = [
    5/2*alpha + beta*3,
    alpha/2,
    alpha/2 + beta,
    3/2*alpha + beta,
    3/2*alpha + beta*2,
    5/2*alpha + beta*2
]
thetaB = np.array(thetaB)
thetaP = rotate_angles(thetaB, -60)

## SETUP PLOT
radiusB = 200.00/2
radiusP = radiusB
legLength = 260

points = angle_to_points(thetaB, radiusB)
points2 = angle_to_points(thetaP, radiusP)

# # Plot 2D
# plt.figure()
# plt.axis('equal')
# plt.grid(True)
# plt.scatter(points[0], points[1], s=100, c='b', label='thetaB')
# plt.scatter(points2[0], points2[1], s=100, c='r', label='thetaP')
# plt.title('Points on Circle from Angles')
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.legend()
# plt.show()

# Compute condition number
condition_number = compute_condition_number(points, points2)
print(f"Condition number: {condition_number:.4f}")

# Save config to JSON
config = {
    "Rb": radiusB,
    "Rp": radiusP,
    "legLength": legLength,
    "thetaB": thetaB.tolist(),
    "thetaP": thetaP.tolist(),
    "conditionNumber": condition_number
}

with open("SPS01_CONFIG.json", "w") as f:
    json.dump(config, f, indent=4)

# 3D Plot of Platform
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

zB = np.zeros(6)
zP = np.ones(6) * legLength

for i in range(6):
    ax.plot([points[0, i], points2[0, i]], [points[1, i], points2[1, i]], [zB[i], zP[i]], 'k-')

ax.scatter(points[0], points[1], zB, c='b', label='Base')
ax.scatter(points2[0], points2[1], zP, c='r', label='Platform')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Stewart Platform Configuration')
ax.legend()
plt.tight_layout()
plt.show()
