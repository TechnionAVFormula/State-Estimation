import numpy as np
import math
from numpy.random import randn


DTime = 0.001
L_Tot = 1.535  # units [m], from Calculations,Weight Transfer file.
L_Rear = 0.7675  # units [m],assuming the weight is distributed equal on the rear and front wheels need
# need to be check.
Gps_sig = 0.5  # the error of the GPS 0.5[m]
angular_vel_sig = 0.01  # angular velocity error error 0.1[rad/sec]
Steering_sig = 0.01  # The angle error for steering 0.01[rad]

# Observation State relation matrix

H = np.array([[1, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 0, 0, 0]])
print(H)
A = A = np.array(
    [
        [1, 0, DTime, 0, 0],
        [0, 1, 0, DTime, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1],
    ]
)
X_init = np.zeros([5, 1])
P_init = 0.01 * np.eye(len(X_init))
R = np.array([])
print(P_init)


def Prediction5d(X, u, Xd):
    Steering = np.mod(math.atan2(Xd[1] - X[1], Xd[0] - X[0]) - X[4], 2 * math.pi)
    Beta = np.mod(math.atan2(math.cos(Steering) * L_Rear, L_Tot), 2 * math.pi)
    u[1] = (
        np.linalg.norm(X[2:4]) * math.cos(Beta) * math.tan(Steering) / L_Tot
    )  # Theta_dot
    B = np.array(
        [
            [(DTime ** 2) / 2 * math.cos(X[4]), 0],
            [(DTime ** 2) / 2 * math.sin(X[4]), 0],
            [DTime * math.cos(X[4]), 0],
            [DTime * math.sin(X[4]), 0],
            [0, DTime],
        ],
    )
    X_Prediction = (A @ X) + (B @ u)
    return X_Prediction


X = np.array([[1], [2], [2], [2], [0]])
B = u = np.array([[1], [0]])
Xd = np.array([[11], [9], [5], [12]])
print(Prediction5d(X, B, Xd))
