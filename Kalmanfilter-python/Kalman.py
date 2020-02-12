import numpy as np
from math import *
from numpy.random import randn

DTime = 0.001
L_Tot = 1.535  # units [m], from Calculations,Weight Transfer file.
L_Rear = 0.7675  # units [m],assuming the weight is distributed equal on the rear and front wheels need

X_init = np.zeros([5, 1])
P_init = 0.01 * np.eye(len(X_init))
R = np.array([])
B = np.array([[1], [2], [3], [4], [5]])


def Prediction5d(X, u, Xd):
    Steering = np.mod(atan2(Xd[1] - X[1], Xd[0] - X[0]) - X[4], 2 * pi)
    Beta = np.mod(atan2(cos(Steering) * L_Rear, L_Tot), 2 * pi)
    V_tot = np.linalg.norm(X[2:4])
    DTheta = V_tot * cos(Beta) * tan(Steering) / L_Tot
    X_Prediction = np.array(
        [
            [X[0] + DTime * X[2] + (DTime ** 2) / 2 * cos(X[4] + Beta)],
            [X[1] + DTime * X[3] + (DTime ** 2) / 2 * sin(X[3] + Beta)],
            [X[2] + DTime * cos(X[4] + Beta)],
            [X[3] + DTime * sin(X[4] + Beta)],
            [X[4] + DTime * DTheta],
        ]
    )
    return X_Prediction, Beta, Steering


def Covariance_matrix(P, V, X, u, Beta, Steering):
    J_x = np.array(
        [
            [1, 0, DTime, 0, -(DTime ** 2) * u[0] * sin(X[4] + Beta)],
            [0, 1, 0, DTime, (DTime ** 2) * u[0] * cos(X[4] + Beta)],
            [0, 0, 1, 0, -DTime * u[0] * sin(X[4] + Beta)],
            [0, 0, 1, 0, DTime * u[0] * cos(X[4] + Beta)],
            [0, 0, 0, 0, 1],
        ]
    )
    J_v = np.array(
        [
            [(DTime ** 2) * cos(X[4] + Beta), 0],
            [(DTime ** 2) * sin(X[4] + Beta), 0],
            [DTime * cos(X[4] + Beta), 0],
            [DTime * sin(X[4] + Beta), 0],
            [
                0,
                DTime
                * np.linalg.norm(X[2:4])
                * cos(Beta)
                / (L_Tot * (cos(Steering) ** 2)),
            ],
        ]
    )
    P_Pre = (J_x @ P @ np.transpose(J_x)) + (J_v @ V @ np.transpose(J_v))
    return P_Pre


P = 0.1 * np.eye(5)
V = 0.01 * np.eye(2)
X = np.array([[1], [2], [2], [2], [0]])
B = u = np.array([[10000], [0]])
Xd = np.array([[11], [9], [5], [12]])
Xx, Beta, Steering = Prediction5d(X, B, Xd)
P_Pre = Covariance_matrix(P, V, X, B, Beta, Steering)
print(P_Pre)
