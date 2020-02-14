import numpy as np
import math as ma
from numpy.random import randn

DTime = 0.001
L_Tot = 1.535  # units [m], from Calculations,Weight Transfer file.
L_Rear = 0.7675  # units [m],assuming the weight is distributed equal on the rear and front wheels need

X_init = np.zeros([5, 1])
P_init = 0.01 * np.eye(len(X_init))
R = np.array([])
B = np.array([[1], [2], [3], [4], [5]])
W = np.diag([0.5 ** 2, 0.5 ** 2, 0.1 ** 2, 0.1 ** 2, 0.01 ** 2])
print(W)
# Steering imported from the data

# Steering = np.mod(ma.atan2(Xd[1] - X[1], Xd[0] - X[0]) - X[4], 2 * ma.pi)


def KF_Prediction(X, u, Xd, Steering):
    Beta = np.mod(ma.atan2(ma.cos(Steering) * L_Rear, L_Tot), 2 * ma.pi)
    V_tot = np.linalg.norm(X[2:4])
    DTheta = V_tot * ma.cos(Beta) * ma.tan(Steering) / L_Tot
    X_Prediction = np.array(
        [
            [X[0] + DTime * X[2] + (DTime ** 2) / 2 * ma.cos(X[4] + Beta)],
            [X[1] + DTime * X[3] + (DTime ** 2) / 2 * ma.sin(X[3] + Beta)],
            [X[2] + DTime * ma.cos(X[4] + Beta)],
            [X[3] + DTime * ma.sin(X[4] + Beta)],
            [X[4] + DTime * DTheta],
        ]
    )
    J_x = np.array(
        [
            [1, 0, DTime, 0, -(DTime ** 2) * u[0] * ma.sin(X[4] + Beta)],
            [0, 1, 0, DTime, (DTime ** 2) * u[0] * ma.cos(X[4] + Beta)],
            [0, 0, 1, 0, -DTime * u[0] * ma.sin(X[4] + Beta)],
            [0, 0, 1, 0, DTime * u[0] * ma.cos(X[4] + Beta)],
            [0, 0, 0, 0, 1],
        ]
    )
    J_v = np.array(
        [
            [(DTime ** 2) * ma.cos(X[4] + Beta), 0],
            [(DTime ** 2) * ma.sin(X[4] + Beta), 0],
            [DTime * ma.cos(X[4] + Beta), 0],
            [DTime * ma.sin(X[4] + Beta), 0],
            [
                0,
                DTime
                * np.linalg.norm(X[2:4])
                * ma.cos(Beta)
                / (L_Tot * (ma.cos(Steering) ** 2)),
            ],
        ]
    )
    P_Prediction = (J_x @ P @ np.transpose(J_x)) + (J_v @ V @ np.transpose(J_v))

    return P_Prediction, X_Prediction


def KF_Update(Sensors_Data, X_Prediction, Beta, P_Prediction, W):
    Z = np.array(
        [
            [Sensors_Data[0]],
            [Sensors_Data[1]],
            [
                X_Prediction[2]
                + DTime
                * (
                    Sensors_Data[2] * ma.cos(X_Prediction[4] + Beta)
                    + Sensors_Data[3] * ma.sin(X_Prediction[4] + Beta)
                )
            ],
            [
                X_Prediction[3]
                + DTime
                * (
                    Sensors_Data[2] * ma.sin(X_Prediction[4] + Beta)
                    - Sensors_Data[3] * ma.cos(X_Prediction[4] + Beta)
                )
            ],
            [X_Prediction[4] + DTime * Sensors_Data[4]],
        ]
    )
    Hx = np.array(
        [
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [
                0,
                0,
                1,
                0,
                -DTime
                * (
                    Sensors_Data[2] * ma.sin(X_Prediction[4] + Beta)
                    - Sensors_Data[3] * ma.cos(X_Prediction[4] + Beta)
                ),
            ],
            [
                0,
                0,
                0,
                1,
                DTime
                * (
                    Sensors_Data[2] * ma.cos(X_Prediction[4] + Beta)
                    + Sensors_Data[3] * ma.sin(X_Prediction[4] + Beta)
                ),
            ],
            [0, 0, 0, 0, 1],
        ]
    )
    S = Hx @ P_Prediction @ np.transpose(Hx) + W  # Measurement prediction covariance
    K = P_Prediction @ np.transpose(Hx) @ np.linalg.inv(S)  # KalmanGain
    X = X_Prediction + K @ (Z - X_Prediction)  # X update
    P = P_Prediction - K @ np.transpose(Hx) @ P_Prediction  # Covariance update
    return X, P


P = 0.1 * np.eye(5)
V = 0.01 * np.eye(2)
X = np.array([[1], [2], [2], [2], [0]])
S = np.array([[1.1], [2.1], [10], [0.1], [0.2]])
B = u = np.array([[10], [0]])
Xd = np.array([[11], [9], [5], [12]])
# Xx, Beta, Steering = Prediction5d(X, B, Xd)
# Zz, Hx = Observation_matrix(S, X, Beta)
# P_Pre = Covariance_matrix(P, V, X, B, Beta, Steering)
