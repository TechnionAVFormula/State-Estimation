# EKF-Slam
import numpy as np
import math as ma
from numpy.random import randn
from Kalman_Check import Check_Line
import matplotlib.pyplot as plt

Sensors_Data, External_sensors, X_Ground_Truth, Y_Ground_Truth, Time, u = Check_Line()

DTime = 0.01
L_Tot = 1.535  # units [m], from Calculations,Weight Transfer file.
L_Rear = 0.7675  # units [m],assuming the weight is distributed equal on the rear and front wheels need
X_init = np.zeros([5, 1])
Alpha = 1


P_init = 0.01 * np.eye(len(X_init))
W_Model = np.diag((0.01 ** 2, 0.01 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2))
W_Observation = np.diag((0.1 ** 2, 0.1 ** 2,))
V = np.diag([0.5 ** 2, 0.1 ** 2])
N = 0
N_Max = 0


def Extend_State_Covariance(X, P, External_sensors, N):
    C = np.array(
        (
            X[0] + External_sensors[0] * ma.cos(External_sensors[1]),
            X[1] + External_sensors[0] * ma.sin(External_sensors[1]),
            N,
        )
    )
    X = np.append(X, C)
    P = np.append(P, 10000 * np.ones((P.shape[0], 2)), 1)
    P = np.append(P, 10000 * np.ones((2, P.shape[1])), 0)

    return X, P


def Prediction(X, P, u, N):
    F = np.eye(5)
    if N > 0:
        F = np.append(F, np.zeros((F.shape[0], 2 * N), 1))
    Beta = np.mod(ma.atan2(ma.tan(u[1]) * L_Rear, L_Tot), 2 * ma.pi)
    V_tot = ma.sqrt(
        ((X[2] + DTime * ma.cos(X[4] + Beta) * u[0]) ** 2)
        + ((X[3] + DTime * ma.sin(X[4] + Beta) * u[0]) ** 2)
    )
    DTheta = V_tot * ma.cos(Beta) * ma.tan(u[1]) / L_Tot
    Movement = np.array(
        [
            DTime * X[2] + (DTime ** 2) / 2 * ma.cos(X[4] + Beta) * u[0],
            DTime * X[3] + (DTime ** 2) / 2 * ma.sin(X[4] + Beta) * u[0],
            DTime * ma.cos(X[4] + Beta) * u[0],
            DTime * ma.sin(X[4] + Beta) * u[0],
            DTime * DTheta,
        ]
    )
    X_Prediction = X + np.transpose(F) @ Movement
    Partial_Jacobian = np.array(
        [
            [0, 0, DTime, 0, -(DTime ** 2) * u[0] * ma.sin(X[4] + Beta)],
            [0, 0, 0, DTime, (DTime ** 2) * u[0] * ma.cos(X[4] + Beta)],
            [0, 0, 0, 0, -DTime * u[0] * ma.sin(X[4] + Beta)],
            [0, 0, 0, 0, DTime * u[0] * ma.cos(X[4] + Beta)],
            [0, 0, 0, 0, 0],
        ]
    )
    Jacobian_Of_Model = np.eye((X.shape[0])) + np.transpose(F) @ Partial_Jacobian @ F
    Jacobian_Of_Control = np.array(
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
                / (L_Tot * (ma.cos(u[1]) ** 2)),
            ],
        ]
    )

    P_Prediction = (
        Jacobian_Of_Model @ P @ np.transpose(Jacobian_Of_Model)
    ) + np.transpose(F) @ (
        Jacobian_Of_Control @ V @ np.transpose(Jacobian_Of_Control) @ F
    )
    return P_Prediction, X_Prediction, Beta


def Correction(Sensors_Data, External_sensors, X_Prediction, P_Prediction, Beta, N):
    F_Model = np.eye(5)
    if N > 0:
        F_Model = np.append(F_Model, np.zeros((F_Model.shape[0], 2 * N), 1))
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
        ],
        dtype="float",
    )
    H_Model = np.array(
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
        ],
        dtype="float",
    )
    H_Model = H_Model @ F_Model
    S = H_Model @ P_Prediction @ np.transpose(H_Model) + W_Model
    Kalman_Gain_Model = P_Prediction @ np.transpose(H_Model) @ np.linalg.inv(S)

    if N > 0:
        j = 0
        Pi = 100000000
        Z_Observation = np.array(
            [[External_sensors[0]], [External_sensors[1]]], dtype="float"
        )
        Observation = np.array(
            [
                X_Prediction[0]
                + Z_Observation[0] * ma.cos(Z_Observation[1] + X_Prediction[4]),
                X_Prediction[1]
                + Z_Observation[0] * ma.sin(Z_Observation[1] + X_Prediction[4]),
            ]
        )
        Observation = np.append(X_Prediction[4:], Observation, 0)
        for i in range(1, N + 1, 2):
            delta = np.array(
                [
                    [Observation[i] - X_Prediction[0]],
                    [Observation[i] - X_Prediction[1]],
                ]
            )
            q = np.transpose(delta) @ delta
            Approximated_Z = np.array(
                [
                    ma.sqrt(q),
                    np.mod(ma.atan2(delta[1], delta[0]), 2 * ma.pi) - X_Prediction[4],
                ],
                dtype="float",
            )
            Hi = np.array(
                [
                    [
                        -ma.sqrt(q) / q * delta[0],
                        -ma.sqrt(q) / q * delta[1],
                        0,
                        0,
                        0,
                        ma.sqrt(q) / q * delta[0],
                        ma.sqrt(q) / q * delta[1],
                    ],
                    [
                        delta[1] / q,
                        -delta[0] / q,
                        0,
                        0,
                        -1,
                        -delta[1] / q,
                        delta[0] / q,
                    ],
                ],
                dtype="float",
            )
            F = np.block(
                [
                    [np.eye(5), np.zeros([5, 2 * N])],
                    [
                        np.zeros(2, 5 + 2 * (i - 1)),
                        np.eye(2),
                        np.zeros(2, 2 * ((i) - N)),
                    ],
                ]
            )
            Hi = Hi @ F
            Si = Hi @ P_Prediction @ np.transpose(Hi) + W_Observation
            if i < N + 1:
                HHi = Hi
                Ss = np.linalg.inv(Si)
                Pi_Check = (
                    np.transpose((Z_Observation - Approximated_Z))
                    @ Ss
                    @ (Z_Observation - Approximated_Z)
                )
                if Pi > Pi_Check:
                    Si_minimal = Hi @ P_Prediction @ np.transpose(Hi) + W_Observation
                    Pi = Pi_Check
                    j = i
        if(Pi_Check<Alpha)
            Kalman_Gain_Observation = P_Prediction @ np.transpose(HHi) @ Ss
        else:
            N = N+1
            Kalman_Gain_Observation = P_Prediction @ np.transpose(Hi) @ np.linalg.inv(Ss)
        # X = X_Prediction + Kalman_Gain_Model @ (Z - X_Prediction) + Kalman_Gain_Observation @(Z - )
    
                

    else:
        X = X_Prediction + Kalman_Gain_Model @ (Z - X_Prediction)
        P = (np.eye(5) - Kalman_Gain_Model @ H_Model) @ P_Prediction
        return X, P


X = np.empty([7, len(Sensors_Data) - 1])
P, Xx, Beta = Prediction(X_init, P_init, u[:, 0], N)
j = 1
for i in range(1, len(Sensors_Data)):
    Xx, P = Correction(Sensors_Data[i, :], External_sensors[i, :], Xx, P, Beta, N)
    P, Xx, Beta = Prediction(Xx, P, u[:, i], N)
    plt.plot(X_Ground_Truth[i], Y_Ground_Truth[i], label="Ground Truth")
    plt.plot(Xx[0], Xx[1], label="State")
    plt.pause(0.05)
    if N_Max < N:
        Extend_State_Covariance(Xx, P, External_sensors[i, :], N)
        N_Max = N_Max + 1

plt.show()

# print("run")
# plt.figure(1)
# plt.plot(X[0, :], X[1, :], label="State")

# plt.legend(["State", "Ground Truth"])
# plt.grid()
# plt.axis("equal")
# plt.figure(2)
# plt.plot(Sensors_Data[:, 0], Sensors_Data[:, 1], "ro", label="Sensors Data")
# plt.plot(X[0, :], X[1, :], label="State")
# plt.plot(X[5, :], X[6, :], "bo", label="Cone")
# plt.legend(["Sensors Data", "State", "Cone"])
# plt.grid()
# plt.axis("equal")

