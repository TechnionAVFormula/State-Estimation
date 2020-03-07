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
Alpha = 5


P_init = 0 * np.zeros(len(X_init))
W_Model = np.diag((0.01 ** 2, 0.01 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2))
W_Observation = np.diag(
    (0.01 ** 2, 0.01 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2)
)
V = np.diag([0.5 ** 2, 0.1 ** 2])
N = 0
N_Max = 0


def Extend_State_Covariance(X, P, External_sensors, N):

    C = np.array(
        (
            X[0] + External_sensors[0] * ma.cos(External_sensors[1]),
            X[1] + External_sensors[0] * ma.sin(External_sensors[1]),
        )
    )
    X = np.append(X, C, 0)
    # P = np.append(P, np.zeros((P.shape[0], 2)), 1)
    # P = np.append(P, np.zeros((2, P.shape[1])), 0)
    P = np.block(
        [
            [P, np.zeros([P.shape[0], 2])],
            [np.zeros([2, P.shape[0]]), 100000 * np.eye(2)],
        ]
    )
    return X, P


def Prediction(X, P, u, N):
    F = np.eye(5)
    if N > 0:
        F = np.append(F, np.zeros((F.shape[0], 2 * N)), 1)
    Beta = np.mod(ma.atan2(ma.tan(u[1]) * L_Rear, L_Tot), 2 * ma.pi)
    V_tot = ma.sqrt(
        ((X[2] + DTime * ma.cos(X[4] + Beta) * u[0]) ** 2)
        + ((X[3] + DTime * ma.sin(X[4] + Beta) * u[0]) ** 2)
    )
    DTheta = V_tot * ma.cos(Beta) * ma.tan(u[1]) / L_Tot
    Movement = np.array(
        [
            [DTime * X[2] + (DTime ** 2) / 2 * ma.cos(X[4] + Beta) * u[0]],
            [DTime * X[3] + (DTime ** 2) / 2 * ma.sin(X[4] + Beta) * u[0]],
            [DTime * ma.cos(X[4] + Beta) * u[0]],
            [DTime * ma.sin(X[4] + Beta) * u[0]],
            [DTime * DTheta],
        ],
        dtype="float",
    )
    X_Prediction = X + np.transpose(F) @ Movement
    Partial_Jacobian = np.array(
        [
            [0, 0, DTime, 0, -(DTime ** 2) * u[0] * ma.sin(X[4] + Beta)],
            [0, 0, 0, DTime, (DTime ** 2) * u[0] * ma.cos(X[4] + Beta)],
            [0, 0, 0, 0, -DTime * u[0] * ma.sin(X[4] + Beta)],
            [0, 0, 0, 0, DTime * u[0] * ma.cos(X[4] + Beta)],
            [0, 0, 0, 0, 0],
        ],
        dtype="float",
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
        ],
        dtype="float",
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
        F_Model = np.append(F_Model, np.zeros((F_Model.shape[0], 2 * N)), 1)
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
    if N > 0:
        Pi = 1000
        Z_Observation = np.array(
            [[External_sensors[0]], [External_sensors[1]]], dtype="float"
        )
        Z_Observation = np.append(Z, Z_Observation, 0)
        # Observation = np.array(
        #     [
        #         X_Prediction[0]
        #         + Z_Observation[0] * ma.cos(Z_Observation[1] + X_Prediction[4]),
        #         X_Prediction[1]
        #         + Z_Observation[0] * ma.sin(Z_Observation[1] + X_Prediction[4]),
        #     ]
        # )
        Observation = X_Prediction[5:]
        for i in range(0, N, 2):
            delta = np.array(
                [
                    Observation[i] - X_Prediction[0],
                    Observation[i + 1] - X_Prediction[1],
                ],
                dtype="float",
            )
            q = np.transpose(delta) @ delta
            Approximated_Z = np.array(
                [
                    [ma.sqrt(q)],
                    [np.mod(ma.atan2(delta[1], delta[0]), 2 * ma.pi) - X_Prediction[4]],
                ],
                dtype="float",
            )
            Approximated_Z = np.append(X_Prediction[:5], Approximated_Z, 0)
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
                        np.zeros([2, 5 + 2 * i]),
                        np.eye(2),
                        np.zeros([2, 2 * (N - 1 - i)]),
                    ],
                ],
            )
            print(F.shape)
            print(Hi.shape)
            print(H_Model.shape)
            Hi = np.append(H_Model, Hi, 0)
            print(Hi)
            Si = Hi @ P_Prediction @ np.transpose(Hi) + W_Observation
            Ss = np.linalg.inv(Si)
            Pi_Check = (
                np.transpose((Z_Observation - Approximated_Z))
                @ Ss
                @ (Z_Observation - Approximated_Z)
            )
            print((Z_Observation - Approximated_Z))
            if Pi > Pi_Check:
                H_Minimal = Hi
                Si_Minimal = Ss
                Z_Liklihood = Approximated_Z
                Pi = Pi_Check
        if Pi < Alpha:
            Kalman_Gain_Observation = (
                P_Prediction @ np.transpose(H_Minimal) @ Si_Minimal
            )
            X = X_Prediction + Kalman_Gain_Observation @ (Z_Observation - Z_Liklihood)
            P = (
                np.eye(X.shape[0]) - Kalman_Gain_Observation @ H_Minimal
            ) @ P_Prediction
        else:
            H_Model = H_Model @ F_Model
            S = H_Model @ P_Prediction @ np.transpose(H_Model) + W_Model
            Kalman_Gain_Model = P_Prediction @ np.transpose(H_Model) @ np.linalg.inv(S)
            N = N + 1
            X = X_Prediction + Kalman_Gain_Model @ (Z - X_Prediction[:5])
            P = (np.eye(X.shape[0]) - Kalman_Gain_Model @ H_Model) @ P_Prediction
    else:
        H_Model = H_Model @ F_Model

        S = H_Model @ P_Prediction @ np.transpose(H_Model) + W_Model
        Kalman_Gain_Model = P_Prediction @ np.transpose(H_Model) @ np.linalg.inv(S)
        X = X_Prediction + Kalman_Gain_Model @ (Z - X_Prediction)
        P = (np.eye(5) - Kalman_Gain_Model @ H_Model) @ P_Prediction
        N = N + 1
    return X, P, N


X_init = np.zeros([5, 1])
X = np.empty([7, len(Sensors_Data) - 1])
P, Xx, Beta = Prediction(X_init, P_init, u[:, 0], N)
j = 1
for i in range(1, len(Sensors_Data)):
    Xx, P, N = Correction(Sensors_Data[i, :], External_sensors[i, :], Xx, P, Beta, N)
    if N_Max < N:
        Xx, P = Extend_State_Covariance(Xx, P, External_sensors[i, :], N)
        N_Max = N_Max + 1
    P, Xx, Beta = Prediction(Xx, P, u[:, i], N)

print("run")
plt.figure(1)
# plt.plot(X_Ground_Truth[:], Y_Ground_Truth[:], label="Ground Truth")
plt.plot(X[0, :], X[1, :], label="State")
plt.legend(["Ground Truth", "State"])
plt.grid()
plt.axis("equal")
plt.figure(2)
plt.plot(Sensors_Data[:, 0], Sensors_Data[:, 1], "ro", label="Sensors Data")
plt.plot(X[0, :], X[1, :], label="State")
plt.plot(X[0, :], X[1, :], label="State")
plt.plot(X[5, :], X[6, :], "bo", label="Cone")
plt.legend(["Sensors Data", "State", "Cone"])
plt.grid()
plt.axis("equal")
plt.show()

