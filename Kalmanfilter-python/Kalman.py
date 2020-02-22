import numpy as np
import math as ma
from numpy.random import randn
from Kalman_Check import Check_Line
import matplotlib.pyplot as plt

# data
Sensors_Data, External_sensors, X_Ground_Truth, Y_Ground_Truth, Time, u = Check_Line()

DTime = 0.01
L_Tot = 1.535  # units [m], from Calculations,Weight Transfer file.
L_Rear = 0.7675  # units [m],assuming the weight is distributed equal on the rear and front wheels need

X_init = np.zeros([5, 1])
# X_init[0, 0] = 100
# X_init[3, 0] = 1
# X_init[4, 0] = ma.pi / 2

P_init = 0.01 * np.eye(len(X_init))
W = np.diag([0.01 ** 2, 0.01 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2])
V = np.diag([0.5 ** 2, 0.1 ** 2])
Q_External = np.diag([0.01 ** 2, 0.01 ** 2])


def KF_Prediction(X, P, u):
    Beta = np.mod(ma.atan2(ma.tan(u[1]) * L_Rear, L_Tot), 2 * ma.pi)
    V_tot = ma.sqrt(
        ((X[2] + DTime * ma.cos(X[4] + Beta) * u[0]) ** 2)
        + ((X[3] + DTime * ma.sin(X[4] + Beta) * u[0]) ** 2)
    )
    DTheta = V_tot * ma.cos(Beta) * ma.tan(u[1]) / L_Tot
    X_Prediction = np.array(
        [
            X[0] + DTime * X[2] + (DTime ** 2) / 2 * ma.cos(X[4] + Beta) * u[0],
            X[1] + DTime * X[3] + (DTime ** 2) / 2 * ma.sin(X[4] + Beta) * u[0],
            X[2] + DTime * ma.cos(X[4] + Beta) * u[0],
            X[3] + DTime * ma.sin(X[4] + Beta) * u[0],
            X[4] + DTime * DTheta,
        ]
    )
    J_x = np.array(
        [
            [1, 0, DTime, 0, -(DTime ** 2) * u[0] * ma.sin(X[4] + Beta)],
            [0, 1, 0, DTime, (DTime ** 2) * u[0] * ma.cos(X[4] + Beta)],
            [0, 0, 1, 0, -DTime * u[0] * ma.sin(X[4] + Beta)],
            [0, 0, 0, 1, DTime * u[0] * ma.cos(X[4] + Beta)],
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
                / (L_Tot * (ma.cos(u[1]) ** 2)),
            ],
        ]
    )
    P_Prediction = (J_x @ P @ np.transpose(J_x)) + (J_v @ V @ np.transpose(J_v))
    return P_Prediction, X_Prediction, Beta


def KF_Update(Sensors_Data, X_Prediction, Beta, P_Prediction):
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
        ],
        dtype="float",
    )

    S = Hx @ P_Prediction @ np.transpose(Hx) + W  # Measurement prediction covariance

    K = P_Prediction @ np.transpose(Hx) @ np.linalg.inv(S)  # KalmanGain
    X = X_Prediction + K @ (Z - X_Prediction)  # X update
    P = P_Prediction - K @ np.transpose(Hx) @ P_Prediction  # Covariance update
    return X, P


def EKF_Slam_Prediction(X, P_External, External_sensors):
    delta = np.array([X[3] - X[0], X[4] - X[1]])
    q = np.transpose(delta) @ delta
    Z = np.array([External_sensors[0], External_sensors[1]])
    h = np.array([ma.sqrt(q), np.mod(ma.atan2(delta[1], delta[0]) - X[2], 2 * ma.pi)])
    Fx = np.eye(5)
    H = (
        np.array(
            [
                [
                    (-ma.sqrt(q) * delta[0]),
                    (-ma.sqrt(q) * delta[1]),
                    0,
                    (ma.sqrt(q) * delta[0]),
                    (ma.sqrt(q) * delta[1]),
                ],
                [delta[1], -delta[0], -q, -delta[1], delta[0]],
            ],
            dtype="float",
        )
    ) / q
    Hi = H @ Fx
    Ss = Hi @ P_External @ np.transpose(Hi) + Q_External
    K = P_External @ np.transpose(Hi) @ np.linalg.inv(Ss)
    X_Correction = X.reshape(5,) + K @ (Z - h)
    P_Correction = (np.eye(len(X_Correction)) - K @ Hi) @ P_External
    return X_Correction, P_Correction


def Slam_Adding_State(X, P, External_sensors):
    Observation = np.array(
        [
            (X[0] + External_sensors[0] * ma.cos(X[2] + External_sensors[1])),
            (X[1] + External_sensors[0] * ma.sin(X[2] + External_sensors[1])),
        ]
    )
    X = np.append(X, Observation, axis=0)
    P = np.array(
        [
            [P[0][0], P[0][1], P[0][4]],
            [P[1][0], P[1][1], P[1][4]],
            [P[4][0], P[4][1], P[4][4]],
        ]
    )
    Pp = np.block([[P, np.zeros([3, 2])], [np.zeros([2, 3]), 1000 * np.eye(2)]])
    return X, Pp


def Slam_Update_State(X, P, X_n, P_n):
    P_update = np.array(
        [
            [P[0][0], P[0][1], P[0][4], P_n[0][3], P_n[0][4]],
            [P[1][0], P[1][1], P[1][4], P_n[1][3], P_n[1][4]],
            [P[4][0], P[4][1], P[4][4], P_n[2][3], P_n[2][4]],
            [P_n[3][0], P_n[3][1], P_n[3][2], P_n[3][3], P_n[3][4]],
            [P_n[4][0], P_n[4][1], P_n[4][2], P_n[4][3], P_n[4][4]],
        ]
    )
    X_update = np.array([X[0], X[1], X[4], X_n[3], X_n[4]])
    return P_update, X_update


# P, Xx, Beta = KF_Prediction(X_init, P_init, u[:, 0])
# X, P = KF_Update(Sensors_Data[:, 0], Xx, Beta, P)
# print(X)
# print(X_Ground_Truth)
# P, Xx, Beta = KF_Prediction(X, P, u[:, 1])
# X, P = KF_Update(Sensors_Data[:, 1], Xx, Beta, P)
# print(X)
# print(X_Ground_Truth[1,1])
X = np.empty([5, len(Sensors_Data)])
X_Slam_array = np.empty([5, len(Sensors_Data)])
P, Xx, Beta = KF_Prediction(X_init, P_init, u[:, 0])
X_Slam_C, P_Slam_C = Slam_Adding_State(X_init[[0, 1, 4]], P, External_sensors[0, :])
j = 1
X_Slam_array[:, 0:1] = X_Slam_C.reshape(5, 1)
for i in range(1, len(Sensors_Data)):
    Xx, P = KF_Update(Sensors_Data[i, :], Xx, Beta, P)
    X[:, i : i + 1] = Xx
    P, Xx, Beta = KF_Prediction(Xx, P, u[:, i])
    P_Slam, X_Slam = Slam_Update_State(Xx, P, X_Slam_C, P_Slam_C)
    X_Slam_C, P_Slam_C = EKF_Slam_Prediction(X_Slam, P_Slam, External_sensors[i, :])
    X_Slam_array[:, i : i + 1] = X_Slam_C.reshape(5, 1)


print("run")
plt.figure(1)
plt.plot(X[0, :], X[1, :], label="State")
plt.plot(X_Ground_Truth[:], Y_Ground_Truth[:], label="Ground Truth")
plt.legend(["State", "Ground Truth"])
plt.grid()
plt.axis("equal")
plt.figure(2)
plt.plot(Sensors_Data[:, 0], Sensors_Data[:, 1], "ro", label="Sensors Data")
plt.plot(X[0, :], X[1, :], label="State")
plt.plot(X_Slam_array[3, :], X_Slam_array[4, :], "go", label="Sensors Data")
plt.legend(["Sensors Data", "State"])
plt.grid()
plt.axis("equal")
plt.show()

