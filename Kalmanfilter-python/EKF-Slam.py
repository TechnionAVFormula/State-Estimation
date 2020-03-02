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

P_init = 0.01 * np.eye(len(X_init))
W = np.diag((0.01 ** 2, 0.01 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2))
V = np.diag([0.5 ** 2, 0.1 ** 2])
Estimation_Structure = np.zeros((1, 3))


def Extend_State_Covariance(X, P, External_sensors, Estimation_Structure):
    if (
        ma.sqrt(
            (External_sensors[0] - Estimation_Structure[0])
            + (External_sensors[1] - Estimation_Structure[1])
        )
        > 1
    ):
        C = np.array(
            (
                X[0] + External_sensors[0] * ma.cos(External_sensors[1]),
                X[1] + External_sensors[0] * ma.sin(External_sensors[1]),
            )
        )
        X = np.append(X, C)
        P = np.append(P, 10000 * np.ones((P.shape[0], 2)), 1)

        P = np.append(P, 10000 * np.ones((2, P.shape[1])), 0)

    return X, P, Estimation_Structure


def Prediction(X, P, u):
    F = np.eye(5)
    F = np.append(F, np.zeros((F.shape[0], (X.shape[0] - 5))), 1)
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
    Jacobian_Of_Control = np.append(
        Jacobian_Of_Control, np.zeros((X.shape[0] - 5, u.shape[0])), 0
    )
    P_Prediction = (Jacobian_Of_Model @ P @ np.transpose(Jacobian_Of_Model)) + (
        Jacobian_Of_Control @ V @ np.transpose(Jacobian_Of_Control)
    )
    return P_Prediction, X_Prediction, Beta


def Correction(Sensors_Data, External_sensors, X_Prediction, P_Prediction, Beta):
    delta = np.array(
        [[X_Prediction[5] - X_Prediction[0]], [X_Prediction[6] - X_Prediction[1]]]
    )
    q = np.transpose(delta) @ delta
    Approximated_Z = np.array(
        [
            ma.sqrt(q),
            np.mod(ma.atan2(delta[1], delta[0]), 2 * ma.pi) - X_Prediction[4],
        ],
        dtype="float",
    )

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
            [External_sensors[0]],
            [External_sensors[1]],
        ],
        dtype="float",
    )
    H = np.array(
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
            [delta[1] / q, -delta[0] / q, 0, 0, -1, -delta[1] / q, delta[0] / q],
        ],
        dtype="float",
    )
    F = np.eye(5)
    F = np.append(F, np.zeros((F.shape[0], (X_Prediction.shape[0] - 5))), 1)
    H = H @ F
    H_Full = np.append(H, Hi, 0)
    Kalman_Gain = (
        P_Prediction
        @ np.transpose(H_Full)
        @ np.linalg.inv(H_Full @ P_Prediction @ np.transpose(H_Full) + W)
    )
    Comperison_Vector = np.append(X_Prediction[0:5], Approximated_Z, 0)
    X = X_Prediction + Kalman_Gain @ (Z.reshape([7,]) - Comperison_Vector)
    P = (np.eye(7) - Kalman_Gain @ H_Full) @ P_Prediction
    return X, P


X = np.empty([7, len(Sensors_Data) - 1])
Xx, P, Estimation_Structure = Extend_State_Covariance(
    X_init, P_init, External_sensors[0][:], Estimation_Structure[0][:]
)
P, Xx, Beta = Prediction(Xx, P, u[:, 0])
j = 1
for i in range(1, len(Sensors_Data)):
    Xx, P = Correction(Sensors_Data[i, :], External_sensors[i, :], Xx, P, Beta)
    X[:, i - 1 : i] = Xx.reshape([7, 1])
    P, Xx, Beta = Prediction(Xx, P, u[:, i])

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
plt.plot(X[5, :], X[6, :], "bo", label="Cone")
plt.legend(["Sensors Data", "State", "Cone"])
plt.grid()
plt.axis("equal")
plt.show()
