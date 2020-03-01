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
W = np.diag((0.01 ** 2, 0.01 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2))
V = np.diag([0.5 ** 2, 0.1 ** 2])
Q_External = np.diag([0.1 ** 2, 0.1 ** 2])
First_Estimation_Structure = np.zeros((1, 2))


def Extend_State_Covariance(X, P, External_sensors, First_Estimation_Structure):
    if (
        ma.sqrt(
            (External_sensors[0] - First_Estimation_Structure[0])
            + (External_sensors[1] - First_Estimation_Structure[1])
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

    return X, P


def Prediction():
    pass


def Correction():
    pass


X, P = Extend_State_Covariance(
    X_init, P_init, External_sensors[0][:], First_Estimation_Structure[0][:]
)
