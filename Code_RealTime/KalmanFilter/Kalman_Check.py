##Kalman check
import math as ma
import numpy as np
from numpy.random import randn

Acceleration = 1


def Check_Line():
    Land_Mark_pos = np.array([[25], [5]])
    Land_Mark_pos2 = np.array([[10], [-5]])
    t = np.arange(0, 10, 0.01)
    X = (t ** 2) / 2 * Acceleration
    y = np.zeros([len(X)])
    Steering = np.zeros([1, len(X)])
    Acceleration_Vec = np.ones([1, len(X)])
    u = np.concatenate((Acceleration_Vec, Steering), axis=0)
    Sensors_data = np.empty([len(X), 5])
    Sensors_data[0, :] = np.array(
        [X[0], y[0], Acceleration + Acceleration * randn(1) * 0.25, 0, 0,]
    )
    for i in range(1, len(Sensors_data)):
        Sensors_data[i, :] = np.array(
            [
                X[i] + randn(1) * 0.25,
                y[i] + randn(1) * 0.25,
                Acceleration + Acceleration * randn(1) * 0.25,
                randn(1) * 0.25,
                randn(1) * 0.25,
            ],
            dtype="float",
        ).reshape(5,)
    External_sensors = np.zeros([2 * len(X), 2])
    for i in range(0, len(Sensors_data), 2):
        External_sensors[i, :] = np.array(
            [
                ma.sqrt(
                    ((X[i] - Land_Mark_pos[0]) ** 2) + ((y[i] - Land_Mark_pos[1]) ** 2)
                )
                + ma.sqrt(
                    ((X[i] - Land_Mark_pos[0]) ** 2) + ((y[i] - Land_Mark_pos[1]) ** 2)
                )
                * randn(1)
                * 0.025,
                ma.atan2(Land_Mark_pos[1] - y[i], Land_Mark_pos[0] - X[i])
                + ma.atan2(Land_Mark_pos[1] - y[i], Land_Mark_pos[0] - X[i])
                * randn(1)
                * 0.0025,
            ],
            dtype="float",
        ).reshape(2,)
        External_sensors[i + 1, :] = np.array(
            [
                ma.sqrt(
                    ((X[i] - Land_Mark_pos2[0]) ** 2)
                    + ((y[i] - Land_Mark_pos2[1]) ** 2)
                )
                + ma.sqrt(
                    ((X[i] - Land_Mark_pos2[0]) ** 2)
                    + ((y[i] - Land_Mark_pos2[1]) ** 2)
                )
                * randn(1)
                * 0.025,
                ma.atan2(Land_Mark_pos2[1] - y[i], Land_Mark_pos2[0] - X[i])
                + ma.atan2(Land_Mark_pos2[1] - y[i], Land_Mark_pos2[0] - X[i])
                * randn(1)
                * 0.0025,
            ],
            dtype="float",
        ).reshape(2,)
    return (Sensors_data, External_sensors, X, y, t, u)


def Check_Circle():
    t = np.arange(0, 20 * ma.pi, 0.01)
    Steering = np.empty([1, len(t)])
    R = 100
    X = np.empty([1, len(t)])
    y = np.empty([1, len(t)])
    X[0, 0] = R
    y[0, 0] = 0
    for i in range(0, len(t)):
        X[0, i] = R * ma.cos(t[i] / 10)
        y[0, i] = R * ma.sin(t[i] / 10)
    Steering[0, :] = 0.01 / 10

    Acceleration_Vec = np.ones([1, len(t)])
    u = np.concatenate((Acceleration_Vec, Steering), axis=0)
    Sensors_data = np.empty([len(t), 5])
    for i in range(0, len(Sensors_data)):
        Sensors_data[i, :] = [
            X[0, i] + np.random.rand(1) * 3 - 1.5,
            y[0, i] + np.random.rand(1) * 3 - 1.5,
            np.random.rand(1) * 0.1 - 0.5,
            R / 100 + np.random.rand(1) * 0.1 - 0.5,
            1 / 10 + np.random.rand(1) * 0.01 - 0.005,
        ]
    return (Sensors_data, X, y, t, u)


def Check_path(time, R, Theta, GPStimedelta, Perceptiontimedelta):
    x = R * ma.cos(time / 2 / R)
    y = R * ma.sin(time / R)
    xd = -1 / 2 * ma.sin(time / 2 / R)
    yd = 1 * ma.cos(time / R)
    xdd = -1 / 4 / R * ma.cos(time / 2 / R)
    ydd = -1 / 4 / R * ma.sin(time / R)
    ImuData = np.array(
        [
            (xdd * ma.cos(Theta) + ydd * ma.sin(Theta)) * (1 + randn(1) * 0.1 - 0.05),
            (xdd * ma.sin(Theta) - ydd * ma.cos(Theta)) * (1 + randn(1) * 0.1 - 0.05),
            (ma.atan2(yd, xd)) * (1 + randn(1) * 0.1 - 0.05),
        ]
    ).reshape([3, 1])
    if not time % GPStimedelta:
        GPSdata = np.array([x + randn(1) * 1 - 0.5, y + randn(1) * 1 - 0.5])
    else:
        GPSdata = np.array([])
    if not time % Perceptiontimedelta:
        t = np.arange(0, 4 * ma.pi, ma.pi / 5)
        TrajCones = np.zeros([2, 2 * len(t)])
        j = len(t)
        for i in range(len(t)):
            TrajCones[0][i] = (R - 3) * ma.cos(t[i] / 2)
            TrajCones[1][i] = (R - 3) * ma.sin(t[i])
            TrajCones[0][j] = (R + 3) * ma.cos(t[i] / 2)
            TrajCones[1][j] = (R + 3) * ma.sin(t[i])
            j = j + 1
        Cones = []
        for i in range(2 * len(t)):
            r = ma.sqrt((TrajCones[0, i] - x) ** 2 + (TrajCones[1, i] - y) ** 2)
            T = ma.atan2(TrajCones[1, i] - y, TrajCones[0, i] - x) - Theta
            if r < 30:
                Cones.append([r, T])
        Cones = np.array(Cones, dtype="float")
    else:
        Cones = np.array([])
    Steering = ma.atan2(y, x) - Theta
    Acceleration = (xdd * ma.cos(Theta) + ydd * ma.sin(Theta)) * (
        1 + randn(1) * 0.2 - 0.1
    )
    Control = np.array([Acceleration, Steering])
    return (ImuData, GPSdata, Cones, Control, x, y)
