##Kalman check
import math as ma
import numpy as np

Acceleration = 1


def Check_Line():
    Land_Mark_pos = np.array([[13], [-8]])
    Land_Mark_pos2 = np.array([[40], [8]])
    t = np.arange(0, 10, 0.1)
    X = (t ** 2) / 2 * Acceleration
    y = np.zeros([len(X)])
    Steering = np.zeros([1, len(X)])
    Acceleration_Vec = np.ones([1, len(X)])
    u = np.concatenate((Acceleration_Vec, Steering), axis=0)
    Sensors_data = np.empty([len(X), 5])
    for i in range(0, len(Sensors_data)):
        Sensors_data[i, :] = [
            X[i] + np.random.rand(1) * 0.5 - 0.25,
            y[i] + np.random.rand(1) * 0.5 - 0.25,
            Acceleration + np.random.rand(1) * 0.1 - 0.05,
            np.random.rand(1) * 0.1 - 0.05,
            np.random.rand(1) * 0.1 - 0.05,
        ]
    External_sensors = np.zeros([2 * len(X), 2])
    for i in range(0, len(Sensors_data), 2):
        External_sensors[i, :] = [
            ma.sqrt(((X[i] - Land_Mark_pos[0]) ** 2) + ((y[i] - Land_Mark_pos[1]) ** 2))
            + np.random.rand(1) * 0.1
            - 0.05,
            ma.atan2(Land_Mark_pos[1] - y[i], Land_Mark_pos[0] - X[i])
            + np.random.rand(1) * 0.1
            - 0.05,
        ]
        External_sensors[i + 1, :] = [
            ma.sqrt(
                ((X[i] - Land_Mark_pos2[0]) ** 2) + ((y[i] - Land_Mark_pos2[1]) ** 2)
            )
            + np.random.rand(1) * 0.1
            - 0.05,
            ma.atan2(Land_Mark_pos2[1] - y[i], Land_Mark_pos2[0] - X[i])
            + np.random.rand(1) * 0.1
            - 0.05,
        ]
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

