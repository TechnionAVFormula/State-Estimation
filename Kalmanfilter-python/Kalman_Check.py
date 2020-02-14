##Kalman check
import math as ma
import numpy as np

Acceleration = 1


def Check_Line():
    t = np.arange(0, 10, 0.01)
    X = (t ** 2) / 2 * Acceleration
    y = np.zeros([1, len(X)])
    Steering = y
    Acceleration_Vec = np.ones([1, len(X)])
    u = np.concatenate((Acceleration_Vec, Steering), axis=0)
    Sensors_data = np.empty([len(X), 5])
    for i in range(0, len(Sensors_data)):
        Sensors_data[i, :] = [
            X[i] + np.random.rand(1) * 0.5 - 0.25,
            y[0, i] + np.random.rand(1) * 0.5 - 0.25,
            Acceleration + np.random.rand(1) * 0.1 - 0.05,
            np.random.rand(1) * 0.1 - 0.05,
            np.random.rand(1) * 0.1 - 0.05,
        ]
    return (Sensors_data, X, y, t, u)


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
            X[0, i] + np.random.rand(1) * 0.5 - 0.25,
            y[0, i] + np.random.rand(1) * 0.5 - 0.25,
            np.random.rand(1) * 0.1 - 0.05,
            R / 100 + np.random.rand(1) * 0.1 - 0.05,
            1 / 10 + np.random.rand(1) * 0.01 - 0.005,
        ]
    return (Sensors_data, X, y, t, u)


Check_Circle()
