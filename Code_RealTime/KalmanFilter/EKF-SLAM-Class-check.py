# EKF-SLAM-Class-check
from EKF_Slam_Class import Kalman
from Kalman_Check import Check_Line
import numpy as np
import math as ma
import matplotlib.pyplot as plt
from celluloid import Camera

plt.style.use("seaborn-pastel")


State_data = np.zeros((9, 1))
Sensors_Data, External_Sensors, X_Ground_Truth, Y_Ground_Truth, Time, u = Check_Line()
A = np.array([[0], [0], [0]])
num = 0
GPS = np.array([[0], [0]])
K = Kalman(GPS, A, num)
K.State_Correction = np.zeros([5, 1])
K.Covariance_Update = 0.01 * np.eye(len(K.State_Correction))
K.Motion_Noise = np.diag([2, 0 ** 2])
K.Measure_Acc_Noise = np.diag([0.00025, 0.00025, 0.00025])
K.Measure_GPS_Noise = np.diag([0.0025, 0.0025])
K.External_Measure_Noise = np.diag([2, 0.01])
K.Control_Command = u[:, 0]
K.State_Prediction_function()
K.State_Update_function()
K.Measure_GPS = np.reshape(Sensors_Data[1, :2], (2, 1))
K.Measure_Accelerometer = np.reshape(Sensors_Data[3, 2:], (3, 1))
K.Control_Command = u[:, 1]
K.State_Prediction_function()
K.External_Measure_Update = External_Sensors[1:2, :]
K.Number_of_Cones = 1

fig = plt.figure()
camera = Camera(fig)

for i in range(len(Sensors_Data)):
    K.State_Update_function()
    K.Control_Command = u[:, i]
    K.Measure_Accelerometer = np.reshape(Sensors_Data[i, 2:], (3, 1))
    if not i % 50:
        K.Measure_GPS = np.reshape(Sensors_Data[i, :2], (2, 1))
    else:
        K.Measure_GPS = np.array([])
    if not i % 20 and i < 50:
        K.External_Measure_Update = External_Sensors[2 * i + 1 : 2 * i + 2, :]
    if not i % 20 and i > 50 and i < 100:
        K.External_Measure_Update = External_Sensors[2 * i : 2 * i + 2, :]
    elif not i % 20 and i > 100 and i < 250:
        K.External_Measure_Update = External_Sensors[2 * i : 2 * i + 1, :]
    else:
        K.External_Measure_Update = np.array([])
    if K.State_Correction.shape[0] == 5:
        State_data = np.append(
            State_data,
            np.stack(K.State_Correction.reshape((5, 1)), np.zeros((4, 1))),
            axis=1,
        )
    elif K.State_Correction.shape[0] == 7:
        State_data = np.append(
            State_data,
            np.append(K.State_Correction.reshape((7, 1)), np.zeros((2, 1)), axis=0),
            axis=1,
        )
        # plt.plot(State_data[1, :], State_data[2, :], label="State")
        # plt.plot(State_data[5, :], State_data[6, :], label="State")
    elif K.State_Correction.shape[0] == 9:
        State_data = np.append(State_data, K.State_Correction.reshape((9, 1)), axis=1)
        # plt.plot(State_data[1, :], State_data[2, :], label="State")
        # plt.plot(State_data[5, :], State_data[6, :], "go", label="Cone 1")
        # plt.plot(State_data[7, :], State_data[8, :], "ro", label="Cone 2")
    K.State_Prediction_function()
print(K.State_Correction)

# camera.snap()

plt.plot(State_data[0, :], State_data[1, :], "g--", label="State", lw=2)
plt.plot(State_data[5, 1:], State_data[6, 1:], "go", label="Cone 1")
plt.plot(10, -5, "k+", label="Cone 1 Real")
plt.plot(State_data[7, 100:], State_data[8, 100:], "ro", label="Cone 2")
plt.plot(25, 5, "k+", label="Cone 2 Real")

plt.plot(
    Sensors_Data[range(0, 1000, 50), 0],
    Sensors_Data[range(0, 1000, 50), 1],
    "b+",
    label="GPS",
)
t = np.arange(0, 10, 0.01)
X = (t ** 2) / 2
Y = np.zeros([len(X)])
plt.plot(X, Y, label="Ground Truth", lw=3)
plt.xlabel("X")
plt.ylabel("Y")
plt.title("EKF-SLAM")
plt.legend(
    ["State", "Cone 1", "Cone 1 Real", "Cone 2", "Cone 2 Real", "GPS", "Ground Truth",]
)
plt.grid()
plt.show()
plt.show()
