# EKF-SLAM-Class-check
from EKF_Slam_Class import Kalman
from Kalman_Check import Check_path
import numpy as np
import math as ma
import matplotlib.pyplot as plt
from celluloid import Camera
import matplotlib.animation as animation

plt.style.use("seaborn-pastel")
Xtraj = []
Ytraj = []
Xreal = []
Yreal = []
A = np.array([[0], [0], [0]])
GPS = np.array([[0], [0]])
K = Kalman()
<<<<<<< HEAD
K.State_Correction = np.array([50, 0, 0, 10, ma.pi / 2]).reshape([5, 1])
K.Covariance_Update = 0.1 * np.eye(len(K.State_Correction))
K.Motion_Noise = np.diag([0.025, 0 ** 2])
K.Measure_Acc_Noise = np.diag([0.000025, 0.000025, 0.05])
K.Measure_GPS_Noise = np.diag([0.00025, 0.00025])
K.External_Measure_Noise = np.diag([5, 0.1])
=======

>>>>>>> cf00a7a3b116f7aae59320712a1100edb6d93d26

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

x = 0
y = 0
for i in range(1, 6500):
    (
        K.Measure_Accelerometer,
        K.Measure_GPS,
        K.External_Measure_Update,
        K.Control_Command,
        x,
        y,
    ) = Check_path(K.Time_Delta * i, 50, K.State_Correction[4], 0.5, 10000, x, y)
    K.State_Prediction_function()
    K.State_Update_function()
    Xtraj.append(K.State_Correction[0])
    Ytraj.append(K.State_Correction[1])
    Xreal.append(x)
    Yreal.append(y)
plt.plot(Xtraj, Ytraj, "g", label="State")
plt.plot(Xreal, Yreal, "y", Label="Real")
plt.legend(["State", "Real"])
# if np.size(K.State_Correction) > 5:
#     for j in range(5, len(K.State_Correction), 2):
#         plt.plot(
#             K.State_Correction[j],
#             K.State_Correction[j + 1],
#             "bo",
#             label="Cones Estimation",
#             Lw=4,
#         )

# t = np.arange(0, 4 * ma.pi, ma.pi / 4)
# TrajCones = np.zeros([2, 2 * len(t)])
# j = len(t)
# R = 50
# for i in range(len(t)):
#     TrajCones[0][i] = (R - 5) * ma.cos(t[i] / 2)
#     TrajCones[1][i] = (R - 5) * ma.sin(t[i])
#     TrajCones[0][j] = (R + 5) * ma.cos(t[i] / 2)
#     TrajCones[1][j] = (R + 5) * ma.sin(t[i])
#     j = j + 1
# plt.plot(TrajCones[0, :], TrajCones[1, :], "ko", label="RealCones", lw=2)
#
plt.show()
# plt.plot(State_data[0, :], State_data[1, :], "g--", label="State", lw=2)
# plt.plot(State_data[5, 1:], State_data[6, 1:], "go", label="Cone 1")
# plt.plot(10, -5, "k+", label="Cone 1 Real")
# plt.plot(State_data[7, 100:], State_data[8, 100:], "ro", label="Cone 2")
# plt.plot(25, 5, "k+", label="Cone 2 Real")

# plt.plot(
#     Sensors_Data[range(0, 1000, 50), 0],
#     Sensors_Data[range(0, 1000, 50), 1],
#     "b+",
#     label="GPS",
# )
# t = np.arange(0, 10, 0.01)
# X = (t ** 2) / 2
# Y = np.zeros([len(X)])
# plt.plot(X, Y, label="Ground Truth", lw=3)
# plt.xlabel("X")
# plt.ylabel("Y")
# plt.title("EKF-SLAM")
# plt.legend(
#     ["State", "Cone 1", "Cone 1 Real", "Cone 2", "Cone 2 Real", "GPS", "Ground Truth",]
# )
# plt.grid()
# plt.show()
# plt.show()
