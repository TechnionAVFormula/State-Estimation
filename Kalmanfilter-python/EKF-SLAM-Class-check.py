# EKF-SLAM-Class-check
from EKF_Slam_Class import Kalman
from Kalman_Check import Check_Line
import numpy as np
import math as ma
import matplotlib.pyplot as plt

Sensors_Data, External_Sensors, X_Ground_Truth, Y_Ground_Truth, Time, u = Check_Line()
A = np.array([[0], [0], [0]])
num = 0
GPS = np.array([[0], [0]])
K = Kalman(GPS, A, num)
K.State_Correction = np.zeros([5, 1])
K.State_Correction[0] = 0
K.State_Correction[4] = 0
K.Covariance_Update = 0.01 * np.eye(len(K.State_Correction))
K.Measure_Noise = np.diag([0.1 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2])
K.Motion_Noise = np.diag([0.1 ** 2, 0.01 ** 2])
K.Measure_Acc_Noise = np.diag([0.1 ** 2, 0.1 ** 2, 0.1 ** 2])
K.Measure_GPS_Noise = np.diag([0.1 ** 2, 0.1 ** 2])
K.Control_Command = u[:, 0]
K.State_Prediction_function()
K.State_Update_function()
K.Measure_GPS = np.reshape(Sensors_Data[1, :2], (2, 1))
K.Measure_Accelerometer = np.reshape(Sensors_Data[1, 2:], (3, 1))
K.Control_Command = u[:, 1]
K.State_Prediction_function()
K.State_Update_function()
print(K.State_Correction)
K.Measure_GPS = np.reshape(Sensors_Data[2, :2], (2, 1))
K.Measure_Accelerometer = np.reshape(Sensors_Data[2, 2:], (3, 1))
K.Control_Command = u[:, 2]
K.State_Prediction_function()
K.State_Update_function()
K.Measure_GPS = np.reshape(Sensors_Data[3, :2], (2, 1))
K.Measure_Accelerometer = np.reshape(Sensors_Data[3, 2:], (3, 1))
K.Control_Command = u[:, 3]
K.State_Prediction_function()
K.External_Measure_Update = External_Sensors[0:2, :]
K.Number_of_Cones = 1
K.External_Measure_Noise = np.diag([0.5 ** 2, 0.1 ** 2])
K.State_Update_function()
print(K.State_Correction)
K.Control_Command = u[:, 4]
K.State_Prediction_function()
K.Measure_GPS = np.array([])
K.Measure_Accelerometer = np.reshape(Sensors_Data[4, 2:], (3, 1))
K.External_Measure_Update = External_Sensors[2:4, :]
print(External_Sensors[2:4, :])
K.State_Update_function()
print(K.State_Correction)
# # K.Control_Command = u[:, 5]
# # K.State_Prediction_function()
# # K.Measure_GPS = np.reshape(Sensors_Data[5, :2], (2, 1))
# # K.Measure_Accelerometer = np.reshape(Sensors_Data[5, 2:], (3, 1))
# # K.External_Measure_Update = External_Sensors[2:3, :]
# # K.State_Update_function()
# # print(K.State_Correction)
