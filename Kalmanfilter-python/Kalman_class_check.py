from Kalman_class import Kalman
from Kalman_Check import Check_Circle
import numpy as np

Sensors_Data, X_Ground_Truth, Y_Ground_Truth, Time, u = Check_Circle()

K = Kalman()
K.State_Correction = np.zeros([5, 1])
K.Covariance_Update = 0.01 * np.eye(len(K.State_Correction))
K.Measure_Noise = np.diag([0.01 ** 2, 0.01 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2])
K.Motion_Noise = np.diag([0.5 ** 2, 0.1 ** 2])
X = np.empty([5, len(Sensors_Data)])
K.State_Prediction_function(u[:, 0])
for i in range(len(Sensors_Data)):
    K.State_Update_function(Sensors_Data[i, :])
    X[:, i : i + 1] = K.State_Correction
    K.State_Prediction_function(u[:, i])
