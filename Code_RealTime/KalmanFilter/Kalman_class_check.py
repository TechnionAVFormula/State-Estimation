from Kalman_class import Kalman
from Kalman_Check import Check_Circle
import numpy as np
import math as ma
import matplotlib.pyplot as plt

Sensors_Data, X_Ground_Truth, Y_Ground_Truth, Time, u = Check_Circle()

K = Kalman()
K.State_Correction = np.zeros([5, 1])
K.State_Correction[0] = 100
K.State_Correction[4] = ma.pi / 2
K.Covariance_Update = 0.01 * np.eye(len(K.State_Correction))
K.Measure_Noise = np.diag([0.1 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2, 0.1 ** 2])
K.Motion_Noise = np.diag([0.1 ** 2, 0.01 ** 2])
X = np.empty([5, len(Sensors_Data)])
K.State_Prediction_function(u[:, 0])
for i in range(len(Sensors_Data)):
    K.State_Update_function(Sensors_Data[i, :])
    X[:, i : i + 1] = K.State_Correction
    K.State_Prediction_function(u[:, i])

plt.figure(1)
plt.plot(Sensors_Data[:, 0], Sensors_Data[:, 1], "ro", label="Sensors Data")
plt.plot(X[0, :], X[1, :], label="State")
plt.plot(X_Ground_Truth[0, :], Y_Ground_Truth[0, :], label="Ground Truth")
plt.legend(["Ground Truth", "State", "Sensors Data"])
plt.grid()
plt.axis("equal")
plt.grid()
plt.figure(2)
plt.plot(Sensors_Data[:, 0], Sensors_Data[:, 1], "ro", label="Sensors Data")
plt.plot(X[0, :], X[1, :], label="State")
plt.legend(["Sensors Data", "State"])
plt.grid()
plt.axis("equal")
plt.show()
