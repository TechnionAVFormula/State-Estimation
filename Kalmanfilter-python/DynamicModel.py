##dynamic model for kalman filter
import math
import numpy as np
from DynamicModel import SteeringTranslate, GPSTranslate

##run initial data and then start the loop.
L_Tot = 1.535  # units [m], from Calculations,Weight Transfer file.
L_Rear = 0.7675  # units [m],assuming the weight is distributed equal on the rear and front wheels need
# need to be check.
# control static Magnitude
Px = 1
Py = 1
#
Vd = 1  # need to take from the track algorithm.
Steering_angle = SteeringTranslate()
# slip angle.
beta_before = L_Rear * np.tan(Steering_angle) / L_Tot

# control efforts.
ddx = Px * np.norm(Vd - V_before) * np.cos(Theta_before + beta_before)
ddy = Py * np.norm(Vd - V_before) * np.sin(Theta_before + beta_before)
dTheta = PT * np.arctan2(Yd - y_before, Xd - x_befoSre)

# state space.
x = x_before + dTime * dx_before + np.power(dTime, 2) * ddx / 2
y = y_before + dTime * dy_before + np.power(dTime, 2) * ddy / 2
dx = dx_before + dTime * ddx
dy = dy_before + dTime * ddy
Theta = Theta_before + dTime * dTheta

