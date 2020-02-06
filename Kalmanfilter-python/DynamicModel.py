##dynamic model for kalman filter
import math
import numpy as np
import matplotlib.pyplot as plt

##run initial data and then start the loop.

# slip angle.
beta_before = L_rear * np.tan(Steering_angle) / L_tot

# control efforts.
ddx = Px * np.norm(Vd - V_before) * np.cos(T_before + beta_before)
ddy = Py * np.norm(Vd - V_before) * np.sin(T_before + beta_before)
dTheta = PT * np.arctan2(yd - y_before, xd - x_before)

# state space.
x = x_before + dTime * dx_before + np.power(dTime, 2) * ddx / 2
y = y_before + dTime * dy_before + np.power(dTime, 2) * ddy / 2
dx = dx_before + dTime * ddx
dy = dy_before + dTime * ddy
Theta = Theta_before + dTime * dTheta

