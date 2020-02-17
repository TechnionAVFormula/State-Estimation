import numpy as np
import math as ma
from numpy.random import randn


class Kalman:
    def __init__(self):
        self.Data.Time_Delta = []
        self.Data.Vehicle_Rear_Length = []
        self.Data.Vehicle_Total_Length = []
        self.Prediction_State.State_Dynamic = []
        self.Prediction_State.State_Dynamic_Jacobian = []
        self.Prediction_State.Covariance.Prediction = []
        self.Prediction_State.Motion_Noise = []
        self.Prediction_State.Motion_Noise_Jacobian = []
        self.Update_state.Kalman_Gain = []
        self.Update_State.Jacobian = []
        self.Update_State.Measure_Noise = []
        self.Update_State.Measure = []
        self.Update_State.State_Correction = []
        self.Update_State.State_Dynamic = []
        self.Update_State.Covariance = []

    def State_Prediction():
        pass

