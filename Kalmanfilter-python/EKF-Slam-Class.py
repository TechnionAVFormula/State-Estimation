import numpy as np
import math as ma
from numpy.random import randn


class Kalman:
    def __init__(self,):
        self.Time_Delta = 0.01
        self.Vehicle_Rear_Length = 0.7675
        self.Vehicle_Total_Length = 1.535
        self.Number_of_Cones = []
        self.Slip_angle = []
        self.Motion_Noise = []
        self.Measure_Noise = []
        self.State_Prediction = []
        self.Covariance_Prediction = []
        self.Measure_GPS = []
        self.Measure_Accelerometer = []
        self.External_Measure_Update = []
        self.State_Correction = []
        self.State_Update = []
        self.Covariance_Update = []

    @property
    def Measure_GPS(self):
        return self.__Measure_GPS

    @property
    def Measure_Accelerometer(self):
        return self.__Measure_Accelerometer

    @Measure_GPS.setter
    def Measure_GPS(self, GPS):
        if self.Measure_GPS == GPS:
            self.__Measure_GPS = []
        else:
            self.__Measure_GPS = GPS

    @Measure_Accelerometer.setter
    def Measure_Accelerometer(self, Accelerometer):
        if self.Measure_Accelerometer == Accelerometer:
            self.__Measure_Accelerometer = []
        else:
            self.__Measure_Accelerometer = Accelerometer

    def State_Prediction_function(self, u):
        F = np.eye(5)
        if self.Number_of_Cones > 0:
            F = np.append(F, np.zeros((F.shape[0], 2 * self.Number_of_Cones)), 1)
        self.Slip_angle = np.mod(
            ma.atan2(
                ma.tan(u[1]) * self.Vehicle_Rear_Length, self.Vehicle_Total_Length,
            ),
            2 * ma.pi,
        )
        V_tot = ma.sqrt(
            (
                (
                    self.State_Correction[2]
                    + self.Time_Delta
                    * ma.cos(self.State_Correction[4] + self.Slip_angle)
                    * u[0]
                )
                ** 2
            )
            + (
                (
                    self.State_Correction[3]
                    + self.Time_Delta
                    * ma.sin(self.State_Correction[4] + self.Slip_angle)
                    * u[0]
                )
                ** 2
            )
        )
        DTheta = (
            V_tot * ma.cos(self.Slip_angle) * ma.tan(u[1]) / self.Vehicle_Total_Length
        )
        Movement = np.array(
            [
                [
                    self.Time_Delta * self.State_Prediction[2]
                    + (self.Time_Delta ** 2)
                    / 2
                    * ma.cos(self.State_Prediction[4] + self.Slip_angle)
                    * u[0]
                ],
                [
                    self.Time_Delta * self.State_Prediction[3]
                    + (self.Time_Delta ** 2)
                    / 2
                    * ma.sin(self.State_Prediction[4] + self.Slip_angle)
                    * u[0]
                ],
                [
                    self.Time_Delta
                    * ma.cos(self.State_Prediction[4] + self.Slip_angle)
                    * u[0]
                ],
                [
                    self.Time_Delta
                    * ma.sin(self.State_Prediction[4] + self.Slip_angle)
                    * u[0]
                ],
                [self.Time_Delta * DTheta],
            ],
            dtype="float",
        )
        self.State_Prediction = self.State_Prediction + Movement
        Partial_Jacobian = np.array(
            [
                [
                    0,
                    0,
                    self.Time_Delta,
                    0,
                    -(self.Time_Delta ** 2)
                    * u[0]
                    * ma.sin(self.State_Correction[4] + self.Slip_angle),
                ],
                [
                    0,
                    0,
                    0,
                    self.Time_Delta,
                    (self.Time_Delta ** 2)
                    * u[0]
                    * ma.cos(self.State_Correction[4] + self.Slip_angle),
                ],
                [
                    0,
                    0,
                    0,
                    0,
                    -self.Time_Delta
                    * u[0]
                    * ma.sin(self.State_Correction[4] + self.Slip_angle),
                ],
                [
                    0,
                    0,
                    0,
                    0,
                    self.Time_Delta
                    * u[0]
                    * ma.cos(self.State_Correction[4] + self.Slip_angle),
                ],
                [0, 0, 0, 0, 0],
            ]
        )
        Jacobian_State = (
            np.eye((self.State_Correction.shape[0]))
            + np.transpose(F) @ Partial_Jacobian @ F
        )
        Jacobian_Motion_Noise = np.array(
            [
                [
                    (self.Time_Delta ** 2)
                    * ma.cos(self.State_Correction[4] + self.Slip_angle),
                    0,
                ],
                [
                    (self.Time_Delta ** 2)
                    * ma.sin(self.State_Correction[4] + self.Slip_angle),
                    0,
                ],
                [
                    self.Time_Delta
                    * ma.cos(self.State_Correction[4] + self.Slip_angle),
                    0,
                ],
                [
                    self.Time_Delta
                    * ma.sin(self.State_Correction[4] + self.Slip_angle),
                    0,
                ],
                [
                    0,
                    self.Time_Delta
                    * np.linalg.norm(self.State_Correction[2:4])
                    * ma.cos(self.Slip_angle)
                    / (self.Vehicle_Total_Length * (ma.cos(u[1]) ** 2)),
                ],
            ]
        )

        self.Covariance_Prediction = (
            (Jacobian_State @ self.Covariance_Update @ np.transpose(Jacobian_State))
            + np.transpose(F)
            @ (
                Jacobian_Motion_Noise
                @ self.Motion_Noise
                @ np.transpose(Jacobian_Motion_Noise)
            )
            @ F
        )

    def State_Update_function(self, Sensors_Data):
        self.Measure_Update = np.array(
            [
                [Sensors_Data[0]],
                [Sensors_Data[1]],
                [
                    self.State_Prediction[2]
                    + self.Time_Delta
                    * (
                        Sensors_Data[2]
                        * ma.cos(self.State_Prediction[4] + self.Slip_angle)
                        + Sensors_Data[3]
                        * ma.sin(self.State_Prediction[4] + self.Slip_angle)
                    )
                ],
                [
                    self.State_Prediction[3]
                    + self.Time_Delta
                    * (
                        Sensors_Data[2]
                        * ma.sin(self.State_Prediction[4] + self.Slip_angle)
                        - Sensors_Data[3]
                        * ma.cos(self.State_Prediction[4] + self.Slip_angle)
                    )
                ],
                [self.State_Prediction[4] + self.Time_Delta * Sensors_Data[4]],
            ],
            dtype="float",
        )
        Jacobian_Measure_Model = np.array(
            [
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [
                    0,
                    0,
                    1,
                    0,
                    -self.Time_Delta
                    * (
                        Sensors_Data[2]
                        * ma.sin(self.State_Prediction[4] + self.Slip_angle)
                        - Sensors_Data[3]
                        * ma.cos(self.State_Prediction[4] + self.Slip_angle)
                    ),
                ],
                [
                    0,
                    0,
                    0,
                    1,
                    self.Time_Delta
                    * (
                        Sensors_Data[2]
                        * ma.cos(self.State_Prediction[4] + self.Slip_angle)
                        + Sensors_Data[3]
                        * ma.sin(self.State_Prediction[4] + self.Slip_angle)
                    ),
                ],
                [0, 0, 0, 0, 1],
            ],
            dtype="float",
        )

        S = (
            Jacobian_Measure_Model
            @ self.Covariance_Prediction
            @ np.transpose(Jacobian_Measure_Model)
            + self.Measure_Noise
        )
        Kalman_Gain = (
            self.Covariance_Prediction
            @ np.transpose(Jacobian_Measure_Model)
            @ np.linalg.inv(S)
        )
        self.State_Correction = self.State_Prediction + Kalman_Gain @ (
            self.Measure_Update - self.State_Prediction
        )
        self.Covariance_Update = (
            self.Covariance_Prediction
            - Kalman_Gain
            @ np.transpose(Jacobian_Measure_Model)
            @ self.Covariance_Prediction
        )
