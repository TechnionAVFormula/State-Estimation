import numpy as np
import math as ma
from numpy.random import randn


class Kalman:
    def __init__(self):
        self.Time_Delta = 0.01
        self.Vehicle_Rear_Length = 0.7675
        self.Vehicle_Total_Length = 1.535
        self.Number_of_Cones = []
        self.Slip_angle = []
        self.Motion_Noise = []
        self.Measure_Noise = []
        self.External_Measure_Noise = []
        self.State_Prediction = []
        self.Covariance_Prediction = []
        self.Measure_GPS = []
        self.Measure_Accelerometer = []
        self.External_Measure_Update = []
        self.State_Correction = []
        self.State_Update = []
        self.Covariance_Update = []
        self.Measure_Model = []
        self.Control_Command = []

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

    def State_Prediction_function(self):
        F = np.eye(5)
        if self.Number_of_Cones > 0:
            F = np.append(F, np.zeros((F.shape[0], 2 * self.Number_of_Cones)), 1)
        self.Slip_angle = np.mod(
            ma.atan2(
                ma.tan(self.Control_Command[1]) * self.Vehicle_Rear_Length,
                self.Vehicle_Total_Length,
            ),
            2 * ma.pi,
        )
        V_tot = ma.sqrt(
            (
                (
                    self.State_Correction[2]
                    + self.Time_Delta
                    * ma.cos(self.State_Correction[4] + self.Slip_angle)
                    * self.Control_Command[0]
                )
                ** 2
            )
            + (
                (
                    self.State_Correction[3]
                    + self.Time_Delta
                    * ma.sin(self.State_Correction[4] + self.Slip_angle)
                    * self.Control_Command[0]
                )
                ** 2
            )
        )
        DTheta = (
            V_tot
            * ma.cos(self.Slip_angle)
            * ma.tan(self.Control_Command[1])
            / self.Vehicle_Total_Length
        )
        Movement = np.array(
            [
                [
                    self.Time_Delta * self.State_Prediction[2]
                    + (self.Time_Delta ** 2)
                    / 2
                    * ma.cos(self.State_Prediction[4] + self.Slip_angle)
                    * self.Control_Command[0]
                ],
                [
                    self.Time_Delta * self.State_Prediction[3]
                    + (self.Time_Delta ** 2)
                    / 2
                    * ma.sin(self.State_Prediction[4] + self.Slip_angle)
                    * self.Control_Command[0]
                ],
                [
                    self.Time_Delta
                    * ma.cos(self.State_Prediction[4] + self.Slip_angle)
                    * self.Control_Command[0]
                ],
                [
                    self.Time_Delta
                    * ma.sin(self.State_Prediction[4] + self.Slip_angle)
                    * self.Control_Command[0]
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
                    * self.Control_Command[0]
                    * ma.sin(self.State_Correction[4] + self.Slip_angle),
                ],
                [
                    0,
                    0,
                    0,
                    self.Time_Delta,
                    (self.Time_Delta ** 2)
                    * self.Control_Command[0]
                    * ma.cos(self.State_Correction[4] + self.Slip_angle),
                ],
                [
                    0,
                    0,
                    0,
                    0,
                    -self.Time_Delta
                    * self.Control_Command[0]
                    * ma.sin(self.State_Correction[4] + self.Slip_angle),
                ],
                [
                    0,
                    0,
                    0,
                    0,
                    self.Time_Delta
                    * self.Control_Command[0]
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
                    / (
                        self.Vehicle_Total_Length
                        * (ma.cos(self.Control_Command[1]) ** 2)
                    ),
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

    def State_Update_function(self):
        V_T = ma.sqrt(
            (self.State_Prediction[2] ** 2) + (self.State_Prediction[3] ** 2) + 0.01
        )  # 0.01 to avoid zero division
        if self.Measure_GPS:
            self.Measure_Model = np.array(
                [
                    [self.State_Prediction[0]],
                    [self.State_Prediction[1]],
                    [
                        self.Control_Command[0]
                        * ma.cos(self.State_Prediction[4] + self.Control_Command[1])
                    ],
                    [
                        self.Control_Command[0]
                        * ma.sin(self.State_Prediction[4] + self.Control_Command[1])
                    ],
                    [
                        V_T
                        * ma.cos(self.Slip_angle)
                        * ma.tan(self.Control_Command[1])
                        / self.Vehicle_Total_Length
                    ],
                ],
                dtype="float",
            )
            Jacobian_Measure_Model = np.array(
                [
                    [1, 0, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0, 0],
                    [
                        0,
                        0,
                        -self.Control_Command[0]
                        * ma.sin(self.State_Prediction[4] + self.Control_Command[1]),
                        0,
                        0,
                        0,
                        0,
                    ],
                    [
                        0,
                        0,
                        self.Control_Command[0]
                        * ma.cos(self.State_Prediction[4] + self.Control_Command[1]),
                        0,
                        0,
                        0,
                        0,
                    ],
                    [
                        0,
                        0,
                        self.State_Prediction[3]
                        * ma.cos(self.Slip_angle)
                        * ma.tan(self.Control_Command[1])
                        / (V_T * self.Vehicle_Total_Length),
                        self.State_Prediction[4]
                        * ma.cos(self.Slip_angle)
                        * ma.tan(self.Control_Command[1])
                        / (V_T * self.Vehicle_Total_Length),
                        0,
                        0,
                        0,
                    ],
                ],
                dtype="float",
            )
        else:
            self.Measure_Model = np.array(
                [
                    [
                        self.Control_Command[0]
                        * ma.cos(self.State_Prediction[4] + self.Control_Command[1])
                    ],
                    [
                        self.Control_Command[0]
                        * ma.sin(self.State_Prediction[4] + self.Control_Command[1])
                    ],
                    [
                        V_T
                        * ma.cos(self.Slip_angle)
                        * ma.tan(self.Control_Command[1])
                        / self.Vehicle_Total_Length
                    ],
                ],
                dtype="float",
            )
            Jacobian_Measure_Model = np.array(
                [
                    [
                        0,
                        0,
                        -self.Control_Command[0]
                        * ma.sin(self.State_Prediction[4] + self.Control_Command[1]),
                        0,
                        0,
                        0,
                        0,
                    ],
                    [
                        0,
                        0,
                        self.Control_Command[0]
                        * ma.cos(self.State_Prediction[4] + self.Control_Command[1]),
                        0,
                        0,
                        0,
                        0,
                    ],
                    [
                        0,
                        0,
                        self.State_Prediction[3]
                        * ma.cos(self.Slip_angle)
                        * ma.tan(self.Control_Command[1])
                        / (V_T * self.Vehicle_Total_Length),
                        self.State_Prediction[4]
                        * ma.cos(self.Slip_angle)
                        * ma.tan(self.Control_Command[1])
                        / (V_T * self.Vehicle_Total_Length),
                        0,
                        0,
                        0,
                    ],
                ],
                dtype="float",
            )
            if self.Number_of_Cones:
                F = np.block([np.eye(7), np.zeros([7, self.Number_of_Cones - 1])])
            else:
                F = np.block([[np.eye(5)], [np.zeros([2, 5])]])
            Jacobian_Measure_Model = Jacobian_Measure_Model @ F
            S_Inv_Model = np.linalg.inv(
                Jacobian_Measure_Model
                @ self.Covariance_Prediction
                @ np.transpose(Jacobian_Measure_Model)
                + self.Measure_Noise
            )
            Model_Gain = (
                self.Covariance_Prediction
                @ np.transpose(Jacobian_Measure_Model)
                @ S_Inv_Model
            )
            ###
            # need to switch Measure_Noise  between two states

        for Measure_Update in self.External_Measure_Update:
            Observation = np.array(
                [
                    [
                        self.State_Prediction[0]
                        + Measure_Update[0]
                        * ma.cos(
                            Measure_Update[1]
                            + self.State_Prediction[4]
                            + self.Slip_angle
                        )
                    ],
                    [
                        self.State_Prediction[1]
                        + Measure_Update[0]
                        * ma.sin(
                            Measure_Update[1]
                            + self.State_Prediction[4]
                            + self.Slip_angle
                        )
                    ],
                ]
            )
            for K in range(1, self.Number_of_Cones, 2):
                Delta = np.array(
                    [
                        [self.State_Prediction[4 + K] - self.State_Prediction[0]],
                        [self.State_Prediction[4 + (K + 1)] - self.State_Prediction[1]],
                    ]
                )
                q = np.transpose(Delta) @ Delta
                Estimate_Measure = np.array(
                    [
                        [ma.sqrt(q)],
                        [ma.atan2(Delta[1], Delta[0]) - self.State_Prediction[4]],
                    ]
                )
                F
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
