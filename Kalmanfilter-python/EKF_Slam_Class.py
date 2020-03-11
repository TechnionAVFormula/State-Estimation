import numpy as np
import math as ma
from numpy.random import randn
from numpy.linalg import inv


class Kalman:
    def __init__(self, GPS, Accelerometer, num):
        self.Alpha = 100
        self.Time_Delta = 0.01
        self.Vehicle_Rear_Length = 0.7675
        self.Vehicle_Total_Length = 1.535
        self.Number_of_Cones = num
        self.Slip_angle = np.array([])
        self.Motion_Noise = np.array([])
        self.Measure_Acc_Noise = np.array([])
        self.Measure_GPS_Noise = np.array([])
        self.External_Measure_Noise = np.array([])
        self.State_Prediction = np.array([])
        self.Covariance_Prediction = np.array([])
        self.Measure_GPS = GPS
        self.Measure_Accelerometer = Accelerometer
        self.External_Measure_Update = np.array([])
        self.State_Correction = np.array([])
        self.State_Update = np.array([])
        self.Covariance_Update = np.array([])
        self.Measure_GPS_Model = np.array([])
        self.Measure_Accelerometer_Model = np.array([])
        self.Control_Command = np.array([])

    @property
    def Measure_GPS(self):
        return self.__Measure_GPS

    @property
    def Measure_Accelerometer(self):
        return self.__Measure_Accelerometer

    @property
    def Number_of_Cones(self):
        return self.__Number_of_Cones

    @Measure_GPS.setter
    def Measure_GPS(self, GPS):
        self.__Measure_GPS = GPS

    @Number_of_Cones.setter
    def Number_of_Cones(self, num):
        self.__Number_of_Cones = num
        if self.__Number_of_Cones == 1:
            Observation = np.array(
                [
                    self.State_Prediction[0]
                    + self.External_Measure_Update[0][0]
                    * ma.cos(
                        self.External_Measure_Update[0][1]
                        + self.State_Prediction[4]
                        + self.Slip_angle
                    ),
                    self.State_Prediction[1]
                    + self.External_Measure_Update[0][0]
                    * ma.sin(
                        self.External_Measure_Update[0][1]
                        + self.State_Prediction[4]
                        + self.Slip_angle
                    ),
                ]
            )
            self.State_Prediction = np.concatenate(
                [self.State_Correction, Observation],
            )
            self.Covariance_Prediction = np.block(
                [
                    [
                        self.Covariance_Prediction,
                        np.zeros([self.Covariance_Prediction.shape[0], 2]),
                    ],
                    [
                        np.zeros([2, self.Covariance_Prediction.shape[0]]),
                        1000 * np.eye(2),
                    ],
                ],
            )

    @Measure_Accelerometer.setter
    def Measure_Accelerometer(self, Accelerometer):
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
                    self.Time_Delta * self.State_Correction[2]
                    + (self.Time_Delta ** 2)
                    / 2
                    * ma.cos(self.State_Correction[4] + self.Slip_angle)
                    * self.Control_Command[0]
                ],
                [
                    self.Time_Delta * self.State_Correction[3]
                    + (self.Time_Delta ** 2)
                    / 2
                    * ma.sin(self.State_Correction[4] + self.Slip_angle)
                    * self.Control_Command[0]
                ],
                [
                    self.Time_Delta
                    * ma.cos(self.State_Correction[4] + self.Slip_angle)
                    * self.Control_Command[0]
                ],
                [
                    self.Time_Delta
                    * ma.sin(self.State_Correction[4] + self.Slip_angle)
                    * self.Control_Command[0]
                ],
                [self.Time_Delta * DTheta],
            ],
            dtype="float",
        )
        Movement = np.transpose(F) @ Movement
        self.State_Prediction = self.State_Correction + Movement
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
            np.eye(self.State_Correction.shape[0])
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
        print((Jacobian_State @ self.Covariance_Update @ np.transpose(Jacobian_State)))
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
        self.Measure_GPS_Model = np.array(
            [self.State_Prediction[0], self.State_Prediction[1]]
        )
        self.Measure_Accelerometer_Model = np.array(
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
        Jacobian_Measure_Gps_Model = np.array(
            [[1, 0, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0, 0],], dtype="float",
        )
        Jacobian_Measure_Accelerometer_Model = np.array(
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
        if self.Number_of_Cones > 0:
            F = np.block([np.eye(7), np.zeros([7, 2 * (self.Number_of_Cones - 1)])])
        else:
            F = np.block([[np.eye(5)], [np.zeros([2, 5])]])
        Jacobian_Measure_Gps_Model = Jacobian_Measure_Gps_Model @ F
        Jacobian_Measure_Accelerometer_Model = Jacobian_Measure_Accelerometer_Model @ F
        S_Inv_Acc_Model = inv(
            Jacobian_Measure_Accelerometer_Model
            @ self.Covariance_Prediction
            @ np.transpose(Jacobian_Measure_Accelerometer_Model)
            + self.Measure_Acc_Noise
        )
        S_Inv_GPS_Model = inv(
            Jacobian_Measure_Gps_Model
            @ self.Covariance_Prediction
            @ np.transpose(Jacobian_Measure_Gps_Model)
            + self.Measure_GPS_Noise
        )
        Model_Gain_Acc = (
            self.Covariance_Prediction
            @ np.transpose(Jacobian_Measure_Accelerometer_Model)
            @ S_Inv_Acc_Model
        )
        Model_Gain_GPS = (
            self.Covariance_Prediction
            @ np.transpose(Jacobian_Measure_Gps_Model)
            @ S_Inv_GPS_Model
        )
        self.State_Correction = Model_Gain_Acc @ (
            self.Measure_Accelerometer - self.Measure_Accelerometer_Model
        )
        if self.Measure_GPS.shape[0] > 0:
            self.State_Correction = self.State_Correction + Model_Gain_GPS @ (
                self.Measure_GPS - self.Measure_GPS_Model
            )
        if self.Number_of_Cones > 0:
            for i in range(0, len(self.External_Measure_Update)):
                Measure_Update = self.External_Measure_Update[i][:]
                Observation = np.array(
                    [
                        self.State_Prediction[0]
                        + Measure_Update[0]
                        * ma.cos(
                            Measure_Update[1]
                            + self.State_Prediction[4]
                            + self.Slip_angle
                        ),
                        self.State_Prediction[1]
                        + Measure_Update[0]
                        * ma.sin(
                            Measure_Update[1]
                            + self.State_Prediction[4]
                            + self.Slip_angle
                        ),
                    ]
                )
                Pi_Threshold = 100000
                for K in range(1, 2 * self.Number_of_Cones, 2):
                    Delta = np.array(
                        [
                            self.State_Prediction[4 + K] - self.State_Prediction[0],
                            self.State_Prediction[4 + (K + 1)]
                            - self.State_Prediction[1],
                        ]
                    )
                    q = np.transpose(Delta) @ Delta
                    Estimate_Measure = np.array(
                        [
                            [ma.sqrt(q)],
                            [ma.atan2(Delta[1], Delta[0]) - self.State_Prediction[4]],
                        ]
                    )
                    F = np.block(
                        [
                            [np.eye(5), np.zeros([5, 2 * (self.Number_of_Cones)])],
                            [
                                np.zeros([2, 4 + K]),
                                np.eye(2),
                                np.zeros([2, 2 * self.Number_of_Cones - K - 1]),
                            ],
                        ]
                    )
                    External_Jacobian = (
                        np.array(
                            [
                                [
                                    -ma.sqrt(q) * Delta[0],
                                    -ma.sqrt(q) * Delta[1],
                                    0,
                                    0,
                                    0,
                                    ma.sqrt(q) * Delta[0],
                                    ma.sqrt(q) * Delta[1],
                                ],
                                [Delta[1], -Delta[0], 0, 0, -q, -Delta[1], Delta[0]],
                            ],
                            dtype="float",
                        )
                        / q
                    )
                    External_Jacobian = External_Jacobian @ F
                    S_External = (
                        External_Jacobian
                        @ self.Covariance_Prediction
                        @ np.transpose(External_Jacobian)
                        + self.External_Measure_Noise
                    )
                    S_External = inv(S_External)
                    Pi = (
                        np.transpose(Observation - Estimate_Measure)
                        @ S_External
                        @ (Observation - Estimate_Measure)
                    )
                    if Pi < Pi_Threshold:
                        Minimal_Jacobian = External_Jacobian
                        S_Minimal = S_External
                        Pi_Threshold = Pi
                        ####Covariance update!!!!!
                if Pi_Threshold < self.Alpha:
                    Kalman_Gain = (
                        self.Covariance_Prediction
                        @ np.transpose(Minimal_Jacobian)
                        @ S_Minimal
                    )
                    self.State_Correction = (
                        self.State_Correction
                        + self.State_Prediction
                        + Kalman_Gain
                        @ (Measure_Update.reshape(2, 1) - Estimate_Measure)
                    )
                    ####Covariance update!!!!!
                    ####Covariance update!!!!!
                else:
                    self.State_Correction = np.concatenate(
                        [self.State_Correction, np.zeros([2, 1])]
                    )
                    self.Number_of_Cones = self.Number_of_Cones + 1
                    self.Covariance_Update = np.block(
                        [
                            [
                                self.Covariance_Update,
                                np.zeros([self.Covariance_Update.shape[0], 2]),
                            ],
                            [
                                np.zeros([2, self.Covariance_Update.shape[1]]),
                                10000 * np.eye(2),
                            ],
                        ]
                    )
        else:
            self.State_Correction = self.State_Prediction + self.State_Correction
            self.Covariance_Update = (
                np.eye(self.State_Correction.shape[0])
                - Model_Gain_GPS @ Jacobian_Measure_Gps_Model
                - Model_Gain_Acc @ Jacobian_Measure_Accelerometer_Model
            ) @ self.Covariance_Prediction

