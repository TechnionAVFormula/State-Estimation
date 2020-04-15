import numpy as np
import math as ma
from numpy.random import randn
from numpy.linalg import inv, norm

'''
## import depanding on running state / configuration state:
from ..config import CONFIG , ConfigEnum , IS_DEBUG_MODE

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import messages
else:
    raise NameError('User Should Choose Configuration from config.py')
'''


class Kalman(config = 1):
    def __init__(self):
        self.Alpha = 1000
        self.Time_Delta = 0.01
        self.Vehicle_Rear_Length = 0.7675
        self.Vehicle_Total_Length = 1.535
        self.Number_of_Cones = 0
        if (CONFIG  == ConfigEnum.REAL_TIME):
            # Real time covariance.
            pass
        
        elif ( CONFIG == ConfigEnum.LOCAL_TEST) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
            # Test values:
            self.State_Correction = np.zeros([5 , 1])
            self.Covariance_Update = 0.01 * np.eye(len(self.State_Correction))
            self.Motion_Noise = np.diag([3, 0 ** 2])
            self.Measure_Acc_Noise = np.diag([0.00025, 0.0025, 0.25])
            self.Measure_GPS_Noise = np.diag([0.0025, 0.0025])
            self.External_Measure_Noise = np.diag([3, 0.1])
        '''
        function [P,R,Q]  = initial_cov_mats()
              %x    %y      %Vx  %Vy   %Theta
R  =   [ [    2   , 0      ,   0       ,  0         , 0]  ;  ...  
            [     0     ,  2  ,   0       ,  0         , 0]  ; ...
            [     0     ,    0   ,  0.1555     ,  0         , 0]  ; ...
            [     0      ,   0   ,   0       ,  0.1555         , 0]  ; ...
            [     0      ,   0   ,   0       ,  0         , 0.22]   ]  ;
        %measurement noise of x,y are currently sheker ve'cazav because I didn't find
        %the error of the GPS
        %theta measurement noise given in degrees
        %I uesd the error given in the files plus the statistical measurment error
Q  =   [ [    3   , 0      ,   0       ,  0         , 0]  ;  ...  
            [     0     ,  3  ,   0       ,  0         , 0]  ; ...
            [     0     ,    0   ,  0.23     ,  0           , 0]  ; ...
            [     0      ,   0   ,   0       ,  0.23        , 0]  ; ...
            [     0      ,   0   ,   0       ,  0           , 0.33]   ]  ;   
        %same here for x,y. Nir please be proud of me 
        %process noise calculated weighting the filter's update step and dynamic model
        P =  [0.124536262397879              ,1.68586023371090e-07               ,0                                         ,0                                             ,-2.04133926343496e-08 ; ...
                1.68586023371084e-07         ,0.124583808868131                    ,0                                          ,0                                              ,-0.00353120114306259 ;
                 0                                            ,0                                                   ,0.118614066163451           ,0.0186140661634507             ,0                                     ; ...
                 0                                            ,0                                                   ,0.0186140661634507         ,0.118614066163451               ,0                                     ; 
                 -2.04133926343496e-08      ,-0.00353120114306259                ,0                                          ,0                                              ,2.65329138921966        ] ;
 
end
        '''

        self.Slip_angle = np.array([])
        self.Motion_Noise = np.array([])
        self.Measure_Acc_Noise = np.array([])
        self.Measure_GPS_Noise = np.array([])
        self.External_Measure_Noise = np.array([])
        self.State_Prediction = np.array([])
        self.Covariance_Prediction = np.array([])
        self.Measure_GPS = np.array([])
        self.Measure_Accelerometer = np.array([])
        self.External_Measure_Update = np.array([])
        self.State_Correction = np.array([])
        self.State_Update = np.array([])
        self.Covariance_Update = np.array([])
        self.Measure_GPS_Model = np.array([])
        self.Measure_Accelerometer_Model = np.array([])
        self.Control_Command = np.array([])
        self.Addinig_State = np.array([])

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

    @Measure_Accelerometer.setter
    def Measure_Accelerometer(self, Accelerometer):
        self.__Measure_Accelerometer = Accelerometer

    def State_Prediction_function(self):
        F = np.eye(5)
        if self.Number_of_Cones > 0:
            F = np.append(F, np.zeros((F.shape[0], 2 * self.Number_of_Cones)), 1)
        self.Slip_angle = ma.atan2(
            ma.tan(self.Control_Command[1]) * self.Vehicle_Rear_Length,
            self.Vehicle_Total_Length,
        )
        V_tot = norm(self.State_Correction[2:4])
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
        ).reshape([5, 1])

        Movement = F.T @ Movement
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
            ],
            dtype="float",
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
                    norm(self.State_Correction[2:4])
                    * ma.cos(self.Slip_angle)
                    / (
                        self.Vehicle_Total_Length
                        * (ma.cos(self.Control_Command[1]) ** 2)
                    ),
                ],
            ],
            dtype="float",
        )
        self.Covariance_Prediction = (
            (Jacobian_State @ self.Covariance_Update @ Jacobian_State.T)
            + F.T
            @ (Jacobian_Motion_Noise @ self.Motion_Noise @ Jacobian_Motion_Noise.T)
            @ F
        )

    def State_Update_function(self):
        V_T = norm(self.State_Prediction[2:4] + 0.01)
        self.Measure_GPS_Model = np.array(
            [self.State_Prediction[0], self.State_Prediction[1]]
        )
        self.Measure_Accelerometer_Model = np.array(
            [
                [
                    self.Control_Command[0]
                    * ma.cos(self.State_Prediction[4] + self.Slip_angle)
                ],
                [
                    self.Control_Command[0]
                    * ma.sin(self.State_Prediction[4] + self.Slip_angle)
                ],
                [
                    V_T
                    * ma.cos(self.Slip_angle)
                    * ma.tan(self.Control_Command[1])
                    / self.Vehicle_Total_Length
                ],
            ],
            dtype="float",
        ).reshape([3, 1])
        Jacobian_Measure_Accelerometer_Model = np.array(
            [
                [
                    0,
                    0,
                    0,
                    0,
                    -self.Control_Command[0]
                    * ma.sin(self.State_Prediction[4] + self.Slip_angle),
                    0,
                    0,
                ],
                [
                    0,
                    0,
                    0,
                    0,
                    self.Control_Command[0]
                    * ma.cos(self.State_Prediction[4] + self.Slip_angle),
                    0,
                    0,
                ],
                [
                    0,
                    0,
                    self.State_Prediction[2]
                    * ma.cos(self.Slip_angle)
                    * ma.tan(self.Control_Command[1])
                    / (V_T * self.Vehicle_Total_Length),
                    self.State_Prediction[3]
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
            F = np.block([np.eye(7), np.zeros([7, 2 * (self.Number_of_Cones - 1)])],)
        else:
            F = np.block([[np.eye(5)], [np.zeros([2, 5])]])
        Jacobian_Measure_Accelerometer_Model = Jacobian_Measure_Accelerometer_Model @ F
        S_Inv_Acc_Model = inv(
            np.array(
                Jacobian_Measure_Accelerometer_Model
                @ self.Covariance_Prediction
                @ Jacobian_Measure_Accelerometer_Model.T
                + self.Measure_Acc_Noise,
                dtype="float",
            )
        )
        Model_Gain_Acc = (
            self.Covariance_Prediction
            @ np.transpose(Jacobian_Measure_Accelerometer_Model)
            @ S_Inv_Acc_Model
        )
        self.State_Correction = Model_Gain_Acc @ (
            self.Measure_Accelerometer - self.Measure_Accelerometer_Model
        )
        Covariance_Minimal = Model_Gain_Acc @ Jacobian_Measure_Accelerometer_Model
        if np.size(self.Measure_GPS) > 0:
            Jacobian_Measure_Gps_Model = np.array(
                [[1, 0, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0, 0],], dtype="float",
            )
            Jacobian_Measure_Gps_Model = Jacobian_Measure_Gps_Model @ F
            S_Inv_GPS_Model = inv(
                Jacobian_Measure_Gps_Model
                @ self.Covariance_Prediction
                @ Jacobian_Measure_Gps_Model.T
                + self.Measure_GPS_Noise,
            )
            Model_Gain_GPS = (
                self.Covariance_Prediction
                @ np.transpose(Jacobian_Measure_Gps_Model)
                @ S_Inv_GPS_Model
            )
            self.State_Correction = self.State_Correction + Model_Gain_GPS @ (
                self.Measure_GPS - self.Measure_GPS_Model
            )
            Covariance_Minimal = (
                Covariance_Minimal + Model_Gain_GPS @ Jacobian_Measure_Gps_Model
            )
        if np.size(self.External_Measure_Update) > 0:
            if self.Number_of_Cones == 0:
                Covariance_Minimal = self.Cone_Initialization_function(
                    Covariance_Minimal
                )
            for Measure_Update in self.External_Measure_Update:
                Measure_Update = np.reshape(Measure_Update, [2, 1])
                Observation = [
                    self.State_Prediction[0]
                    + Measure_Update[0]
                    * ma.cos(
                        Measure_Update[1] + self.State_Prediction[4] + self.Slip_angle
                    ),
                    self.State_Prediction[1]
                    + Measure_Update[0]
                    * ma.sin(
                        Measure_Update[1] + self.State_Prediction[4] + self.Slip_angle
                    ),
                ]
                Pi_Threshold = 100000
                for K in range(1, 2 * self.Number_of_Cones, 2):
                    Delta = np.array(
                        [
                            self.State_Prediction[4 + K] - self.State_Prediction[0],
                            self.State_Prediction[5 + K] - self.State_Prediction[1],
                        ]
                    )
                    q = np.transpose(Delta) @ Delta
                    Estimate_Measure = np.array(
                        [
                            [ma.sqrt(q)],
                            [ma.atan2(Delta[1], Delta[0]) - self.State_Prediction[4]],
                        ],
                        dtype="float",
                    )
                    F = np.block(
                        [
                            [np.eye(5), np.zeros([5, 2 * (self.Number_of_Cones)])],
                            [
                                np.zeros([2, 4 + K]),
                                np.eye(2),
                                np.zeros([2, 2 * self.Number_of_Cones - K - 1]),
                            ],
                        ],
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
                    S_External = inv(np.array(S_External, dtype="float"))
                    Pi = (
                        np.transpose(Measure_Update - Estimate_Measure)
                        @ S_External
                        @ (Measure_Update - Estimate_Measure)
                    )
                    if Pi < Pi_Threshold:
                        Minimal_Jacobian = External_Jacobian
                        S_Minimal = S_External
                        Pi_Threshold = Pi
                if Pi_Threshold < self.Alpha:
                    Kalman_Gain = (
                        self.Covariance_Prediction
                        @ np.transpose(Minimal_Jacobian)
                        @ S_Minimal
                    )
                    Covariance_Minimal = (
                        Covariance_Minimal + Kalman_Gain @ Minimal_Jacobian
                    )
                    self.State_Correction = self.State_Correction + Kalman_Gain @ (
                        Measure_Update - Estimate_Measure
                    )
                else:
                    self.State_Prediction = np.concatenate(
                        [self.State_Prediction, Observation]
                    )
                    self.State_Correction = np.concatenate(
                        [self.State_Correction, np.zeros([2, 1])]
                    )
                    self.Number_of_Cones = self.Number_of_Cones + 1
                    self.Covariance_Prediction = np.block(
                        [
                            [
                                self.Covariance_Prediction,
                                np.zeros([self.Covariance_Prediction.shape[0], 2]),
                            ],
                            [
                                np.zeros([2, self.Covariance_Prediction.shape[1]]),
                                0.01 * np.eye(2),
                            ],
                        ]
                    )
                    Covariance_Minimal = np.block(
                        [
                            [
                                Covariance_Minimal,
                                np.zeros([Covariance_Minimal.shape[0], 2]),
                            ],
                            [
                                np.zeros([2, Covariance_Minimal.shape[1]]),
                                np.zeros([2, 2]),
                            ],
                        ]
                    )
            self.State_Correction = self.State_Prediction + self.State_Correction
            self.Covariance_Update = (
                np.eye(self.State_Correction.shape[0]) - Covariance_Minimal
            ) @ self.Covariance_Prediction
        else:
            self.State_Correction = self.State_Prediction + self.State_Correction
            self.Covariance_Update = (
                np.eye(self.State_Correction.shape[0]) - Covariance_Minimal
            ) @ self.Covariance_Prediction

    def Cone_Initialization_function(self, Covariance_Minimal):
        self.External_Measure_Update = np.array(self.External_Measure_Update)
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
            ],
            dtype="float",
        )
        self.State_Prediction = np.concatenate(
            [self.State_Prediction, Observation.reshape([2, 1])]
        )
        self.State_Correction = np.concatenate(
            [self.State_Correction, np.zeros([2, 1])]
        )
        self.Number_of_Cones = self.Number_of_Cones + 1
        self.Covariance_Prediction = np.block(
            [
                [
                    self.Covariance_Prediction,
                    np.zeros([self.Covariance_Prediction.shape[0], 2]),
                ],
                [np.zeros([2, self.Covariance_Prediction.shape[1]]), 0.01 * np.eye(2),],
            ]
        )
        self.External_Measure_Update = self.External_Measure_Update[1:, :]
        Covariance_Minimal = np.block(
            [
                [Covariance_Minimal, np.zeros([Covariance_Minimal.shape[0], 2]),],
                [np.zeros([2, Covariance_Minimal.shape[1]]), np.zeros([2, 2]),],
            ]
        )
        return Covariance_Minimal
