import numpy as np
import math as ma
from numpy.random import randn
from numpy.linalg import inv, norm

## For relative import:
import os
import sys
from pathlib import Path

current_folder_str = os.path.dirname(__file__)
current_folder_path = Path(current_folder_str)  # make a Path an object out of the path
relative_folder_path = current_folder_path.parent  # get relative parent
sys.path.append(str(relative_folder_path))  # add to search path for imports

## import depanding on running state / configuration state:
from config import CONFIG, ConfigEnum, IS_DEBUG_MODE

if (CONFIG == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
elif CONFIG == ConfigEnum.LOCAL_TEST:
    from pyFormulaClientNoNvidia import messages
else:
    raise NameError("User Should Choose Configuration from config.py")


class Kalman:
    def __init__(self):
        self._Alpha = 1000
        self._Time_Delta = 0.01  # Shouldn't be known in advance.
        self._Vehicle_Rear_Length = 0.7675
        self._Vehicle_Total_Length = 1.535
        self._Number_of_Cones = 0
        if CONFIG == ConfigEnum.REAL_TIME:
            # Real time covariance.
            self._State_Correction = np.zeros([5, 1])
            self._Measure_GPS_Noise = np.diag([2, 2])
            self._Covariance_Update = 0.01 * np.eye(len(self._State_Correction))
            self._Motion_Noise = np.diag([3, 0 ** 2])
            self._Measure_Acc_Noise = np.diag([0.1555, 0.1555, 0.22])
            self._External_Measure_Noise = np.diag([3, 0.1])
        elif (CONFIG == ConfigEnum.LOCAL_TEST) or (
            CONFIG == ConfigEnum.COGNATA_SIMULATION
        ):
            # Test values:
            self._State_Correction = np.zeros([5, 1])
            self._Measure_GPS_Noise = np.diag([0.0025, 0.0025])
            self._Covariance_Update = 0.01 * np.eye(len(self._State_Correction))
            self._Motion_Noise = np.diag([3, 0 ** 2])
            self._Measure_Acc_Noise = np.diag([0.00025, 0.0025, 0.25])
            self._External_Measure_Noise = np.diag([3, 0.1])
        """
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
        """

        self._Slip_angle = np.array([])
        self._State_Prediction = np.array([])
        self._Covariance_Prediction = np.array([])
        self._Measure_GPS = np.array([])
        self._Measure_Accelerometer = np.array([])
        self._External_Measure_Update = np.array([])
        self._State_Correction = np.array([])
        self._State_Update = np.array([])
        self._Covariance_Update = np.array([])
        self._Measure_GPS_Model = np.array([])
        self._Measure_Accelerometer_Model = np.array([])
        self._Control_Command = np.array([])
        self._Rotational_Speed = []

    # =================================================================
    """Functions that need implementation:   <<Start>>"""

    """Get a list of cones we're sure about. Each cone has an ID that needs to be followed"""

    def Measure_Cones(self, cone_array):
        for cone in cone_array:
            r = cone.r
            theta = cone.theta
            cone_id = cone.cone_id
            print(f" r={r}  theta={theta}  cone_id={cone_id}")

    """Returns an array with all the cones we are following. Each cone as the following fields:
       cone_id
       x    # not for this code
       y    # not for this code
       r
       alpha #angle from car's prespective
       position_deviation
       """

    def Get_Cones(self):
        pass

    def Get_Current_State(self):
        state = messages.state_est.CarState()
        """	state has the following fields, most of them should be inserted:
            Vector2D position = 1;				
            Vector2D position_deviation = 2;
            
            Vector2D velocity = 3;			
            Vector2D velocity_deviation = 4;
            
            double theta = 5;    				
            double theta_deviation = 6;  
            
            double theta_dot = 7;    			
            double theta_dot_deviation = 8;  
                
            double steering_angle = 9;
            double steering_angle_deviation = 10;

        Then the function returns car_state    
        """
        # Example:
        state.position.x = 15
        state.position.y = -40.123
        state.position_deviation.x = 0.53  # that's the variance in x
        state.position_deviation.x = 0.72  # that's the variance in y
        state.velocity.x = 7  # we call it Vx
        state.velocity.y = -3.14159265  # we call it Vy
        state.velocity_deviation.x = 0.53  # that's the variance in x
        state.velocity_deviation.x = 0.72  # that's the variance in y
        return state

    """Do the prediction when called.  delta_t_milisec is the time since last"""

    def State_Prediction(self, data):
        delta_t_milisec = data["delta_t_milisec"]  # time since alst prediction
        delta_t_sec = float(delta_t_milisec) / 1000
        steering_angle = data["steering_angle"]
        acceleration_long = data["acceleration_long"]

        # ==update what needed h`ere:
        self._Time_Delta = delta_t_milisec

        # ==act here::
        self._State_Prediction_function()

    def State_Correction(self, data):
        x = data["x"]
        y = data["y"]

        self._Measure_GPS = np.array([x, y])
        self._Measure_Accelerometer = ()

        # ==update what needed here:

        # ==act here:
        self._State_Update_function()

    """Functions that need implementation:   <<End>>"""
    # =================================================================

    @property
    def _Measure_GPS(self):
        return self.__Measure_GPS

    @property
    def _Measure_Accelerometer(self):
        return self.__Measure_Accelerometer

    @property
    def _Number_of_Cones(self):
        return self.__Number_of_Cones

    @_Measure_GPS.setter
    def _Measure_GPS(self, GPS):
        self.__Measure_GPS = GPS

    @_Number_of_Cones.setter
    def _Number_of_Cones(self, num):
        self.__Number_of_Cones = num

    @_Measure_Accelerometer.setter
    def _Measure_Accelerometer(self, Accelerometer):
        self.__Measure_Accelerometer = Accelerometer

    def _State_Prediction_function(self):
        F = np.eye(5)
        if self._Number_of_Cones > 0:
            F = np.append(F, np.zeros((F.shape[0], 2 * self._Number_of_Cones)), 1)
        self._Slip_angle = ma.atan2(
            ma.tan(self._Control_Command[1]) * self._Vehicle_Rear_Length,
            self._Vehicle_Total_Length,
        )
        V_tot = norm(self._State_Correction[2:4])
        self._Rotational_Speed = (
            V_tot
            * ma.cos(self._Slip_angle)
            * ma.tan(self._Control_Command[1])
            / self._Vehicle_Total_Length
        )

        Movement = np.array(
            [
                [
                    self._Time_Delta * self._State_Correction[2]
                    + (self._Time_Delta ** 2)
                    / 2
                    * ma.cos(self._State_Correction[4] + self._Slip_angle)
                    * self._Control_Command[0]
                ],
                [
                    self._Time_Delta * self._State_Correction[3]
                    + (self._Time_Delta ** 2)
                    / 2
                    * ma.sin(self._State_Correction[4] + self._Slip_angle)
                    * self._Control_Command[0]
                ],
                [
                    self._Time_Delta
                    * ma.cos(self._State_Correction[4] + self._Slip_angle)
                    * self._Control_Command[0]
                ],
                [
                    self._Time_Delta
                    * ma.sin(self._State_Correction[4] + self._Slip_angle)
                    * self._Control_Command[0]
                ],
                [self._Time_Delta * self._Rotational_Speed],
            ],
            dtype="float",
        ).reshape([5, 1])

        Movement = F.T @ Movement
        self._State_Prediction = self._State_Correction + Movement
        Partial_Jacobian = np.array(
            [
                [
                    0,
                    0,
                    self._Time_Delta,
                    0,
                    -(self._Time_Delta ** 2)
                    * self._Control_Command[0]
                    * ma.sin(self._State_Correction[4] + self._Slip_angle),
                ],
                [
                    0,
                    0,
                    0,
                    self._Time_Delta,
                    (self._Time_Delta ** 2)
                    * self._Control_Command[0]
                    * ma.cos(self._State_Correction[4] + self._Slip_angle),
                ],
                [
                    0,
                    0,
                    0,
                    0,
                    -self._Time_Delta
                    * self._Control_Command[0]
                    * ma.sin(self._State_Correction[4] + self._Slip_angle),
                ],
                [
                    0,
                    0,
                    0,
                    0,
                    self._Time_Delta
                    * self._Control_Command[0]
                    * ma.cos(self._State_Correction[4] + self._Slip_angle),
                ],
                [0, 0, 0, 0, 0],
            ],
            dtype="float",
        )
        Jacobian_State = (
            np.eye(self._State_Correction.shape[0])
            + np.transpose(F) @ Partial_Jacobian @ F
        )
        Jacobian_Motion_Noise = np.array(
            [
                [
                    (self._Time_Delta ** 2)
                    * ma.cos(self._State_Correction[4] + self._Slip_angle),
                    0,
                ],
                [
                    (self._Time_Delta ** 2)
                    * ma.sin(self._State_Correction[4] + self._Slip_angle),
                    0,
                ],
                [
                    self._Time_Delta
                    * ma.cos(self._State_Correction[4] + self._Slip_angle),
                    0,
                ],
                [
                    self._Time_Delta
                    * ma.sin(self._State_Correction[4] + self._Slip_angle),
                    0,
                ],
                [
                    0,
                    norm(self._State_Correction[2:4])
                    * ma.cos(self._Slip_angle)
                    / (
                        self._Vehicle_Total_Length
                        * (ma.cos(self._Control_Command[1]) ** 2)
                    ),
                ],
            ],
            dtype="float",
        )
        self._Covariance_Prediction = (
            (Jacobian_State @ self._Covariance_Update @ Jacobian_State.T)
            + F.T
            @ (Jacobian_Motion_Noise @ self._Motion_Noise @ Jacobian_Motion_Noise.T)
            @ F
        )

    def _State_Update_function(self):
        V_T = norm(self._State_Prediction[2:4] + 0.01)
        self._Measure_GPS_Model = np.array(
            [self._State_Prediction[0], self._State_Prediction[1]]
        )
        self._Measure_Accelerometer_Model = np.array(
            [
                [
                    self._Control_Command[0]
                    * ma.cos(self._State_Prediction[4] + self._Slip_angle)
                ],
                [
                    self._Control_Command[0]
                    * ma.sin(self._State_Prediction[4] + self._Slip_angle)
                ],
                [
                    V_T
                    * ma.cos(self._Slip_angle)
                    * ma.tan(self._Control_Command[1])
                    / self._Vehicle_Total_Length
                ],
            ],
            dtype="float",
        ).reshape([3, 1])
        Jacobian__Measure_Accelerometer_Model = np.array(
            [
                [
                    0,
                    0,
                    0,
                    0,
                    -self._Control_Command[0]
                    * ma.sin(self._State_Prediction[4] + self._Slip_angle),
                    0,
                    0,
                ],
                [
                    0,
                    0,
                    0,
                    0,
                    self._Control_Command[0]
                    * ma.cos(self._State_Prediction[4] + self._Slip_angle),
                    0,
                    0,
                ],
                [
                    0,
                    0,
                    self._State_Prediction[2]
                    * ma.cos(self._Slip_angle)
                    * ma.tan(self._Control_Command[1])
                    / (V_T * self._Vehicle_Total_Length),
                    self._State_Prediction[3]
                    * ma.cos(self._Slip_angle)
                    * ma.tan(self._Control_Command[1])
                    / (V_T * self._Vehicle_Total_Length),
                    0,
                    0,
                    0,
                ],
            ],
            dtype="float",
        )
        if self._Number_of_Cones > 0:
            F = np.block([np.eye(7), np.zeros([7, 2 * (self._Number_of_Cones - 1)])],)
        else:
            F = np.block([[np.eye(5)], [np.zeros([2, 5])]])
        Jacobian__Measure_Accelerometer_Model = (
            Jacobian__Measure_Accelerometer_Model @ F
        )
        S_Inv_Acc_Model = inv(
            np.array(
                Jacobian__Measure_Accelerometer_Model
                @ self._Covariance_Prediction
                @ Jacobian__Measure_Accelerometer_Model.T
                + self._Measure_Acc_Noise,
                dtype="float",
            )
        )
        Model_Gain_Acc = (
            self._Covariance_Prediction
            @ np.transpose(Jacobian__Measure_Accelerometer_Model)
            @ S_Inv_Acc_Model
        )
        self._State_Correction = Model_Gain_Acc @ (
            self._Measure_Accelerometer - self._Measure_Accelerometer_Model
        )
        Covariance_Minimal = Model_Gain_Acc @ Jacobian__Measure_Accelerometer_Model
        if np.size(self._Measure_GPS) > 0:
            Jacobian__Measure_Gps_Model = np.array(
                [[1, 0, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0, 0],], dtype="float",
            )
            Jacobian__Measure_Gps_Model = Jacobian__Measure_Gps_Model @ F
            S_Inv_GPS_Model = inv(
                Jacobian__Measure_Gps_Model
                @ self._Covariance_Prediction
                @ Jacobian__Measure_Gps_Model.T
                + self._Measure_GPS_Noise,
            )
            Model_Gain_GPS = (
                self._Covariance_Prediction
                @ np.transpose(Jacobian__Measure_Gps_Model)
                @ S_Inv_GPS_Model
            )
            self._State_Correction = self._State_Correction + Model_Gain_GPS @ (
                self._Measure_GPS - self._Measure_GPS_Model
            )
            Covariance_Minimal = (
                Covariance_Minimal + Model_Gain_GPS @ Jacobian__Measure_Gps_Model
            )
        if np.size(self._External_Measure_Update) > 0:
            if self._Number_of_Cones == 0:
                Covariance_Minimal = self.Cone_Initialization_function(
                    Covariance_Minimal
                )
            for Measure_Update in self._External_Measure_Update:
                Measure_Update = np.reshape(Measure_Update, [2, 1])
                Observation = [
                    self._State_Prediction[0]
                    + Measure_Update[0]
                    * ma.cos(
                        Measure_Update[1] + self._State_Prediction[4] + self._Slip_angle
                    ),
                    self._State_Prediction[1]
                    + Measure_Update[0]
                    * ma.sin(
                        Measure_Update[1] + self._State_Prediction[4] + self._Slip_angle
                    ),
                ]
                Pi_Threshold = 100000
                for K in range(1, 2 * self._Number_of_Cones, 2):
                    Delta = np.array(
                        [
                            self._State_Prediction[4 + K] - self._State_Prediction[0],
                            self._State_Prediction[5 + K] - self._State_Prediction[1],
                        ]
                    )
                    q = np.transpose(Delta) @ Delta
                    Estimate_Measure = np.array(
                        [
                            [ma.sqrt(q)],
                            [ma.atan2(Delta[1], Delta[0]) - self._State_Prediction[4]],
                        ],
                        dtype="float",
                    )
                    F = np.block(
                        [
                            [np.eye(5), np.zeros([5, 2 * (self._Number_of_Cones)])],
                            [
                                np.zeros([2, 4 + K]),
                                np.eye(2),
                                np.zeros([2, 2 * self._Number_of_Cones - K - 1]),
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
                        @ self._Covariance_Prediction
                        @ np.transpose(External_Jacobian)
                        + self._External_Measure_Noise
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
                if Pi_Threshold < self._Alpha:
                    Kalman_Gain = (
                        self._Covariance_Prediction
                        @ np.transpose(Minimal_Jacobian)
                        @ S_Minimal
                    )
                    Covariance_Minimal = (
                        Covariance_Minimal + Kalman_Gain @ Minimal_Jacobian
                    )
                    self._State_Correction = self._State_Correction + Kalman_Gain @ (
                        Measure_Update - Estimate_Measure
                    )
                else:
                    self._State_Prediction = np.concatenate(
                        [self._State_Prediction, Observation]
                    )
                    self._State_Correction = np.concatenate(
                        [self._State_Correction, np.zeros([2, 1])]
                    )
                    self._Number_of_Cones = self._Number_of_Cones + 1
                    self._Covariance_Prediction = np.block(
                        [
                            [
                                self._Covariance_Prediction,
                                np.zeros([self._Covariance_Prediction.shape[0], 2]),
                            ],
                            [
                                np.zeros([2, self._Covariance_Prediction.shape[1]]),
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
            self._State_Correction = self._State_Prediction + self._State_Correction
            self._Covariance_Update = (
                np.eye(self._State_Correction.shape[0]) - Covariance_Minimal
            ) @ self._Covariance_Prediction
        else:
            self._State_Correction = self._State_Prediction + self._State_Correction
            self._Covariance_Update = (
                np.eye(self._State_Correction.shape[0]) - Covariance_Minimal
            ) @ self._Covariance_Prediction

    def Cone_Initialization_function(self, Covariance_Minimal):
        self._External_Measure_Update = np.array(self._External_Measure_Update)
        Observation = np.array(
            [
                self._State_Prediction[0]
                + self._External_Measure_Update[0][0]
                * ma.cos(
                    self._External_Measure_Update[0][1]
                    + self._State_Prediction[4]
                    + self._Slip_angle
                ),
                self._State_Prediction[1]
                + self._External_Measure_Update[0][0]
                * ma.sin(
                    self._External_Measure_Update[0][1]
                    + self._State_Prediction[4]
                    + self._Slip_angle
                ),
            ],
            dtype="float",
        )
        self._State_Prediction = np.concatenate(
            [self._State_Prediction, Observation.reshape([2, 1])]
        )
        self._State_Correction = np.concatenate(
            [self._State_Correction, np.zeros([2, 1])]
        )
        self._Number_of_Cones = self._Number_of_Cones + 1
        self._Covariance_Prediction = np.block(
            [
                [
                    self._Covariance_Prediction,
                    np.zeros([self._Covariance_Prediction.shape[0], 2]),
                ],
                [
                    np.zeros([2, self._Covariance_Prediction.shape[1]]),
                    0.01 * np.eye(2),
                ],
            ]
        )
        self._External_Measure_Update = self._External_Measure_Update[1:, :]
        Covariance_Minimal = np.block(
            [
                [Covariance_Minimal, np.zeros([Covariance_Minimal.shape[0], 2]),],
                [np.zeros([2, Covariance_Minimal.shape[1]]), np.zeros([2, 2]),],
            ]
        )
        return Covariance_Minimal
