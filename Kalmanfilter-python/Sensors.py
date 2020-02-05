# Sensors
import math
import numpy as np
import matplotlib.pyplot as plt
from ExcelReading import excel_function

R = 6371 * pow(10, 3)  # Earth radius
Eperimetertoangular = R * 2 * math.pi / 360  # Earth perimeter
Tie_rod_wheel_pose = 46  # The longitude distance in [mm]
Delta_time = 0.01  # Sampling time


Sensors_Data = excel_function()
print(Sensors_Data)
# initial position GPS
ZeroLong = Sensors_Data[0, 0]
ZeroLatitude = Sensors_Data[0, 1]

# The function GPSTranslate transfer the coordinate data from geographic coordinate data
# to eucludean the columns express the north and the east coordinate respectively.
def GPSTranslate():
    arr = np.empty([len(Sensors_Data), 2])
    for i in range(0, len(Sensors_Data)):
        arr[i, :] = np.array(
            [
                Eperimetertoangular * (Sensors_Data[i, 1] - ZeroLatitude),
                Eperimetertoangular
                * (Sensors_Data[i, 0] - ZeroLong)
                * math.cos(Sensors_Data[i, 1] / 180 * math.pi),
            ]
        )
    return arr


def SteeringTranslate():
    arr = np.empty([len(Sensors_Data), 2])
    for i in range(0, len(Sensors_Data)):
        arr[i, :] = [
            Delta_time * i / 60,
            math.atan2(Sensors_Data[i, 2], Tie_rod_wheel_pose),
        ]

    return arr


plt.figure(1)
GPSpose = GPSTranslate()
plt.plot(GPSpose[:, 0], GPSpose[:, 1])
plt.grid()

plt.figure(2)
Steering_angle = SteeringTranslate()
plt.plot(
    Steering_angle[range(0, len(Steering_angle), 300), 0],
    Steering_angle[range(0, len(Steering_angle), 300), 1] / math.pi * 90,
)
plt.grid()
# print(Steering_angle[0:10, :])
plt.show()

