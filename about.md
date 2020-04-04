
# Sensors

## SPC speed sensor SP1117

- measurment freaquency - 8KHz
- error/resolotion depends on the ditstance between measured magnetic pulses. this is based on mechanical design and build and can be measuerd on the car. from mechanical design was found to be 8 pulses per rotation with wheel diameter of 396mm which gives an error/resolotion of 155.5mm.

## KR22 rotary potentiometer

- continuous measurment
- max operation speed is 150 RPM
- Resolution - Essentiacilly infinite // this is a direct qoute from the sensor 				datasheet. this will be found more precisely in testing in idle conditions to 			see how irattic the readings are
- from log files found that MoTec returns throttel persentage with error/resolotion of 0.1%.

## KP94B Miniature Linear Position Sensor

- continuous measurment
- max operation speed is 10 M/s
- Resolution - Essentiacilly infinite // this is a direct qoute from the sensor datasheet. when looking at car logs found to be 0.2 degrees of steering angle.

## KH31 rotary position sensor

- measurment freaquency - 5KHz
- measures every 1 degree with an error of 0.1 degrees in measurment

## MM5.10 acceleration sensor (F 02U V01 511-02, F 02U V01 511-91,  F 02U V01 512-02)

- continuous measurment
- cutoff frequency - 15,30,60Hz accordingly
- rotation rate measuring range +-160 deg/s
- rotation rate error/resolotion - 0.1 deg/s
- acceleration measuring range +- 4.2 g
- acceleration error/resolotion - 0.01 g 

## Pressure Sensor Fluid PSS-260 (break preasure sensor)

- runs from 0 to 260 bar
- tolerance of 1% at 0-100 degrees celcius
- Sensitivity 15.38 mV/bar at US = 5 V
- Offset 500 mV at US = 5 V


In summary to get data with the same timestamp from all sensors, we need to measure at 0.833 KHz or 1.2ms


# MoTeC-130

## Hardware

### Input

- 8 Analog inputs; 12 bits. Full-Scale (-30,35)[V]
- 2 High-band width KNOCK. 12 bits. Full-Scale (-30,35)[V]
- 3 Inner accelometeres (G sensors)
- 7 Universal-Digital 10 bits. Full-Scale (-10.7 , 11.4)[V].
   compatible with HALL sensors.

### Output

- 6 Half-Bridge (comaptible with servo)
- Low Side Inkector Output; 24bit timers; RMS current 2[A];   max 60[V];
- Peak Hold Injector.
- Low side ignition


### Communication

- CAN-bus 2.0b
- Ethernet 10/100/
- 2 X   5[V] Voltage
- 6.3[V] 
- Continous Battery Input

# Study

## Kalman Filtering
- [Kalman Filters Videos](https://youtu.be/ul3u2yLPwU0)
- Kalman Filters Matlab Example: ``` openExample('control/KalmanFilteringExample') ```
- [Implementation guide to python](https://towardsdatascience.com/kalman-filters-a-step-by-step-implementation-guide-in-python-91e7e123b968) 
- [Dynamic model example with Kalman Filtering on Simulink] ```openExample('control/KalmanTimeVaryingExample')```

## Vehicle Dynamic Modeling
- Vehicle Dynamics System Matlab Example: ```openExample('ident/idnlgreydemo11') ```



## ORB_SLAM2

### input

- camera image of current frame
- timestamp of the current image frame

### output

- 2D pointcloud of key features
- the last frame that was captured

### method

- searches for only key features in the frame. 
Tries to close loops over the images to build a map and localize in respect to earlier images.

### pros

- doesn't use a lot of computing time or need heavyduty hardware due to only using key features in a frame and not every pixel in frame

### cons

- doesn't work well at high speeds due to high percentage of key features changing between frames 
  
## Rovio

### input

- camera image of current frame
- IMU measurments

### output

- angle to each key feature from current location
- distance vector to each key feature from current location

### method

- by adding the error in position of features in the stream to an Extended Kalman Filter's update stage 
it adds stability in motion and movments when location cannot be determind from camera due to loss of key features. 
when this happenes estimation is not is accurate but isn't lost completely.  

### pros

- IMU measurements make the system more robust at high speeds.

### cons

- requires accurate dynamic model.
- requires high accuracy IMU sensors.

## Servo

### input

- camera image of current frame
- timestamp of the current image frame
- IMU measurments

### output

- 2D pointcloud of key features
- the last frame that was captured

### method

- this is an extention of ORB_SLAM2.
the ORB_SLAM2 is added IMU measurments to better approximate the location and avoid losing tracking
and spacial awareness when no similar key features are found between frames.
very similar to rovio and even uses it in current version but the idea is to use the camera adn finetune with IMU
as apposed to rovio that dose the oposite.

### pros

- doesn't use a lot of computing time or need heavyduty hardware due to only using key features in a frame and not every pixel in frame
- doesn't lose localization that easaly at high speeds due to coupling with IMU data.

### cons

- requires at current implementation high accuracy sensing and precise dynamic model 
