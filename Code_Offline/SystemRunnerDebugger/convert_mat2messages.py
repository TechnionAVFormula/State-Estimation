#!python
#!/usr/bin/env python
import scipy.io as sio
import math
import numpy as np 
import matplotlib.pyplot as plt 
from oct2py import Oct2Py
oc = Oct2Py()

# use octave with oct2py to load the struct:
simulation_results = 'LoopTest_01.mat'
data = oc.feval("C:\\Users\\NGBig\\Documents\\GitHub\\State-Estimation\\Code_Offline\\SimulationTests\\read_simulation_results.m",simulation_results)

# The sub stucts:
Cones           = data.Cones
Measurements    = data.Measurements
Ground_Truth    = data.Ground_Truth

#get length of data
time_length     = int(round(  oc.size(Cones.Time)[0][0]           )) 
num_left_cones  = int(round(  oc.size(Cones.LeftConesSeen)[0][1]  ))
num_right_cones = int(round(  oc.size(Cones.rightConesSeen)[0][1] ))

for i in range(time_length):
    real_x      = Ground_Truth.x[i][0]
    real_y      = Ground_Truth.y[i][0]
    real_theta  = Ground_Truth.theta[i][0] 
    
    for cone_ind in range(num_left_cones):
        is_seen = Cones.LeftConesSeen[i]
        
        
        cone_noised_x = Cones
        cone_noised_y
    

    perception_cone_x , perception_cone_y = calc_perception_cone(cone_noised_x , cone_noised_y , real_x , real_y , real_theta)
    perception_cone_z = 0




print(0)




def calc_perception_cone(cone_x , cone_y , car_x , car_y , car_theta):
    delta_x = cone_x - car_x
    delta_y = cone_y - car_y
    angle_from_car_position = math.atan2(delta_y , delta_x)

    perception_angle    = car_theta - angle_from_car_position
    perception_distance = math.sqrt(  (delta_x**2)   +  (delta_y**2)   )

    perception_x = perception_distance*math.sin(perception_angle)
    perception_y = perception_distance*math.cos(perception_angle)

    return perception_cone_x , perception_cone_y