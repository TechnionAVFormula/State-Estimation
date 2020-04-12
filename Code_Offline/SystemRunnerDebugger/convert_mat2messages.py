#!python
#!/usr/bin/env python
# Common tools:
import scipy.io as sio
import math
import numpy as np 
import matplotlib.pyplot as plt 

#oct2py is used to run octave code on a python script. 
# octave is a tool to  read and manage matlab scripts and files
from oct2py import Oct2Py
oc = Oct2Py()

#Messages and Client stuff:
from create_message_file import create_cone_message , create_gps_message , create_IMU_message
from pyFormulaClientNoNvidia import messages
from pyFormulaClientNoNvidia.FormulaClient import FormulaClient, ClientSource, SYSTEM_RUNNER_IPC_PORT


def calc_perception_cone(cone_x , cone_y , car_x , car_y , car_theta):
    delta_x = cone_x - car_x
    delta_y = cone_y - car_y
    angle_from_car_position = math.atan2(delta_y , delta_x)

    perception_angle    = car_theta - angle_from_car_position
    perception_distance = math.sqrt(  (delta_x**2)   +  (delta_y**2)   )

    perception_x = perception_distance*math.sin(perception_angle)
    perception_y = perception_distance*math.cos(perception_angle)

    return perception_x , perception_y



def main():

    # Establish the client:
    perception_client = FormulaClient(ClientSource.PERCEPTION, 
        read_from_file=os.devnull, write_to_file='fromSimulation.messages')
    perception_conn = perception_client.connect(SYSTEM_RUNNER_IPC_PORT)


    # use octave with oct2py to load the struct:
    simulation_results = 'LoopTest_01.mat'
    data = oc.feval("C:\\Users\\NGBig\\Documents\\GitHub\\State-Estimation\\Code_Offline\\SimulationTests\\read_simulation_results.m",simulation_results)

    # The sub stucts:
    Cones           = data.Cones
    Measurements    = data.Measurements
    Ground_Truth    = data.Ground_Truth

    #get length of data
    num = {}
    num['tests']       = int(round(  oc.size(Cones.Time)[0][0]           )) 
    num['left_cones']  = int(round(  oc.size(Cones.LeftConesSeen)[0][1]  ))
    num['right_cones'] = int(round(  oc.size(Cones.rightConesSeen)[0][1] ))


    # for each time stamp:
    for i in range(num['tests']):
        real_x      = Ground_Truth.x[i][0]
        real_y      = Ground_Truth.y[i][0]
        real_theta  = Ground_Truth.theta[i][0] 
        

        cone_arr = []  
        #For each cone, check if it seen by car. if it is: make
        for cone_ind in range(num['left_cones']):
            is_seen = Cones.LeftConesSeen[i][cone_ind]
            if is_seen:
                cone_noised_x =  Cones.LeftConesNoise[cone_ind][0][i]
                cone_noised_y =  Cones.LeftConesNoise[cone_ind][1][i]
                perception_cone_x , perception_cone_y = calc_perception_cone(cone_noised_x , cone_noised_y , real_x , real_y , real_theta)        
                perception_cone_z = 0
                perception_cone_id = cone_ind
                cone_arr.append( 
                    {
                        "cone_id": perception_cone_id,
                        "x": perception_cone_x,
                        "y": perception_cone_y,
                        "z": perception_cone_z,
                        "type": messages.perception.Blue
                    },
                 )                
        if len( cone_arr ) > 0:
            msg = create_cone_message(cone_arr)
            msg.header.timestamp.CopyFrom(messages.get_proto_system_timestamp())
            perception_conn.send_message(msg)


'''
end main
'''

if __name__ == '__main__':
    main()    

print(0)






