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
import os

'''
Values to be used:
'''
PRINT_ON_MSG = True
INPUT_MAT_NAME =  'Test_01.mat'
PATH2READ_SIMULATION_RESULTS_M = "C:\\Users\\NGBig\\Documents\\GitHub\\State-Estimation\\Code_Offline\\SimulationTests\\read_simulation_results.m"

perception_sampling_time_milisec = (1/30) * 1000   # in [m sec]   for 30 Hz Fs

'''
Main:
'''
def calc_perception_cone(cone_x , cone_y , car_x , car_y , car_theta):
    delta_x = cone_x - car_x
    delta_y = cone_y - car_y
    angle_from_car_position = math.atan2(delta_y , delta_x)

    perception_angle    = car_theta - angle_from_car_position
    perception_distance = math.sqrt(  (delta_x**2)   +  (delta_y**2)   )

    perception_x = perception_distance*math.sin(perception_angle)
    perception_y = perception_distance*math.cos(perception_angle)

    return perception_x , perception_y

def add_cone_to_cone_array( Ground_Truth , noised_cones , cone_arr , test_ind , cone_ind ):
    #true position:
    real_x      = Ground_Truth.x[test_ind][0]
    real_y      = Ground_Truth.y[test_ind][0]
    real_theta  = Ground_Truth.theta[test_ind][0] 

    # creating noised cones from car's perspective:
    cone_noised_x =  noised_cones[cone_ind][0][test_ind]
    cone_noised_y =  noised_cones[cone_ind][1][test_ind]
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
        }
    ) 
    return cone_arr


def main(output_file_name , input_mat_name):


    # Establish the client:
    perception_client = FormulaClient(ClientSource.PERCEPTION, 
        read_from_file=os.devnull, write_to_file=output_file_name)
    perception_conn = perception_client.connect(SYSTEM_RUNNER_IPC_PORT)


    # use octave with oct2py to load the struct:
    simulation_results =  input_mat_name 
    data = oc.feval(PATH2READ_SIMULATION_RESULTS_M,simulation_results)

    # The sub stucts:
    Cones           = data.Cones
    Measurements    = data.Measurements
    Ground_Truth    = data.Ground_Truth

    #get length of data
    num = {}
    num['tests']       = int(round(  oc.size(Cones.Time)[0][0]           )) 
    num['left_cones']  = int(round(  oc.size(Cones.LeftConesSeen)[0][1]  ))
    num['right_cones'] = int(round(  oc.size(Cones.RightConesSeen)[0][1] ))


    last_perception_sample_milisec = 0
    # for each time stamp:
    # vvvvvvvvvvvvvvvvvvvvvvvvvvv #
    for test_ind in range(num['tests']):
        time_in_milisec = int( ( Ground_Truth.Time[test_ind][0] )*1000 )

        '''Cones:'''
        #only sample when perception is able to:
        if time_in_milisec >= last_perception_sample_milisec + perception_sampling_time_milisec:
            last_perception_sample_milisec=time_in_milisec

            cone_arr = [] # make an empty cone array  
            #For each left cone, check if it seen by car. 
            for cone_ind in range(num['left_cones']):
                is_seen = Cones.LeftConesSeen[test_ind][cone_ind]
                if is_seen:
                    cone_arr = add_cone_to_cone_array( Ground_Truth , Cones.LeftConesNoise 
                                                        , cone_arr , test_ind , cone_ind )  
            #For each right cone, check if it seen by car. if it is: make
            for cone_ind in range(num['right_cones']):
                is_seen = Cones.RightConesSeen[test_ind][cone_ind]
                if is_seen:
                    cone_arr = add_cone_to_cone_array( Ground_Truth , Cones.RightConesNoise 
                                                        , cone_arr , test_ind , cone_ind )   
            # if there are cones, make a cone message:             
            if len( cone_arr ) > 0:
                if PRINT_ON_MSG:
                    print(f"cone msg on test {test_ind}")
                msg = create_cone_message(cone_arr)
                msg.header.timestamp.FromMilliseconds( time_in_milisec )
                perception_conn.send_message(msg)


        '''Cones:'''
    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
    # END for each time stamp:

    # Exit:
    exit_data = messages.server.ExitMessage()
    exit_msg = messages.common.Message()
    exit_msg.data.Pack(exit_data)
    perception_conn.send_message(exit_msg)

'''
end main
'''

if __name__ == '__main__':
    output_file_name = 'fromSimulation.messages'
    main(output_file_name , INPUT_MAT_NAME )
    print("Done.")
    print(f"File saved as {output_file_name}")    









