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

# for relative path
import os 
import sys
current_dir_name = os.path.dirname(__file__)
relative_dir_name = os.path.join(current_dir_name, '../SystemRunnerDebugger')
sys.path.append(relative_dir_name)

#Making Messages:
from create_message_file import make_IMUMeasurments , make_CarMeasurments
from create_message_file import create_cone_message , create_gps_message , create_car_data_message  , create_ground_truth_message
from create_message_file import NOT_AVAILABLE as SENSOR_NOT_AVAILABLE

# Client for compiling the messages as though someone had sent it in Real-Life
from pyFormulaClientNoNvidia import messages
from pyFormulaClientNoNvidia.FormulaClient import FormulaClient, ClientSource, SYSTEM_RUNNER_IPC_PORT
import os

# Reverse X-Y Axes?
# Simulation uses:  X-east   Y-north
# State-Estimation uses the navigation concensus of xNorth yEast:
IS_xNorth_yEast = True

#enum:
BLUE    = messages.perception.Blue
YELLOW  = messages.perception.Yellow

'''
Values to be used:
'''
PRINT_ON_MSG = True
INPUT_MAT_NAME =  'Test_05.mat'
OUT_FILE_NAME = 'fromSimulation.messages'
PATH2READ_SIMULATION_RESULTS_M = "C:\\Users\\NGBig\\Documents\\GitHub\\State-Estimation\\Code_Offline\\SimulationTests\\read_simulation_results.m"

perception_sampling_time_milisec = (1/30) * 1000   # in [m sec]   for 30 Hz Fs
gps_sampling_time_milisec = (1/2) * 1000 # in [m sec]  for 2 Hz Fs

def flip_x_y( Cones , Measurements , Ground_Truth ):
    
    print("convert_mat2messages: flipping X-Y coordinates")

    num = {}
    num['tests']       = int(round(  oc.size(Cones.Time)[0][0]           )) 
    num['left_cones']  = int(round(  oc.size(Cones.LeftConesSeen)[0][1]  ))
    num['right_cones'] = int(round(  oc.size(Cones.RightConesSeen)[0][1] ))
    

    ## Real Comes:
    for cone_ind in range(num['left_cones']):
        old_x = Cones.LeftConesReal[cone_ind][0]
        old_y = Cones.LeftConesReal[cone_ind][1]
        Cones.LeftConesReal[cone_ind][0]=old_y
        Cones.LeftConesReal[cone_ind][1]=old_x
    for cone_ind in range(num['right_cones']):
        old_x = Cones.RightConesReal[cone_ind][0]
        old_y = Cones.RightConesReal[cone_ind][1]
        Cones.RightConesReal[cone_ind][0]=old_y
        Cones.RightConesReal[cone_ind][1]=old_x


    ## Noised Comes:
    for test_ind in range(num['tests'] ):
        for cone_ind in range(num['left_cones']):
            old_x = Cones.LeftConesNoise[cone_ind][0][test_ind]
            old_y = Cones.LeftConesNoise[cone_ind][1][test_ind]
            Cones.LeftConesNoise[cone_ind][0][test_ind]=old_y
            Cones.LeftConesNoise[cone_ind][1][test_ind]=old_x
        for cone_ind in range(num['right_cones']):
            old_x = Cones.RightConesNoise[cone_ind][0][test_ind]
            old_y = Cones.RightConesNoise[cone_ind][1][test_ind]
            Cones.RightConesNoise[cone_ind][0][test_ind]=old_y
            Cones.RightConesNoise[cone_ind][1][test_ind]=old_x

    ## Ground Truth:
    for test_ind in range(num['tests'] ):
        old_x       = Ground_Truth.x[test_ind][0]
        old_y       = Ground_Truth.y[test_ind][0]
        old_theta   = Ground_Truth.theta[test_ind][0]
        Ground_Truth.x[test_ind][0]     = old_y     
        Ground_Truth.y[test_ind][0]     = old_x     
        Ground_Truth.theta[test_ind][0] = math.pi/2 - old_theta # theta grows from x to y

    ## GPS:
    for test_ind in range(num['tests'] ):
        time_in_milisec = int( ( Ground_Truth.Time[test_ind][0] )*1000 )
        gps_ind , is_exist = find_measurment_at_time( time_in_milisec , Measurements.GPS_Time ) 
        if is_exist:
            old_x = Measurements.GPS_x[gps_ind][0]
            old_y = Measurements.GPS_y[gps_ind][0]
            Measurements.GPS_x[gps_ind][0] = old_y 
            Measurements.GPS_y[gps_ind][0] = old_x

    ## Car measurements:
    for test_ind in range(num['tests']):
        # positive omega (theta dot ) is to the right (intrinsic). positve from new x to new y:
        old_omega = Measurements.omega[ test_ind ][0]
        Measurements.omega[ test_ind ][0] = -old_omega   
        old_delta = Measurements.delta[ test_ind ][0]
        Measurements.delta[ test_ind ][0] = -old_delta
        
    return Cones , Measurements , Ground_Truth 


def calc_perception_cone(cone_x , cone_y , car_x , car_y , car_theta):
    delta_x = cone_x - car_x
    delta_y = cone_y - car_y
    angle_from_car_position = math.atan2(delta_y , delta_x)

    perception_angle    = car_theta - angle_from_car_position
    perception_distance = math.sqrt(  (delta_x**2)   +  (delta_y**2)   )

    perception_x = perception_distance*math.sin(perception_angle)
    perception_y = perception_distance*math.cos(perception_angle)

    return perception_x , perception_y

def add_cone_to_cone_array( Ground_Truth , noised_cones , cone_arr , test_ind , cone_ind , type):
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
            "type": type
        }
    ) 
    return cone_arr



def find_measurment_at_time( time_to_find_in_milisec , measurement_times):
    is_exist = False
    
    for ind in range( len(measurement_times) ):
        measure_time_in_milisec = int( measurement_times[ind][0] * 1000  )

        if measure_time_in_milisec == time_to_find_in_milisec:
            is_exist = True
            return ind , is_exist 

    return ind , is_exist 


'''
Main:
'''


def main(output_file_name , input_mat_name):

    print(f"convert_mat2messages: starting converstion for file: {input_mat_name}")

    # Establish the client:
    perception_client = FormulaClient(ClientSource.PERCEPTION, 
        read_from_file=os.devnull, write_to_file=output_file_name)
    simulation_conn = perception_client.connect(SYSTEM_RUNNER_IPC_PORT)


    # use octave with oct2py to load the struct:
    simulation_results =  input_mat_name 
    data = oc.feval(PATH2READ_SIMULATION_RESULTS_M,simulation_results)
        
    # The sub stucts:
    Cones           = data.Cones
    Measurements    = data.Measurements
    Ground_Truth    = data.Ground_Truth

    if IS_xNorth_yEast:
        Cones , Measurements , Ground_Truth = flip_x_y( Cones , Measurements , Ground_Truth )

    #get length of data
    num = {}
    num['tests']       = int(round(  oc.size(Cones.Time)[0][0]           )) 
    num['left_cones']  = int(round(  oc.size(Cones.LeftConesSeen)[0][1]  ))
    num['right_cones'] = int(round(  oc.size(Cones.RightConesSeen)[0][1] ))

    last_GPS_sample_milisec = 0
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
                                                        , cone_arr , test_ind , cone_ind , BLUE )  
            #For each right cone, check if it seen by car. if it is: make
            for cone_ind in range(num['right_cones']):
                is_seen = Cones.RightConesSeen[test_ind][cone_ind]
                if is_seen:
                    cone_arr = add_cone_to_cone_array( Ground_Truth , Cones.RightConesNoise 
                                                        , cone_arr , test_ind , cone_ind , YELLOW)   
            # if there are cones, make a cone message:             
            if len( cone_arr ) > 0:
                if PRINT_ON_MSG:
                    print(f"cone msg on index {test_ind:5} ; num_cones={len( cone_arr ):4}")
                #create message and send it (to file):
                msg = create_cone_message(cone_arr)
                msg.header.id = test_ind
                msg.header.timestamp.FromMilliseconds( time_in_milisec )
                simulation_conn.send_message(msg)

        '''
        GPS Measurements:
        '''
        if time_in_milisec >= last_GPS_sample_milisec + gps_sampling_time_milisec:
            last_GPS_sample_milisec=time_in_milisec

            gps_ind , is_exist = find_measurment_at_time( time_in_milisec , Measurements.GPS_Time ) 
            if is_exist:
                x = Measurements.GPS_x[gps_ind][0]
                y = Measurements.GPS_y[gps_ind][0]
                z = 0
                if PRINT_ON_MSG:
                    print(f"GPS  msg on index {test_ind:5} ; x={x:5.2f} , y={y:5.2f}")
                #create message and send it (to file):
                msg = create_gps_message(x,y,z)
                msg.header.id = test_ind
                msg.header.timestamp.FromMilliseconds( time_in_milisec )
                simulation_conn.send_message(msg)


        '''
        CarData - CarMeasurments and IMUMeasurements:
        '''
        imu_ind = test_ind
        # get data:
        a_lat  = Measurements.a_lat[ imu_ind][0]
        a_long = Measurements.a_long[imu_ind][0]
        delta  = Measurements.delta[ imu_ind][0]
        omega  = Measurements.omega[ imu_ind][0]
        # Adding fake theta measurment by noising the GT:
        theta = Ground_Truth.theta[  imu_ind][0] + 0.001*np.random.randn()
        # compile sensors data:
        car_measurments = make_CarMeasurments(delta)
        imu_measurments = make_IMUMeasurments( a_long , a_lat , omega , theta , SENSOR_NOT_AVAILABLE   )
        # compile entire message:
        msg = create_car_data_message(car_measurments , imu_measurments )
        msg.header.id = test_ind
        msg.header.timestamp.FromMilliseconds( time_in_milisec )
        simulation_conn.send_message(msg)


        '''
        Ground Truth:
        '''
        cone_array = []
        for cone_ind in range(num['left_cones']):
            x = Cones.LeftConesReal[cone_ind][0]
            y = Cones.LeftConesReal[cone_ind][1]
            cone = {
                "cone_id": cone_ind,
                "x": x,
                "y": y,
                "type": messages.perception.Blue
            }
            cone_array.append(cone)
        for cone_ind in range(num['right_cones']):
            x = Cones.RightConesReal[cone_ind][0]
            y = Cones.RightConesReal[cone_ind][1]
            cone = {
                "cone_id": cone_ind,
                "x": x,
                "y": y,
                "type": messages.perception.Yellow
            }
            cone_array.append(cone)

        gt_ind = test_ind
        # get data:
        delta = Ground_Truth.delta[gt_ind][0]
        theta = Ground_Truth.theta[gt_ind][0]
        v     = Ground_Truth.v[gt_ind][0]
        x     = Ground_Truth.x[gt_ind][0]
        y     = Ground_Truth.y[gt_ind][0]
        # compile sensors data::
        car_position = [x , y]
        car_measurments = make_CarMeasurments(delta)
        imu_measurments = make_IMUMeasurments( SENSOR_NOT_AVAILABLE , SENSOR_NOT_AVAILABLE , SENSOR_NOT_AVAILABLE , theta , v  )
        # compile entire message:
        msg =  create_ground_truth_message( car_position , car_measurments , imu_measurments , cone_array)
        msg.header.id = test_ind
        msg.header.timestamp.FromMilliseconds( time_in_milisec )  
        simulation_conn.send_message(msg)
        
    

    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
    # END for each time stamp:

    # Exit:
    exit_data = messages.server.ExitMessage()
    exit_msg = messages.common.Message()
    exit_msg.data.Pack(exit_data)
    simulation_conn.send_message(exit_msg)

    print("Done.")
    print(f"File saved as {output_file_name}") 

'''
end main
'''

if __name__ == '__main__':
    main(OUT_FILE_NAME , INPUT_MAT_NAME )   







