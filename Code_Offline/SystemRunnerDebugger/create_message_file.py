from pyFormulaClientNoNvidia import messages
from pyFormulaClientNoNvidia.FormulaClient import FormulaClient, ClientSource, SYSTEM_RUNNER_IPC_PORT

import os

NOT_AVAILABLE = -1000

def create_cone_message(cone_arr):
    msg = messages.common.Message()
    data = messages.perception.ConeMap()
    # msg.header.id = create_cone_message._msg_id
    # create_cone_message._msg_id += 1
    for cone_data in cone_arr:
        cone = messages.perception.Cone()
        cone.x = cone_data['x']
        cone.y = cone_data['y']
        cone.z = cone_data['z']
        cone.type = cone_data['type']
        data.cones.append(cone)
        
    msg.data.Pack(data)
    return msg

def create_gps_message(x,y,z):
    sensor_data = messages.sensors.GPSSensor()
    sensor_data.position.x = x
    sensor_data.position.y = y
    sensor_data.position.z = z
    sensor_msg = messages.common.Message()
    sensor_msg.data.Pack(sensor_data) 
    return sensor_msg

def make_CarMeasurments( delta ):
    CarMeasurment = messages.sensors.CarMeasurments()
    CarMeasurment.wheel_velocity_rear_left = NOT_AVAILABLE
    CarMeasurment.wheel_velocity_rear_right = NOT_AVAILABLE
    CarMeasurment.wheel_velocity_front_left = NOT_AVAILABLE
    CarMeasurment.wheel_velocity_front_right = NOT_AVAILABLE
    CarMeasurment.throttle_position = NOT_AVAILABLE
    CarMeasurment.steering_angle = delta   # radians. right positive
    CarMeasurment.brakes_position = NOT_AVAILABLE
    return CarMeasurment

def make_IMUMeasurments(acceleration_long , acceleration_lat , omega , theta , speed ):
    IMUMeasurments = messages.sensors.IMUMeasurments()
    IMUMeasurments.acceleration.x = acceleration_long
    IMUMeasurments.acceleration.y = acceleration_lat
    IMUMeasurments.acceleration.z = NOT_AVAILABLE
    IMUMeasurments.angular_acceleration.x = NOT_AVAILABLE
    IMUMeasurments.angular_acceleration.y = NOT_AVAILABLE
    IMUMeasurments.angular_acceleration.z = NOT_AVAILABLE
    IMUMeasurments.velocity.x = NOT_AVAILABLE
    IMUMeasurments.velocity.y = NOT_AVAILABLE
    IMUMeasurments.velocity.z = NOT_AVAILABLE
    IMUMeasurments.angular_velocity.x = NOT_AVAILABLE
    IMUMeasurments.angular_velocity.y = NOT_AVAILABLE
    IMUMeasurments.angular_velocity.z = omega
    IMUMeasurments.orientation.x = NOT_AVAILABLE
    IMUMeasurments.orientation.y = NOT_AVAILABLE
    IMUMeasurments.orientation.z = theta
    IMUMeasurments.speed = speed
    return IMUMeasurments


def create_car_data_message(CarMeasurments , IMUMeasurments ):
    car_data = messages.sensors.CarData()
    car_data.car_measurments.CopyFrom( CarMeasurments )  
    car_data.imu_sensor.imu_measurments.CopyFrom( IMUMeasurments )
    CarData_message = messages.common.Message()
    CarData_message.header.id = 350
    CarData_message.data.Pack(car_data)
    return CarData_message

'''
def create_IMU_message(v , theta):
    sensor_data = messages.sensors.IMUSensor()
    sensor_data.velocity.x = v[0]
    sensor_data.velocity.y = v[1]
    sensor_data.velocity.z = v[2]
    sensor_data.orientation.x = -1
    sensor_data.orientation.y = -1
    sensor_data.orientation.z = theta
    sensor_msg = messages.common.Message()
    sensor_msg.header.id = 200
    sensor_msg.data.Pack(sensor_data) 
    return sensor_msg
'''

def create_ground_truth_message( car_position , CarMeasurments , IMUMeasurments , cone_array):
    gt_data = messages.ground_truth.GroundTruth()

    if (car_position == None):
        gt_data.has_position_truth = False
    else:
        gt_data.has_position_truth = True
        gt_data.position.x = car_position[0]
        gt_data.position.y = car_position[1]
        gt_data.position.z = 0

    if CarMeasurments == None:
        gt_data.has_car_measurments_truth = False
    else:
        gt_data.has_car_measurments_truth = True
        gt_data.car_measurments.CopyFrom(CarMeasurments)
    
    if IMUMeasurments == None:
        gt_data.has_imu_measurments_truth = False
    else:
        gt_data.has_imu_measurments_truth = True
        gt_data.imu_measurments.CopyFrom(IMUMeasurments)

    if (cone_array == None):
        gt_data.has_cones_truth  = False
    else:
        gt_data.has_cones_truth  = True
        for cone_data in cone_array:
            cone = messages.state_est.StateCone()
            cone.position.x = cone_data['x']
            cone.position.y = cone_data['y']
            cone.cone_id = cone_data['cone_id']
            cone.type = cone_data['type']
            cone.position_deviation = 0
            gt_data.cones  .append(cone)

    gt_msg = messages.common.Message()
    gt_msg.data.Pack(gt_data) 
    return gt_msg   
        

# create_cone_message._msg_id = 1

def main():

    # Establish the client:
    perception_client = FormulaClient(ClientSource.PERCEPTION, 
        read_from_file=os.devnull, write_to_file='fromSimulation.messages')
    perception_conn = perception_client.connect(SYSTEM_RUNNER_IPC_PORT)
    
    
    #================================================================================================#
    #VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV#

    # car_data:
    car_measurments = make_CarMeasurments(3)
    imu_measurments = make_IMUMeasurments(1.2 , 3.4 , 5.6 , 0.78 , NOT_AVAILABLE  )
    msg = create_car_data_message(car_measurments , imu_measurments) 
    perception_conn.send_message(msg)

    # Ground Truth:
    cone_array = [
        {
            "cone_id": 1,
            "x": 10,
            "y":-10,
            "type": messages.perception.Yellow
        },
        {
            "cone_id": 2,
            "x": 10,
            "y":-10,
            "type": messages.perception.Blue
        },
        {
            "cone_id": 3,
            "x": 10,
            "y":-10,
            "type": messages.perception.Blue
        }
    ]
    gt_car_measurments = make_CarMeasurments(2)
    gt_imu_measurments = make_IMUMeasurments(2.1 , 4.3 , 6.5 , 0.87 , 9)
    msg = create_ground_truth_message( [50 , 60] , gt_car_measurments , gt_imu_measurments , cone_array)
    perception_conn.send_message(msg)


    # GPS:
    x = 0 
    y = 0
    z = 0
    sensor_msg = create_gps_message(x,y,z)
    perception_conn.send_message(sensor_msg)
    
    

    # Cones:
    cone_arr = [
        {
            "cone_id": 1,
            "x": 10,
            "y":-10,
            "z": 2,
            "type": messages.perception.Blue
        },
        {
            "cone_id": 2,
            "x": 10,
            "y": 10,
            "z": 1,
            "type": messages.perception.Yellow
        }, 
        {
            "cone_id": 3,
            "x": 20,
            "y":-10,
            "z": 2,
            "type": messages.perception.Blue
        }
    ]
    msg = create_cone_message(cone_arr)
    msg.header.timestamp.CopyFrom(messages.get_proto_system_timestamp())
    perception_conn.send_message(msg)


    # GPS:
    x = 9.5  #almost 10 meters towards the east 
    y = 0
    z = 0
    sensor_msg = create_gps_message(x,y,z)
    perception_conn.send_message(sensor_msg)
    



    # Cones:
    cone_arr = [
        {
            "cone_id": 1,
            "x": 0,
            "y":-10,
            "z": 2,
            "type": messages.perception.Blue
        },
        {
            "cone_id": 2,
            "x": 0,
            "y": 10,
            "z": 1,
            "type": messages.perception.Yellow
        }, 
        {
            "cone_id": 3,
            "x": 10,
            "y":-10,
            "z": 2,
            "type": messages.perception.Blue
        },

        #new cone:
        {
            "cone_id": 4,
            "x": 10,
            "y": 10,
            "z": 2.3,
            "type": messages.perception.Yellow
        }

    ]
    msg = create_cone_message(cone_arr)
    msg.header.timestamp.CopyFrom(messages.get_proto_system_timestamp())
    perception_conn.send_message(msg)



    #^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^#
    #================================================================================================#

    # Exit:
    exit_data = messages.server.ExitMessage()
    exit_msg = messages.common.Message()
    exit_msg.data.Pack(exit_data)
    perception_conn.send_message(exit_msg)


if __name__ == '__main__':
    main()
