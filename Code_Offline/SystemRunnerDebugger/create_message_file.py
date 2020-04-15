from pyFormulaClientNoNvidia import messages
from pyFormulaClientNoNvidia.FormulaClient import FormulaClient, ClientSource, SYSTEM_RUNNER_IPC_PORT

import os

def create_cone_message(cone_arr):
    msg = messages.common.Message()
    data = messages.perception.ConeMap()
    msg.header.id = create_cone_message._msg_id
    create_cone_message._msg_id += 1
    i = 0
    for cone_data in cone_arr:
        cone = messages.perception.Cone()
        cone.x = cone_data['x']
        cone.y = cone_data['y']
        cone.z = cone_data['z']
        cone.type = cone_data['type']
        data.cones.append(cone)
        i += 1

    msg.data.Pack(data)
    return msg

def create_gps_message(x,y,z):
    sensor_data = messages.sensors.GPSSensor()
    sensor_data.position.x = x
    sensor_data.position.y = y
    sensor_data.position.z = z
    sensor_msg = messages.common.Message()
    sensor_msg.header.id = 100
    sensor_msg.data.Pack(sensor_data) 
    return sensor_msg

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

create_cone_message._msg_id = 1

def main():

    # Establish the client:
    perception_client = FormulaClient(ClientSource.PERCEPTION, 
        read_from_file=os.devnull, write_to_file='fromSimulation.messages')
    perception_conn = perception_client.connect(SYSTEM_RUNNER_IPC_PORT)
    
    
    #================================================================================================#
    #VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV#


    
    # GPS:
    x = 0 
    y = 0
    z = 0
    sensor_msg = create_gps_message(x,y,z)
    perception_conn.send_message(sensor_msg)
    
    # IMU:
    velocity = ( 0.2 , 1 , -0.01)
    orientation = 0.1   #radians
    IMU_msg = create_IMU_message(velocity , orientation)
    perception_conn.send_message(IMU_msg)


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
    
    # IMU:
    velocity = ( 0.5 , 1 , -0.01)
    orientation = -0.1   #radians
    IMU_msg = create_IMU_message(velocity , orientation)
    perception_conn.send_message(IMU_msg)



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
