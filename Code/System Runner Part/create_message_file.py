from pyFormulaClient import messages
from pyFormulaClient.NoNvidiaFormulaClient import FormulaClient, ClientSource, SYSTEM_RUNNER_IPC_PORT

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

create_cone_message._msg_id = 1

def main():
    perception_client = FormulaClient(ClientSource.PERCEPTION, 
        read_from_file=os.devnull, write_to_file='perception.messages')
    perception_conn = perception_client.connect(SYSTEM_RUNNER_IPC_PORT)
    
    cone_arr = [
        {
            "cone_id": 20,
            "x": 10,
            "y": 10,
            "z": 10,
            "type": messages.perception.Blue
        },
        {
            "cone_id": 30,
            "x": 20,
            "y": 20,
            "z": 20,
            "type": messages.perception.Yellow
        } 
    ]
    msg = create_cone_message(cone_arr)
    msg.header.timestamp.CopyFrom(messages.get_proto_system_timestamp())
    perception_conn.send_message(msg)
    msg = create_cone_message(cone_arr)
    msg.header.timestamp.CopyFrom(messages.get_proto_system_timestamp())
    perception_conn.send_message(msg)
    
    sensor_data = messages.sensors.GPSSensor()
    sensor_data.position.x = 10
    sensor_data.position.y = 10
    sensor_data.position.z = 10
    sensor_msg = messages.common.Message()
    sensor_msg.header.id = 100
    sensor_msg.data.Pack(sensor_data)
    perception_conn.send_message(sensor_msg)

    exit_data = messages.server.ExitMessage()
    exit_msg = messages.common.Message()
    exit_msg.data.Pack(exit_data)
    perception_conn.send_message(exit_msg)


if __name__ == '__main__':
    main()
