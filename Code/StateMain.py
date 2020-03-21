from SystemRunnerPart.StateEstClient import StateEstClient
from pyFormulaClientNoNvidia import messages
import time
import signal
import sys
import math

from OrderCones import orderCones

class State:
    def __init__(self):
        self._client = StateEstClient('perception.messages', 'state.messages')
        self._running_id = 1
        self._distance_to_finish = -1
        self.message_timeout = 0.01

    def start(self):
        self._client.connect(1)
        self._client.set_read_delay(0.1)
        self._client.start()

    def stop(self):
        if self._client.is_alive():
            self._client.stop()
            self._client.join()
 
    def process_cones_message(self, cone_msg):
        cone_map = messages.perception.ConeMap()
        cone_msg.data.Unpack(cone_map)

        formula_state = messages.state_est.FormulaState()

        print(f"State got cone message ID {cone_msg.header.id} with {len(cone_map.cones)} cones in the queue")
        for cone in cone_map.cones:
            state_cone = messages.state_est.StateCone()
            state_cone.cone_id = self._running_id
            self._running_id += 1  # just a mock, later we need to indentify cones that already exist in our map.
            #Here Theta and r are self coordinates. Meaning from the perspective of the car:
            state_cone.r = math.sqrt(math.pow(cone.x ,2) + math.pow( cone.y ,2) )
            state_cone.theta =  math.atan2( cone.y , cone.x)
            state_cone.type =  cone.type
            if cone.type == messages.perception.Orange:
                self._distance_to_finish = state_cone.r
            formula_state.state_cones.append(state_cone)

        # Update distance to finish if we've seen the orange cones:
        formula_state.distance_to_finish = self._distance_to_finish

        #Order Cones:
        orderCones(formula_state.state_cones) 

        #Calculate middle of the road:
        # formula_state.angle_from_road_center =
        # formula_state.distance_from_road_center =
        
        return formula_state 


        

    def process_gps_message(self):
        try:
            gps_msg = self._client.get_gps_message(timeout=self.message_timeout)
            gps_data = messages.sensors.GPSSensor()
            gps_msg.data.Unpack(gps_data)
            print(f"got gps: x: {gps_data.position.x} y: {gps_data.position.y} z: {gps_data.position.z}")
        except:
            pass

    def process_imu_message(self):
        try:
            imu_msg = self._client.get_imu_message(timeout=self.message_timeout)
            imu_data = messages.sensors.IMUSensor()
            imu_msg.data.Unpack(imu_data)
        except:
            pass

    def process_server_message(self, server_messages):
        if server_messages.data.Is(messages.server.ExitMessage.DESCRIPTOR):
            return True

        return False

    def send_message2control(self, msg_id, formula_state):
        msg = messages.common.Message()
        msg.header.id = msg_id
        msg.data.Pack(formula_state)
        self._client.send_message(msg)

    def run(self):
        while True:
            try:
                server_msg = self._client.pop_server_message()
                if server_msg is not None:
                    if self.process_server_message(server_msg):
                        return
            except Exception as e:
                pass
                
            self.process_gps_message()
            self.process_imu_message()

            try:
                cone_msg = self._client.get_cone_message(timeout=self.message_timeout)
                formula_state = self.process_cones_message(cone_msg)
                self.send_message2control(cone_msg.header.id, formula_state)    
            except Exception as e:
                pass

state = State()

def stop_all_threads():
    print("Stopping threads")
    state.stop()

def shutdown(a, b):
    print("Shutdown was called")
    stop_all_threads()
    exit(0)


def main():
    print("Initalized State")

    state.start()
    state.run()

    stop_all_threads()
    exit(0)

if __name__ == "__main__":
    for signame in ('SIGINT', 'SIGTERM'):
        signal.signal(getattr(signal, signame), shutdown)
    main()



  