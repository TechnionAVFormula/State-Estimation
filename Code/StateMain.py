from .SystemRunnerPart.StateEstClient import StateEstClient
from pyFormulaClientNoNvidia import messages
import time
import signal
import sys
import math

from .OrderCones.orderConesMain import orderCones

class CarState:
    x = 0
    y = 1
    Vx = 2
    Vy = 3
    theta = 0.4
    

class State:
    def __init__(self):
        #client stuff:
        self._client = StateEstClient('perception.messages', 'state.messages')
        self.message_timeout = 0.01
        #EKF:
        self._car_state = CarState()
        #cone map:
        self._cone_map = []
        self._running_id = 1
        self._distance_to_finish = -1                
        #msc:
        self._msg2control = messages.state_est.FormulaState()


    def start(self):
        self._client.connect(1)
        self._client.set_read_delay(0.1)
        self._client.start()

    def stop(self):
        if self._client.is_alive():
            self._client.stop()
            self._client.join()

    def cone_convert_perception2state(self , perception_cone):
        state_cone = messages.state_est.StateCone()
        state_cone.type =  perception_cone.type
        #Here Theta and r are self coordinates. Meaning from the perspective of the car:
        state_cone.r = math.sqrt(math.pow(perception_cone.x ,2) + math.pow( perception_cone.y ,2) )
        #Normally, one will type  atan(y,x)...
        #but Perception looks at the world where x is positive towards the right side of the car, and y forward.
        state_cone.theta =  math.atan2( perception_cone.y , perception_cone.x)
        
        ## convert to our xNorth yEast:
        theta_total = self._car_state.theta + state_cone.theta
        #distance of cone from car, in our axis
        delta_x = r*math.cos(theta_total)
        delta_y = r*math.sin(theta_total)
        #add distance to cars position
        state_cone.position[0] = self._car_state.x + delta_x
        state_cone.position[0] = self._car_state.y + delta_y
        return state_cone


    def check_exist_cone(self, new_cone):
        epsilon = 1 # 1 meter of error around cone
        x_new = new_cone.position[0]
        y_new = new_cone.position[1]

        for old_cone in self.cone_map:
            x_old = old_cone.position[0]
            y_old = old_cone.position[1]
            seperation =  math.sqrt(math.pow(   x_new - x_old  ,2) + math.pow(    y_new - y_old   ,2) )       
            if seperation < epsilon:
                return True #exist
        #if we came here, no existing cone match one of the existing cones                
        return False #not exist        


    def add_new_cone(self , state_cone):
        state_cone.cone_id = self._running_id
        self._running_id += 1  # just a mock, later we need to indentify cones that already exist in our map.
        self._cone_map.append(state_cone)


    def process_cones_message(self, cone_map):     
        #Analize all cones for position in map and other basic elements: 
        for perception_cone in cone_map.cones:
            state_cone = self.cone_convert_perception2state(perception_cone)

            #if it's a new cone in our map, add it:
            if self.check_exist_cone(state_cone):
                self.add_new_cone(state_cone)   
        #Order Cones:
        self._msg2control.left_bound_cones , self._msg2control.right_bound_cones  = orderCones( self._cone_map , self._car_state ) 



        

    def process_gps_message(self):
        try:
            gps_data = messages.sensors.GPSSensor()
            gps_msg.data.Unpack(gps_data)
            print(f"got gps: x: {gps_data.position.x} y: {gps_data.position.y} z: {gps_data.position.z}")
        except:
            pass

    def process_imu_message(self):
        try:
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

            ## Server:
            try:
                server_msg = self._client.pop_server_message()
                if server_msg is not None:
                    if self.process_server_message(server_msg):
                        return
            except Exception as e:
                pass
            
            ## GPS:
            try:
                gps_msg = self._client.get_gps_message(timeout=self.message_timeout)
                formula_state = self.process_gps_message(gps_msg)                
                self.send_message2control(cone_msg.header.id, formula_state) 
            except Exception as e:
                pass
    
            ## IMU:
            try:
                imu_msg = self._client.get_imu_message(timeout=self.message_timeout)
                formula_state = self.process_imu_message(gps_msg)
                self.send_message2control(cone_msg.header.id, formula_state) 
            except Exception as e:
                pass

            ## Perception:
            try:
                cone_msg = self._client.get_cone_message(timeout=self.message_timeout)                   
                cone_map = messages.perception.ConeMap()
                cone_msg.data.Unpack(cone_map)  

                print(f"State got cone message ID {cone_msg.header.id} with {len(cone_map.cones)} cones in the queue")
                formula_state = self.process_cones_message(cone_map)
                self.send_message2control(cone_msg.header.id, formula_state) 
            except Exception as e:
                pass
            
            
            
    #end run(self)


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



  