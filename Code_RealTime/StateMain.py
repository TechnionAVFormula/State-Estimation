from SystemRunnerPart.StateEstClient import StateEstClient

# our class_defs and functions:
from class_defs.StateEst_CarState import CarState
from class_defs.OrderedConesClass import OrderedCones
from OrderCones.orderConesMain    import orderCones
from class_defs.Cone import Cone

## import depanding on running state / configuration state:
from config import CONFIG
from config import ConfigEnum

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import messages
else:
    raise NameError('User Should Choose Configuration from config.py')



# for showing messages:
from pprint import pprint 
import google.protobuf.json_format as proto_format
# from SystemRunnerPart.print_messages_file import print_messages_file 
import json



#typical python stuff:
import math
import time
import signal
import sys
import numpy as np


class State:
    def __init__(self):
        #DEBUG:
        self.is_debug_mode = True
        #client:
        self._client = StateEstClient()
        self._message_timeout = 0.01
        #EKF:
        self._car_state = CarState()
        #cone map:
        self._cone_map =  np.array([] , dtype=Cone )
        self._ordered_cones = OrderedCones()
        self._running_id = 1             
        


    def start(self):
        self._client.connect(1)
        self._client.set_read_delay(0.1)
        self._client.start()

    def stop(self):
        if self._client.is_alive():
            self._client.stop()
            self._client.join()

    def cone_convert_perception2our_cone(self , perception_cone):
        cone = Cone()
        cone.type =  perception_cone.type
        #Here Theta and r are self coordinates. Meaning from the perspective of the car:
        cone.r = math.sqrt(math.pow(perception_cone.x ,2) + math.pow( perception_cone.y ,2) )
        #Normally, one will type  atan(y,x)...
        #but Perception looks at the world where x is positive towards the right side of the car, and y forward.
        cone.theta =  math.atan2( perception_cone.y , perception_cone.x)
        
        ## convert to our xNorth yEast:
        theta_total = self._car_state.theta + cone.theta
        #distance of cone from car, in our axis
        delta_x = cone.r*math.cos(theta_total)
        delta_y = cone.r*math.sin(theta_total)
        #add distance to cars position
        cone.x = self._car_state.x + delta_x
        cone.y = self._car_state.y + delta_y
        return cone


    def cone_convert_state2msg(self , cone):
        state_cone = messages.state_est.StateCone()
        # things that we already have:
        state_cone.type =  cone.type
        state_cone.cone_id =  cone.cone_id
        state_cone.position.x = cone.x
        state_cone.position.y = cone.y
        # Translate theta and are according to current car position and orientation

        delta_x = cone.x - self._car_state.x
        delta_y = cone.y - self._car_state.y
        state_cone.r = math.sqrt(math.pow(  delta_x ,2) + math.pow( delta_y ,2) )
        
        total_theta = math.atan2( delta_y , delta_x  ) 
        state_cone.theta =   total_theta - self._car_state.theta

        return state_cone


    def check_exist_cone(self, new_cone):
        epsilon = 1 # 1 meter of error around cone
        x_new = new_cone.x
        y_new = new_cone.y

        for old_cone in self._cone_map:
            x_old = old_cone.x
            y_old = old_cone.y
            seperation =  math.sqrt(math.pow(   x_new - x_old  ,2) + math.pow(    y_new - y_old   ,2) )       
            if seperation < epsilon:
                return True #exist
        #if we came here, no existing cone match one of the existing cones                
        return False #not exist        


    def add_new_cone(self , state_cone):
        state_cone.cone_id = self._running_id
        self._running_id += 1  # just a mock, later we need to indentify cones that already exist in our map.
        # Add cone to array:
        new_elem = np.array([state_cone] )
        self._cone_map = np.append(self._cone_map , new_elem) 


    def process_cones_message(self, cone_map):     
        #Analize all cones for position in map and other basic elements: 
        for perception_cone in cone_map.cones:
            state_cone = self.cone_convert_perception2our_cone(perception_cone)

            #if it's a new cone in our map, add it:
            if (self.check_exist_cone(state_cone) == False ):
                self.add_new_cone(state_cone)   
        #Order Cones:
        self._ordered_cones.blue_cones , self._ordered_cones.yellow_cones = orderCones( self._cone_map , self._car_state ) 
		#ariela:now the cones are ordered

    def process_gps_message(self , gps_data):
        self._car_state.x = gps_data.position.x         
        self._car_state.y = gps_data.position.y



    def process_imu_message(self , imu_data):
        # Save Velocity:
        self._car_state.Vx = imu_data.velocity.x
        self._car_state.Vy = imu_data.velocity.y
        # Save Orientation:
        self._car_state.theta = imu_data.orientation.z

        if self.is_debug_mode:
            print_proto_message(imu_data)

    def process_server_message(self, server_messages):
        if server_messages.data.Is(messages.server.ExitMessage.DESCRIPTOR):
            return True

        return False
    
    def calc_distance_to_finish(self):
        if len(self._ordered_cones.orange_cones) == 0 : #not seen any orange cones yet
            dist = -1
            is_found = False
        else:
            dist = self._ordered_cones.orange_cones[0]  #take closest orange cone
            is_found = True
        return dist , is_found    

    
    def formula_state_msg(self):
        # Makes a data object according to the formula msg proto "FormulaState"
        # With the updated state         
        
        #create an empty message:
        data = messages.state_est.FormulaState()
        
        # Car's position:
        data.current_state.position.x = self._car_state.x 
        data.current_state.position.y = self._car_state.y 
        data.current_state.velocity.x = self._car_state.Vx
        data.current_state.velocity.y = self._car_state.Vy
        data.current_state.theta_absolute = self._car_state.theta
        
        # finish estimation:
        data.distance_to_finish , is_found = self.calc_distance_to_finish()
        if ( is_found ) and ( data.distance_to_finish < 0 ):
            data.is_finished = True
        else:
            data.is_finished = False

        # Cones:    
        for cone in self._ordered_cones.yellow_cones:
            state_cone = self.cone_convert_state2msg(cone)
            data.right_bound_cones.append(state_cone)    
        for cone in self._ordered_cones.blue_cones:
            state_cone = self.cone_convert_state2msg(cone)
            data.left_bound_cones.append(state_cone)
        # data.right_bound_cones = self._ordered_cones.yellow_cones
        # data.left_bound_cones  = self._ordered_cones.blue_cones
 
        return data
    


    def send_message2control(self, msg_in):
        msg_id = msg_in.header.id
        msg_out = messages.common.Message()
        msg_out.header.id = msg_id

        # Make the message:
        data = self.formula_state_msg()
        if self.is_debug_mode:
            print_proto_message(data)

        msg_out.data.Pack(data)
        self._client.send_message(msg_out)


    # =============================================== Run: =============================================== #
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
                gps_msg = self._client.get_gps_message(timeout=self._message_timeout)
                gps_data = messages.sensors.GPSSensor()
                gps_msg.data.Unpack(gps_data)
                if self.is_debug_mode :
                    print(f"got gps: x: {gps_data.position.x} y: {gps_data.position.y} z: {gps_data.position.z}")
                self.process_gps_message(gps_data)                
                self.send_message2control(gps_msg) 
            except Exception as e:
                pass
    
            ## IMU:
            try:
                imu_msg = self._client.get_imu_message(timeout=self._message_timeout)
                imu_data = messages.sensors.IMUSensor()
                imu_msg.data.Unpack(imu_data)
                self.process_imu_message(imu_data)
                self.send_message2control(imu_msg) 
            except Exception as e:
                pass

            ## Perception:
            try:
                cone_msg = self._client.get_cone_message(timeout=self._message_timeout)                   
                cone_map = messages.perception.ConeMap()
                cone_msg.data.Unpack(cone_map)  
                if self.is_debug_mode:
                    print(f"State got cone message ID {cone_msg.header.id} with {len(cone_map.cones)} cones in the queue")
                self.process_cones_message(cone_map)
                self.send_message2control(cone_msg) 
            except Exception as e:
                pass
    # =============================================== Run: =============================================== #
            
    #end run(self)


state = State()

def stop_all_threads():
    print("Stopping threads")
    state.stop()

def shutdown(a, b):
    print("Shutdown was called")
    stop_all_threads()
    exit(0)


def print_proto_message(data):
    #print message
    msg_dict=proto_format.MessageToDict(data,   including_default_value_fields=True,
                                        preserving_proto_field_name=True)
    print(json.dumps(msg_dict,indent=2) ) 


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

  