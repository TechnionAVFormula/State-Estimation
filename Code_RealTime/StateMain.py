# Client :
from SystemRunnerPart.StateEstClient import StateEstClient


# our class_defs and functions:
from class_defs.StateEst_CarState import CarState
from OrderCones.orderConesMain    import orderCones
# from class_defs.Cone import Cone
from KalmanFilter.EKF_Slam_Class import Kalman
from ConeMapping.ConeMapMain import ConeMap
from class_defs.StateEstCompPlot import CompPlot

## import depanding on running state / configuration state:
from config import CONFIG , ConfigEnum , IS_DEBUG_MODE

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
    from pyFormulaClient.MessageDeque import NoFormulaMessages
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import messages
    from pyFormulaClientNoNvidia.MessageDeque import NoFormulaMessages
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
import numpy as np


class State:
    def __init__(self):
        #DEBUG:
        self.is_debug_mode = IS_DEBUG_MODE
        self.is_kalman_filter = False
        self.is_cone_clusttering = True
        self.is_compare2ground_truth = True
        self.is_plot_state = True
        #client:
        self._client = StateEstClient()
        self._message_timeout = 0.01
        #EKF:
        self._car_state = CarState()

        if self.is_kalman_filter:
            self._kalman_filter = Kalman()

        #cone map:
        if self.is_cone_clusttering:
            self._cone_map = ConeMap()
        else:
            self._cone_map =  np.array([] )
            self._running_id = 1  

        self._ordered_cones = { "left" : np.array([] ) ,
                                "right": np.array([] ) }

        if self.is_compare2ground_truth:
            self._ground_truth_memory = np.array([])
            self._cone_truth = np.array([])
        
        if self.is_plot_state:
            self._comp_plot = CompPlot()
            


    def start(self):
        self._client.connect(1)
        if CONFIG == ConfigEnum.LOCAL_TEST:
            self._client.set_read_delay(0.01)
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

        if self.is_cone_clusttering:
            temp_cone_arr = np.array([] , dtype=Cone)
        #Analize all cones for position in map and other basic elements: 
        for perception_cone in cone_map.cones:
            state_cone = self.cone_convert_perception2our_cone(perception_cone)

            if self.is_cone_clusttering:
                new_elem = np.array([state_cone] )
                temp_cone_arr = np.append(temp_cone_arr , new_elem) 
            else:
                #if it's a new cone in our map, add it:
                if (self.check_exist_cone(state_cone) == False ):
                    self.add_new_cone(state_cone)  

        if self.is_cone_clusttering:
            self._cone_map.insert_new_points( temp_cone_arr )
        #Order Cones:
        self._ordered_cones.blue_cones , self._ordered_cones.yellow_cones = orderCones( self._cone_map , self._car_state ) 
		#ariela:now the cones are ordered

    def process_gps_message(self , gps_data):
        if self.is_kalman_filter:
            pass
        else:
            self._car_state.x = gps_data.position.x         
            self._car_state.y = gps_data.position.y

    def process_ground_truth_message_memory(self , gt_msg):
        #unpack data and time:
        gt_data = messages.ground_truth.GroundTruth()
        gt_msg.data.Unpack(gt_data)
        time_in_milisec = gt_msg.header.timestamp.ToMilliseconds()
        
        #Process Car States:
        car_turth = {}
        car_turth["time_in_milisec"] = time_in_milisec
        if gt_data.has_position_truth:
            car_turth["x"] = gt_data.position.x
            car_turth["y"] = gt_data.position.y
        if gt_data.has_car_measurments_truth:
            car_turth["delta"] = gt_data.car_measurments.steering_angle
        if gt_data.has_imu_measurments_truth:
            car_turth["speed"] = gt_data.imu_measurments.speed
            car_turth["theta"] = gt_data.imu_measurments.orientation.z

        if (self._cone_truth.size == 0): #Check no cones
            for cone in gt_data.cones:
                tmp_cone = {"x": cone.position.x ,
                            "y": cone.position.y ,
                            "type" : cone.type   }
                self._cone_truth = np.append(self._cone_truth , tmp_cone)
            if self.is_plot_state:
                self._comp_plot.plot_cones(self._cone_truth)

        if self.is_compare2ground_truth:
            self._ground_truth_memory = np.append( self._ground_truth_memory , car_turth )
            

        if self.is_plot_state:
            self._comp_plot.update_car_state(car_turth)
            
        
        
            


    def process_car_data_message(self , car_data):
        delta = car_data.car_measurments.steering_angle
        acc = car_data.imu_sensor.imu_measurments.acceleration.x
        if self.is_kalman_filter:
            pass
        else:
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
        data.current_state.theta      = self._car_state.theta
        
        # finish estimation:
        data.distance_to_finish , is_found = self.calc_distance_to_finish()
        # if ( is_found ) and ( data.distance_to_finish < 0 ):
        #     data.is_finished = True
        # else:
        #     data.is_finished = False

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

    
    def act_on_no_message(self , source_str):
        if self.is_debug_mode:
            print(f"No Message from {source_str}")

    # =============================================== Run: =============================================== #
    def run(self):
        while True:

            ## Server:
            try:
                server_msg = self._client.pop_server_message()
                if server_msg is not None:
                    if self.process_server_message(server_msg):
                        return
            except NoFormulaMessages:
                self.act_on_no_message('server')
            except Exception as e:
                print(f"StateMain::Exception: {e}")
            
            ## GPS:
            try:
                gps_msg = self._client.get_gps_message(timeout=self._message_timeout)
                gps_data = messages.sensors.GPSSensor()
                gps_msg.data.Unpack(gps_data)
                if self.is_debug_mode :
                    print(f"got gps: x: {gps_data.position.x} y: {gps_data.position.y} z: {gps_data.position.z}")
                self.process_gps_message(gps_data)                
                self.send_message2control(gps_msg)
            except NoFormulaMessages:
                self.act_on_no_message('GPS') 
            except Exception as e:
                print(f"StateMain::Exception: {e}")
    
            ## car data::
            try:
                car_data_msg = self._client.get_car_data_message(timeout=self._message_timeout)
                car_data_data = messages.sensors.CarData()
                car_data_msg.data.Unpack(car_data_data)
                self.process_car_data_message(car_data_data)
                self.send_message2control(car_data_data)
            except NoFormulaMessages:
                self.act_on_no_message('car data')
            except Exception as e:
                print(f"StateMain::Exception: {e}")

            ## Perception:
            try:
                cone_msg = self._client.get_cone_message(timeout=self._message_timeout)                   
                cone_map = messages.perception.ConeMap()
                cone_msg.data.Unpack(cone_map)  
                if self.is_debug_mode:
                    print(f"State got cone message ID {cone_msg.header.id} with {len(cone_map.cones)} cones in the queue")
                self.process_cones_message(cone_map)
                self.send_message2control(cone_msg)
            except NoFormulaMessages:
                self.act_on_no_message('cone map')
            except Exception as e:
                print(f"StateMain::Exception: {e}")

            ## Ground Truth:
            try:
                ground_truth_msg = self._client.get_ground_truth_message(timeout=self._message_timeout)
                self.process_ground_truth_message_memory(ground_truth_msg)
                #No need to send message2control
            except NoFormulaMessages:
                self.act_on_no_message('ground truth')
            except Exception as e:
                print(f"StateMain::Exception: {e}")
    # =============================================== Run: =============================================== #
            
    #end run(self)
'''
End of class
'''

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

  