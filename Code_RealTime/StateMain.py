# Client :
from SystemRunnerPart.StateEstClient import StateEstClient


## import depanding on running state / configuration state:
from config import CONFIG, ConfigEnum, IS_DEBUG_MODE , IS_TIME_CODE_WITH_TIMER , IS_CONE_MAP_WITH_CLUSTERING , SHOW_REALTIME_DASHBOARD

if (CONFIG == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
    from pyFormulaClient.MessageDeque import NoFormulaMessages
elif CONFIG == ConfigEnum.LOCAL_TEST:
    from pyFormulaClientNoNvidia import messages
    from pyFormulaClientNoNvidia.MessageDeque import NoFormulaMessages
else:
    raise NameError("User Should Choose Configuration from config.py")


# Import our precious sub-functions and objects:
from OrderCones.orderConesMain import orderCones  # for path planning
from KalmanFilter.EKF_Slam_Class import Kalman  # For smart Localization using a kalman filter
from class_defs.GPSOneShot import GPSOneShot


# ConeMap:
if IS_CONE_MAP_WITH_CLUSTERING:
    from ConeMapping.ConeMap_CumulativeClustering import ConeMap_CumulativeClustering as ConeMap
else:
    from ConeMapping.ConeMap_Naive import ConeMap_Naive as ConeMap

# Plot and Visualizations:
from class_defs.StateEstCompPlot import CompPlot  # for plotting
import SystemRunnerPart.StateEst_Dash as StateEst_DashBoard




# for showing messages:
from pprint import pprint
import google.protobuf.json_format as proto_format

# from SystemRunnerPart.print_messages_file import print_messages_file
from SystemRunnerPart.print_messages_file import save_as_json
import json


# typical python stuff:
import math
import time
import signal
import numpy as np

from enum import Enum, auto

if IS_TIME_CODE_WITH_TIMER:
    from timeit import default_timer as timer


YELLOW = messages.perception.Yellow
BLUE = messages.perception.Blue
ORANGE_BIG = messages.perception.OrangeBig
ORANGE_SMALL = messages.perception.OrangeSmall


class State:
    def __init__(self):
        # DEBUG:
        self.is_kalman_filter = True
        self.is_compare2ground_truth = False
        self.is_matplotlib = False  # Reduces running speed sagnificantly when True
        self.is_plotly = SHOW_REALTIME_DASHBOARD
        self.is_order_cones = True
        # client:
        self._client = StateEstClient()
        self._message_timeout = 0.0001
        # Cone map:
        self._cone_map = ConeMap()  # simple version
        self._ordered_cones = {"left": np.array([]), "right": np.array([])}
        # Localization of car (Kalman Filter):
        self._car_state = (
            messages.state_est.CarState()
        )  # keeping our most recent known state here
        if self.is_kalman_filter:
            self._GPSOneShot = GPSOneShot()
            self._last_kalman_time_milisec = None
            self._kalman_filter = Kalman()
            self._car_state_predicted = messages.state_est.CarState()  # prediction only

        if self.is_compare2ground_truth:
            self._ground_truth_memory = np.array([])
            self._cone_truth = np.array([])

        if self.is_matplotlib:
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

    def cone_convert_perception2StateCone(self, perception_cone):
        state_cone = messages.state_est.StateCone()
        state_cone.type = perception_cone.type
        # Here Theta and r are self coordinates. Meaning from the perspective of the car:
        state_cone.r = math.sqrt(
            math.pow(perception_cone.x, 2) + math.pow(perception_cone.y, 2)
        )
        # Normally, one will type  atan(y,x)...
        # but Perception looks at the world where x is positive towards the right side of the car, and y forward.
        state_cone.alpha = math.atan2(perception_cone.y, perception_cone.x)

        ## convert to our xNorth yEast:
        theta_total = (
            self._car_state.theta + state_cone.alpha
        )  #! here we depand on _car_state being recent and relevant
        # distance of cone from car, in our axis
        delta_x = state_cone.r * math.cos(theta_total)
        delta_y = state_cone.r * math.sin(theta_total)
        # add distance to cars position
        state_cone.position.x = self._car_state.position.x + delta_x
        state_cone.position.y = self._car_state.position.y + delta_y
        # We known nothing (John Snow) about the cone:
        state_cone.position_deviation = math.inf
        return state_cone

    def cone_convert_from_ordered2state_cone(self, cone):
        state_cone = cone
        """
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
        """
        return state_cone

    def process_cones_message(self, cone_msg):
        """Parse Data and time"""
        cone_map = messages.perception.ConeMap()
        cone_msg.data.Unpack(cone_map)
        if IS_DEBUG_MODE:
            print(f"State got cone message ID {cone_msg.header.id} with {len(cone_map.cones)} cones in the queue")

        '''ConeMap: '''
        if IS_TIME_CODE_WITH_TIMER:
            cluster_start = timer()

        cone_array = np.array([])
        # Analize all cones for position in map and other basic elements:
        for perception_cone in cone_map.cones:
            state_cone = self.cone_convert_perception2StateCone(perception_cone)
            cone_array = np.append(cone_array, state_cone)
        self._cone_map.insert_new_points(cone_array)
        real_cones = self._cone_map.get_all_samples() 

        if IS_TIME_CODE_WITH_TIMER:
            print(f"clustering took {timer() - cluster_start} ms")

        ''''Order Cones: '''
        if IS_TIME_CODE_WITH_TIMER:
            order_start = timer()

        if self.is_order_cones:
            self._ordered_cones["left"], self._ordered_cones["right"] = orderCones( real_cones  , self._car_state)

        if IS_TIME_CODE_WITH_TIMER:
            print(f"ordering took {timer() - order_start} ms")


    def process_gps_message(self, gps_msg):
        """unpack data and time:"""
        gps_data = messages.sensors.GPSSensor()
        gps_msg.data.Unpack(gps_data)
        x = gps_data.position.x
        y = gps_data.position.y
        # z = gps_data.position.z
        time_in_milisec = gps_msg.header.timestamp.ToMilliseconds()

        """Process:"""
        if IS_DEBUG_MODE:
            print(f"got gps: x: {x:6.2f} y: {gps_data.position.y:6.2f} ")

        if self.is_kalman_filter:
            self._GPSOneShot.set_new_data(x, y, time_in_milisec)

        else:
            self._car_state.position.x = x
            self._car_state.position.y = y

    def process_ground_truth_message_memory(self, gt_msg):

        if not self.is_compare2ground_truth:
            return

        """unpack data and time:"""
        gt_data = messages.ground_truth.GroundTruth()
        gt_msg.data.Unpack(gt_data)
        time_in_milisec = gt_msg.header.timestamp.ToMilliseconds()

        # Process Car States:
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

        if self._cone_truth.size == 0:  # Check no cones
            for cone in gt_data.cones:
                tmp_cone = {
                    "x": cone.position.x,
                    "y": cone.position.y,
                    "type": cone.type,
                }
                self._cone_truth = np.append(self._cone_truth, tmp_cone)
            
            if self.is_matplotlib:

                self._comp_plot.plot_cones(self._cone_truth)

        if self.is_compare2ground_truth:
            self._ground_truth_memory = np.append(self._ground_truth_memory, car_turth)

        if self.is_matplotlib:
            self._comp_plot.update_car_state(car_turth)

    def check_correct_frequency(self, delta_t_milisec):
        return True

    def process_car_data_message(self, car_data_msg):
        """ unpack data and time: """
        car_data = messages.sensors.CarData()
        car_data_msg.data.Unpack(car_data)
        time_in_milisec = car_data_msg.header.timestamp.ToMilliseconds()

        """ Assert that frequency of update is correct: """
        if self._last_kalman_time_milisec == None:  #first timee we're here
            self._last_kalman_time_milisec = time_in_milisec
            return
        else:
            delta_t_milisec = (time_in_milisec - self._last_kalman_time_milisec)  # calc time since last prediction and update new time:
            if not self.check_correct_frequency(delta_t_milisec):
                return
            self._last_kalman_time_milisec = time_in_milisec


        if self.is_kalman_filter:
            
            """ Prediction: """
            delta = car_data.car_measurments.steering_angle  # steering angle
            acceleration_long = car_data.imu_sensor.imu_measurments.acceleration.x
            acceleration_lat = car_data.imu_sensor.imu_measurments.acceleration.y
            # some times this data exists:
            Vx = car_data.imu_sensor.imu_measurments.velocity.x
            Vy = car_data.imu_sensor.imu_measurments.velocity.y
            theta = car_data.imu_sensor.imu_measurments.orientation.z

            data_for_prediction = {
                "steering_angle": delta,  #
                "delta_t_milisec": delta_t_milisec,
                "acceleration_long": acceleration_long,
                "acceleration_lat": acceleration_lat,
            }

            self._kalman_filter.State_Prediction(data_for_prediction)

            ''' Correction: '''

            """ ! This should be changed ! """
            data_for_correction = data_for_prediction
            if self._GPSOneShot.check_new_data():
                x, y = self._GPSOneShot.get_data()
                is_exist_GPS = True
            else:
                x = None
                y = None
                is_exist_GPS = False

            data_for_correction["gyro"] = car_data.imu_sensor.imu_measurments.angular_velocity.z   
            data_for_correction["is_exist_GPS"] = is_exist_GPS
            data_for_correction["GPS_x"] = x
            data_for_correction["GPS_y"] = y

            self._kalman_filter.State_Correction(data_for_correction)

            self._car_state = self._kalman_filter.Get_Current_State()

        else:
            # Save Velocity:
            self._car_state.velocity.x = Vx
            self._car_state.velocity.y = Vy
            # Save Orientation:
            self._car_state.theta = theta

    def process_server_message(self, server_messages):
        if server_messages.data.Is(messages.server.ExitMessage.DESCRIPTOR):
            return True

        return False

    def _calc_distance_to_finish(self):
        # if len(self._ordered_cones.orange_cones) == 0 : #not seen any orange cones yet
        if True:
            dist = -1
            is_found = False
        else:
            dist = self._ordered_cones.orange_cones[0]  # take closest orange cone
            is_found = True
        return dist, is_found

    def create_formula_state_msg(self):
        # Makes a data object according to the formula msg proto "FormulaState"
        # With the updated state

        # create an empty message of state_est data:
        data = messages.state_est.FormulaState()

        data.current_state.position.x            = self._car_state.position.x           
        data.current_state.position.y            = self._car_state.position.y           
        data.current_state.position_deviation.x  = self._car_state.position_deviation.x 
        data.current_state.position_deviation.y  = self._car_state.position_deviation.y 
        data.current_state.velocity.x            = self._car_state.velocity.x           
        data.current_state.velocity.y            = self._car_state.velocity.y           
        data.current_state.velocity_deviation.x  = self._car_state.velocity_deviation.x 
        data.current_state.velocity_deviation.y  = self._car_state.velocity_deviation.y 
        data.current_state.theta                 = self._car_state.theta                
        data.current_state.theta_deviation       = self._car_state.theta_deviation      
        data.current_state.theta_dot             = self._car_state.theta_dot            
        data.current_state.theta_dot_deviation   = self._car_state.theta_dot_deviation 

        # finish estimation:
        data.distance_to_finish, is_found = self._calc_distance_to_finish()

        # Cones:
        if self.is_order_cones:
            for cone in self._ordered_cones["right"]:
                state_cone = self.cone_convert_from_ordered2state_cone(cone)
                data.right_bound_cones.append(state_cone)
            for cone in self._ordered_cones["left"]:
                state_cone = self.cone_convert_from_ordered2state_cone(cone)
                data.left_bound_cones.append(state_cone)
        else:
            for state_cone in self._cone_map.get_all_samples():
                if state_cone.type == YELLOW:
                    data.right_bound_cones.append(state_cone)
                if state_cone.type == BLUE:
                    data.left_bound_cones.append(state_cone)

        """ Missing road estimation:"""
        # distance_from_left  #= 6;
        # distance_from_right #= 7;
        # road_angle          #= 8; // direction of road . absolute in the coordinate system of xNorth yEast

        return data

    def send_message2control(self, msg_in):
        # make an empty message:
        msg_id = msg_in.header.id
        msg_out = messages.common.Message()
        msg_out.header.id = msg_id

        # summarize all the data:
        data = self.create_formula_state_msg()

        # print message for debugging:
        if IS_DEBUG_MODE:
            print_proto_message(data)

        # send message:
        msg_out.data.Pack(data)
        self._client.send_message(msg_out)

        # save_as_json(msg_out)

        ## send data to dash-board
        if self.is_plotly:
            if self.is_compare2ground_truth and ( len(self._ground_truth_memory) > 0 ) :
                StateEst_DashBoard.send_StateEst_DashBoard_with_GroundTruth(msg_out , self._ground_truth_memory[-1])
            else:
                StateEst_DashBoard.send_StateEst_DashBoard_msg(msg_out)

    def act_on_no_message(self, source_str):
        if IS_DEBUG_MODE:
            print(f"No {source_str:10} message")

    # V===============================================V Run: V===============================================V #
    def run(self):
        while True:

            ## Server:
            try:
                server_msg = self._client.pop_server_message()
                if server_msg is not None:
                    if self.process_server_message(server_msg):
                        return
            except NoFormulaMessages:
                self.act_on_no_message("server")
            except Exception as e:
                print(f"StateMain::Exception: {e}")

            ## GPS:
            try:
                gps_msg = self._client.get_gps_message(timeout=self._message_timeout)
                self.process_gps_message(gps_msg)
                self.send_message2control(gps_msg)
            except NoFormulaMessages:
                self.act_on_no_message("GPS")
            except Exception as e:
                print(f"StateMain::Exception: {e}")

            ## car data::
            try:
                car_data_msg = self._client.get_car_data_message( timeout=self._message_timeout)
                self.process_car_data_message(car_data_msg)
                self.send_message2control(car_data_msg)
            except NoFormulaMessages:
                self.act_on_no_message("car data")
            except Exception as e:
                print(f"StateMain::Exception: {e}")

            ## Perception:
            try:
                cone_msg = self._client.get_cone_message(timeout=self._message_timeout)
                self.process_cones_message(cone_msg)
                self.send_message2control(cone_msg)
            except NoFormulaMessages:
                self.act_on_no_message("cone map")
            except Exception as e:
                print(f"StateMain::Exception: {e}")

            ## Ground Truth:
            try:
                ground_truth_msg = self._client.get_ground_truth_message(timeout=self._message_timeout)
                self.process_ground_truth_message_memory(ground_truth_msg)
                # No need to send message2control
            except NoFormulaMessages:
                self.act_on_no_message("ground truth")
            except Exception as e:
                print(f"StateMain::Exception: {e}")

    # ^===============================================^ Run: ^===============================================^ #

    # end run(self)


"""
End of class
"""

state = State()


def stop_all_threads():
    print("Stopping threads")
    state.stop()


def shutdown(a, b):
    print("Shutdown was called")
    stop_all_threads()
    exit(0)


def print_proto_message(data):
    # print message
    msg_dict = proto_format.MessageToDict(data, including_default_value_fields=True, preserving_proto_field_name=True)
    print(json.dumps(msg_dict, indent=2))


def main():
    print("Initalized State")
    state.start()
    state.run()

    stop_all_threads()
    exit(0)


if __name__ == "__main__":
    for signame in ("SIGINT", "SIGTERM"):
        signal.signal(getattr(signal, signame), shutdown)
    main()
