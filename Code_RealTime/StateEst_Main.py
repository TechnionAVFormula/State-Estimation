## Flags and Enums and stuff:
IS_PRINT_ON_NO_MSG = False
IS_KALMAN_FILTER = True
IS_COMPARE_GROUND_TRUTH = True


## For relative imports:
import sys, os, pathlib
currentPath = pathlib.Path(os.path.dirname(__file__))
relativePath = currentPath.parent
sys.path.append(str(relativePath))

## classes and enums from our utilities:
from StateEst_Utils.config import CONFIG, IS_DEBUG_MODE , IS_TIME_CODE_WITH_TIMER , IS_CONE_MAP_WITH_CLUSTERING  , IS_PRINT_OUTPUT_MSG 
from StateEst_Utils.MessagesClass import messages, NoFormulaMessages
from StateEst_Utils.ConfigEnum import ConfigEnum
from StateEst_Utils.ConeType import ConeType

# Client :
from StateEst_SR.StateEst_Client import StateEstClient

# Our imports:
from OrderCones.orderConesMain import orderCones  # for path planning
from KalmanFilter.EKF_Slam_Class import Kalman as KalmanFilter # For smart Localization using a kalman filter
from StateEst_SR.GPSOneShot import GPSOneShot
from StateEst_SR.Logger import InitLogger
from StateEst_SR.StateEst_CalibrationVerifier import CalibrationVerifier 
from StateEst_SR.KalmanFilterManager import KalmanManager

# ConeMap:
if IS_CONE_MAP_WITH_CLUSTERING:
    from ConeMapping.ConeMap_CumulativeClustering import ConeMap_CumulativeClustering as ConeMap
else:
    from ConeMapping.ConeMap_Naive import ConeMap_Naive as ConeMap

'''Delete'''
# Plot and Visualizations:
import StateEst_SR.StateEst_Dash as StateEst_DashBoard

# for showing messages:
from pprint import pprint
import google.protobuf.json_format as proto_format

# from SystemRunnerPart.print_messages_file import print_messages_file
from StateEst_SR.print_messages_file import save_as_json
import json


# typical python stuff:
import math
import time
import signal
import numpy as np



if IS_TIME_CODE_WITH_TIMER:
    from timeit import default_timer as timer

if IS_COMPARE_GROUND_TRUTH:
    import scipy.io as sio

class State:
    def __init__(self):
        # Development Flag:
        self.is_order_cones = True
        # logger: better than printing everything down to the Terminal:
        self.logger = InitLogger()
        # CalibrationVerifier: Makes sure we are not sending information too soon, when we actually know nothing:
        self.CalibrationVerifier = CalibrationVerifier( minCones=2 , minTime=1.5 , minIterations=10)
        # client: Reads messages and send messages:
        self._client = StateEstClient()
        self._message_timeout = 0.0001
        # Cone map:
        self._cone_map = ConeMap()  # simple version
        self._ordered_cones = {"left": np.array([]), "right": np.array([])}
        # Localization of car (Kalman Filter):
        self._car_state = messages.state_est.CarState()  # keeping our most recent known state here
        if IS_KALMAN_FILTER:
            #Kalman Filter and proxy objects:
            self._KalmanFilter = KalmanFilter()
            self._GPSOneShot = GPSOneShot()
            # KalmanManager: changes between 'prediction' and 'correction' and helps with frequency checking
            # updateInterval can be used to set minimum time between kalman updates
            self._KalmanManager = KalmanManager( updateInterval=0 ) 
        
        if IS_COMPARE_GROUND_TRUTH:
            self._Compare_GroundTruth = []
            self._Compare_StateEstimation = []
            self._cone_truth = np.array([])


    # V===============================================V Run: V===============================================V #
    def run(self):
        while True:
            ''' === Server: === '''
            try:
                server_msg = self._client.pop_server_message()
                if server_msg is not None:
                    if self.process_server_message(server_msg):
                        return
            except NoFormulaMessages:
                self.act_on_no_message("server message")
            except Exception as e:
                self.act_on_error(server_msg ,e, "server message")

            ''' === GPS: === '''
            try:
                gps_msg = self._client.get_gps_message(timeout=self._message_timeout)
                self.process_gps_message(gps_msg)
                self.send_message2control(gps_msg)
                self.send_message2dash_board(gps_msg)
            except NoFormulaMessages:
                self.act_on_no_message("GPS message")
            except Exception as e:
                self.act_on_error(gps_msg ,e, "GPS message")

            ''' === Car Data: === '''
            try:
                car_data_msg = self._client.get_car_data_message( timeout=self._message_timeout)
                self.process_car_data_message(car_data_msg)
                self.send_message2control(car_data_msg)
                self.send_message2dash_board(car_data_msg)
            except NoFormulaMessages:
                self.act_on_no_message("car data message")
            except Exception as e:
                self.act_on_error(car_data_msg ,e, "car data message")

            ''' === Perception: === '''
            try:
                cone_msg = self._client.get_cone_message(timeout=self._message_timeout)
                self.process_cones_message(cone_msg)
                self.send_message2control(cone_msg)
                self.send_message2dash_board(cone_msg)
            except NoFormulaMessages:
                self.act_on_no_message("perception message")
            except Exception as e:
                self.act_on_error(cone_msg ,e, "perception message")

            ''' === Ground Truth: === '''
            try:
                ground_truth_msg = self._client.get_ground_truth_message(timeout=self._message_timeout)
                self.process_ground_truth_message_memory(ground_truth_msg)
                # No need to send message2control
                self.send_message2dash_board(ground_truth_msg)
            except NoFormulaMessages:
                self.act_on_no_message("ground truth message")
            except Exception as e:
                self.act_on_error(ground_truth_msg ,e, "ground truth message")

    # ^===============================================^ Run: ^===============================================^ #

    def start(self):
        self._client.connect(1)
        if CONFIG == ConfigEnum.LOCAL_TEST:
            self._client.set_read_delay(0.01)
        self._client.start()

    def stop(self):
        # Print Finish message:
        self.logger.info(f"StateEstimation System Runner is shutting down")
        #Save Comparison Data for later analysis:
        if IS_COMPARE_GROUND_TRUTH:
            self._save_compare_data()
        #Shut down the client:
        if self._client.is_alive():
            self._client.stop()
            self._client.join()

    def _save_compare_data(self):
        # fetch data:
        gt = self._Compare_GroundTruth
        state = self._Compare_StateEstimation
        # Save data is dictionary
        mydict = {}
        mydict['GroundTruth'] = gt
        mydict['StateEstimation'] = state
        # set fullpath and name for file:
        outputDir = os.path.join(str(currentPath) ,"Output" )  
        fullpath = os.path.join( outputDir , 'CompareFile.mat')
        # Save file:
        try:
            sio.savemat( fullpath, mydict)
            self.logger.info(f"Saved Data for comparision at: {outputDir}")
        except: 
            self.logger.info(f"Failed at saving Data for comparision")

    def cone_convert_perception2StateCone(self, perception_cone):
        state_cone = messages.state_est.StateCone()

        try:
            state_cone.type = perception_cone.type #TODO The commented statement raises an error!
        except:
            self.logger.info(f"StateEst:cone_convert_perception2StateCone: Failed at setting cone type")


        # Here Theta and r are self coordinates. Meaning from the perspective of the car:
        state_cone.r = math.sqrt(
            math.pow(perception_cone.x, 2) + math.pow(perception_cone.y, 2)
        )
        # Normally, one will type  atan(y,x)...
        # but Perception looks at the world where x is positive towards the right side of the car, and y forward.
        state_cone.alpha = math.atan2(perception_cone.y, perception_cone.x)

        ## convert to our xNorth yEast:
        theta_total = (self._car_state.theta + state_cone.alpha)  #! here we depand on _car_state being recent and relevant

        # distance of cone from car, in our axis
        delta_x = state_cone.r * math.cos(theta_total)
        delta_y = state_cone.r * math.sin(theta_total)
        # add distance to cars position
        state_cone.position.x = self._car_state.position.x + delta_x
        state_cone.position.y = self._car_state.position.y + delta_y
        # We known nothing (John Snow) about the cone:
        state_cone.position_deviation = math.inf
        return state_cone

    def generate_cone_output_msg(self, cone):
        # state_cone = cone
    
        state_cone = messages.state_est.StateCone
        # things that we already have:
        if isinstance(cone,dict):
            state_cone.type       = cone["type"]
            state_cone.cone_id    = cone["cone_id"]
            state_cone.position.x = cone["position"][0]
            state_cone.position.y = cone["position"][1]       
        else:
            state_cone.type       = cone.type
            state_cone.cone_id    = cone.cone_id
            state_cone.position.x = cone.x
            state_cone.position.y = cone.y

        # Translate theta and are according to current car position and orientation
        delta_x = cone.x - self._car_state.x
        delta_y = cone.y - self._car_state.y
        state_cone.r = math.sqrt(math.pow(  delta_x ,2) + math.pow( delta_y ,2) )

        total_theta = math.atan2( delta_y , delta_x  )
        state_cone.theta =   total_theta - self._car_state.theta
       
        return state_cone

    def process_cones_message(self, cone_msg):
        """Parse Data and time"""
        cone_map = messages.perception.ConeMap()
        cone_msg.data.Unpack(cone_map)
        #Log for debugging:
        self.logger.debug(f"State got cone message ID {cone_msg.header.id} with {len(cone_map.cones)} cones in the queue")

        '''ConeMap: '''
        cone_array = np.array([])
        # Analyze all cones for position in map and other basic elements:
        for perception_cone in cone_map.cones:
            state_cone = self.cone_convert_perception2StateCone(perception_cone)
            cone_array = np.append(cone_array, state_cone)

        if IS_TIME_CODE_WITH_TIMER:
            cluster_start = timer()
 
        self._cone_map.insert_new_points(cone_array)
        real_cones = self._cone_map.get_real_cones()

        if IS_TIME_CODE_WITH_TIMER:
            print(f"clustering took {timer() - cluster_start} ms")



        ''''Order Cones: '''
        if IS_TIME_CODE_WITH_TIMER:
            order_start = timer()

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
            self.logger.debug(f"got gps: x: {x:6.2f} y: {gps_data.position.y:6.2f} ")


        if IS_KALMAN_FILTER:
            self._GPSOneShot.set_new_data(x, y, time_in_milisec)

        else:
            self._car_state.position.x = x
            self._car_state.position.y = y

    def process_ground_truth_message_memory(self, gt_msg):

        if not IS_COMPARE_GROUND_TRUTH:
            return

        """unpack data and time:"""
        gt_data = messages.ground_truth.GroundTruth()
        gt_msg.data.Unpack(gt_data)
        time_in_milisec = gt_msg.header.timestamp.ToMilliseconds()

        # Process Car States:
        car_turth = {}
        car_turth["time_in_milisec"] = time_in_milisec
        if gt_data.state_ground_truth.has_position_truth:
            car_turth["x"] = gt_data.state_ground_truth.position.x
            car_turth["y"] = gt_data.state_ground_truth.position.y
        if gt_data.state_ground_truth.has_car_measurments_truth:
            car_turth["delta"] = gt_data.state_ground_truth.car_measurments.steering_angle
        if gt_data.state_ground_truth.has_imu_measurments_truth:
            car_turth["speed"] = gt_data.state_ground_truth.imu_measurments.speed
            car_turth["theta"] = gt_data.state_ground_truth.imu_measurments.orientation.z

        if self._cone_truth.size == 0:  # Check no cones
            for cone in gt_data.state_ground_truth.cones:
                tmp_cone = {
                    "x": cone.position.x,
                    "y": cone.position.y,
                    "type": cone.type,
                }
                self._cone_truth = np.append(self._cone_truth, tmp_cone)

        if IS_COMPARE_GROUND_TRUTH:
            self._Compare_GroundTruth.append(car_turth)



    def process_car_data_message(self, car_data_msg):
        """ unpack data and time: """
        car_data = messages.sensors.CarData()
        car_data_msg.data.Unpack(car_data)
        time_in_milisec = car_data_msg.header.timestamp.ToMilliseconds()

        '''Parse Data'''
        delta = car_data.car_measurments.steering_angle  # steering angle
        acceleration_long = car_data.imu_sensor.imu_measurments.acceleration.x
        acceleration_lat = car_data.imu_sensor.imu_measurments.acceleration.y
        # some times this data exists:
        Vx = car_data.imu_sensor.imu_measurments.velocity.x
        Vy = car_data.imu_sensor.imu_measurments.velocity.y   

        if IS_KALMAN_FILTER:
            #don't update too frequently (Don't break your computer)
            '''Check if legit frequency:''' 
            isAllowedFrequency , DeltaT_fromPreviousUpdate_milisec = self._KalmanManager.checkCorrectUpdateFrequency(time_in_milisec)
            if not isAllowedFrequency:
                return


            """ Prediction: """
            # Make sure everyone knows that we're working on prediction
            self._KalmanManager.set_state("prediction") 

            data_for_prediction = {
                "steering_angle": delta,  #
                "delta_t_milisec": DeltaT_fromPreviousUpdate_milisec,
                "acceleration_long": acceleration_long,
                "acceleration_lat": acceleration_lat,
            }
            self._KalmanFilter.State_Prediction(data_for_prediction)

            ''' Correction: '''
            # Make sure everyone knows that we're working on correction
            self._KalmanManager.set_state("correction") 

            """ ! This should be changed ! Data for prediction should come from Control """
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
            data_for_correction["theta"] = car_data.imu_sensor.imu_measurments.orientation.z

            self._KalmanFilter.State_Correction(data_for_correction)

            '''Get State Estimation: '''
            self._car_state = self._KalmanFilter.Get_Current_State()

        else: #Not using kalman filter, but rather pure GPS and sensors:
            # Save Velocity:
            self._car_state.velocity.x = Vx
            self._car_state.velocity.y = Vy
            # Save Orientation:
            self._car_state.theta = theta    

        '''comparing with ground truth:'''
        if IS_COMPARE_GROUND_TRUTH:
            tempDict = {}
            tempDict['x']  = self._car_state.position.x
            tempDict['y']  = self._car_state.position.y
            tempDict['theta']  = self._car_state.theta
            tempDict['Vx']  = self._car_state.velocity.x
            tempDict['Vy']  = self._car_state.velocity.y
            self._Compare_StateEstimation.append(tempDict)


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

    def _create_formula_state_msg(self):
        # Makes a data object according to the formula msg proto "FormulaState" With the updated state:

        # create an empty message of state_est data:
        data = messages.state_est.FormulaState()
        
        """Current State:"""
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

        """Distance to finish:"""
        data.distance_to_finish, _ = self._calc_distance_to_finish()

        
        """ Cones:"""
        for cone in self._ordered_cones["right"]:
            state_cone = self.generate_cone_output_msg(cone)
            try:
                data.right_bound_cones.append(state_cone)
            except Exception as e:
                self.logger.info(f"Corrupted Cone. Exception Message: {e}")
        for cone in self._ordered_cones["left"]:
            state_cone = self.generate_cone_output_msg(cone)
            try:
                data.left_bound_cones.append(state_cone)
            except Exception as e:
                self.logger.info(f"Corrupted Cone. Exception Message: {e}")

        """Road estimation:"""
        #Missing values:
        # distance_from_left  #= 6;
        # distance_from_right #= 7;
        # road_angle          #= 8; // direction of road . absolute in the coordinate system of xNorth yEast

        '''Decide which kind of message to send:'''
        #Check if we are Calibrated when sending data to control:
        isCalibrated = self._check_module_calibrated(data) 
        if isCalibrated:
            data.message_type = self._KalmanManager.currentStateMessageType()
            # data.message_type = messages.state_est.FormulaStateMessageType.prediction_and_correction
        else:
            data.message_type = messages.state_est.FormulaStateMessageType.still_calibrating 

        '''That's our data'''    
        return data

    def _check_module_calibrated(self , data):
        numLeftCones  = len(data.left_bound_cones)  
        numRightCones = len(data.right_bound_cones)
        isCalibrated = self.CalibrationVerifier.checkCalibrated(numLeftCones=numLeftCones ,numRightCones=numRightCones )
        return isCalibrated


    def send_message2control(self, msg_in):
        # make an empty message:
        msg_id = msg_in.header.id
        msg_out = messages.common.Message()
        msg_out.header.id = msg_id

        # summarize all the data:
        data = self._create_formula_state_msg()

        # print message for debugging:
        if IS_PRINT_OUTPUT_MSG:
            message_as_text = parse_proto_message(data)
            self.logger.debug(message_as_text)

        # send message:
        msg_out.data.Pack(data)
        self._client.send_message(msg_out)

    def send_message2dash_board(self , msg_in):
        pass

    def act_on_no_message(self, source_str):
        if IS_DEBUG_MODE and IS_PRINT_ON_NO_MSG:
            msg =  f" no  {source_str:10}  message"
            self.logger.debug(msg)

    def act_on_error(self , inputMsg , e, source_str):
        error_msg = e.args[0]
        input_msg_id = inputMsg.header.id
        errorMsg =  f" Error at {source_str:15} ; Due to msg id {input_msg_id:10} ; Error: {error_msg}"
        self.logger.info(errorMsg)


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

def parse_proto_message(data):
    # print message
    msg_dict = proto_format.MessageToDict(data, including_default_value_fields=True, preserving_proto_field_name=True)
    msg_text = json.dumps(msg_dict, indent=2)
    return msg_text


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
