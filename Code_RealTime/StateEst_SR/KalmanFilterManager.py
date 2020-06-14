from enum import Enum

## For relative imports:
import sys, os, pathlib
currentPath = pathlib.Path(os.path.dirname(__file__))
relativePath = currentPath.parent.parent
sys.path.append(str(relativePath))

## classes and enums from our utilities:
from StateEst_Utils.MessagesClass import messages, NoFormulaMessages


class KalmanState(Enum):
    idle        = None
    prediction  = 1
    correction  = 2


class KalmanManager():
    """KalmanManager 

    Helps manage stuff about our KalmanFilter Module in the State Est System Runner.

    kwargs:
            - updateInterval (double): time between kalman updates [mili-sec]
 
    """
    def __init__(self , **kwargs):
        self.currentState = KalmanState.idle
        self.lastKalmanUpdateTime = None

        # Default values:
        self.updateInterval = 0
        # Possible Inputs:
        for key, value in kwargs.items(): 
            if key == 'updateInterval':                                                    
                self.updateInterval = value 

    def currentStateMessageType(self):
        if self.currentState == KalmanState.prediction:
            return messages.state_est.FormulaStateMessageType.only_prediction
        elif self.currentState == KalmanState.correction:
            return messages.state_est.FormulaStateMessageType.prediction_and_correction 
        else:
            return None

    def set_state(self , state_str):
        if   state_str == "prediction":
            self.currentState = KalmanState.prediction
        elif state_str == "correction":
            self.currentState = KalmanState.correction
        elif state_str == "idle":
            self.currentState = KalmanState.idle
        else:
            raise Exception("Not Legit 'state_str' ")

    def checkCorrectUpdateFrequency(self, time_in_milisec):
        """checkCorrectUpdateFrequency check if it's okay to activate Kalmad update at the moment.

        Args:
            time_in_milisec (double): current time or time of message [mili-sec]

        Returns:
            Bolean: Are we ready or not
        """

        if self.lastKalmanUpdateTime == None:  #first timee we're here
            self.lastKalmanUpdateTime = time_in_milisec # Update inner clock
            DeltaT_milisec = 0
            isAllowed = False
        else:
            DeltaT_milisec = time_in_milisec - self.lastKalmanUpdateTime  # calc time since last kalman update
            #Check frequency of update:
            if DeltaT_milisec >= self.updateInterval:
                self.lastKalmanUpdateTime = time_in_milisec # Update inner clock
                isAllowed = True                
            else:
                # self.lastKalmanUpdateTime stays the same
                isAllowed = False
        
        return isAllowed , DeltaT_milisec

        


if __name__ == "__main__":
    km1 = KalmanManager()  #immplies updateInterval=0
    km2 = KalmanManager( updateInterval=150)
    print(f"current state = {km1.currentState}")
    print(f"current state = {km2.currentState}")
    is_ready_1 , deltaT_1 = km1.checkCorrectUpdateFrequency(0)
    is_ready_2 , deltaT_2 = km2.checkCorrectUpdateFrequency(0)
    print(f"kalmanManager1 ready = {str(is_ready_1): <5} , DeltaT={deltaT_1}")    
    print(f"kalmanManager2 ready = {str(is_ready_2): <5} , DeltaT={deltaT_2}")
    is_ready_1 , deltaT_1 = km1.checkCorrectUpdateFrequency(100)
    is_ready_2 , deltaT_2 = km2.checkCorrectUpdateFrequency(100)
    print(f"kalmanManager1 ready = {str(is_ready_1): <5} , DeltaT={deltaT_1}")    
    print(f"kalmanManager2 ready = {str(is_ready_2): <5} , DeltaT={deltaT_2}")
    is_ready_1 , deltaT_1 = km1.checkCorrectUpdateFrequency(200)
    is_ready_2 , deltaT_2 = km2.checkCorrectUpdateFrequency(200)
    print(f"kalmanManager1 ready = {str(is_ready_1): <5} , DeltaT={deltaT_1}")    
    print(f"kalmanManager2 ready = {str(is_ready_2): <5} , DeltaT={deltaT_2}")