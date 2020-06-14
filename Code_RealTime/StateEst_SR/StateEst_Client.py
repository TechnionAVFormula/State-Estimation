## For relative imports:
import sys, os, pathlib
currentPath = pathlib.Path(os.path.dirname(__file__))
relativePath = currentPath.parent.parent
sys.path.append(str(relativePath))

from StateEst_Utils.config import ConfigEnum
from StateEst_Utils.config import CONFIG, IN_MESSAGE_FILE, OUT_MESSAGE_FILE

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import FormulaClient, messages  
    from pyFormulaClient.ModuleClient import ModuleClient
    from pyFormulaClient.MessageDeque import MessageDeque
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import FormulaClient, messages  
    from pyFormulaClientNoNvidia.ModuleClient import ModuleClient
    from pyFormulaClientNoNvidia.MessageDeque import MessageDeque
else:
    raise NameError('User Should Choose Configuration from config.py')

class StateEstClient(ModuleClient):
    def __init__(self):
        if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
            super().__init__(FormulaClient.ClientSource.STATE_EST)       
        elif ( CONFIG == ConfigEnum.LOCAL_TEST):
            in_message_file  = IN_MESSAGE_FILE
            out_message_file = OUT_MESSAGE_FILE
            super().__init__(FormulaClient.ClientSource.STATE_EST, in_message_file, out_message_file)
            # parent_dir = 'Code_RealTime'
            # try:
            #     super().__init__(FormulaClient.ClientSource.STATE_EST, in_message_file, out_message_file)
            # except Exception as e:
            #     print(f"Init error of StateEstClient(ModuleClient) for error:  {e} ") 
            #     #in_message_file  = os.path.join(parent_dir,in_message_file)   #Funny but this gives Unix pathseperator
            #     in_message_file  = parent_dir + '/' + in_message_file 
            #     out_message_file = parent_dir + '/' + out_message_file 
            #     super().__init__(FormulaClient.ClientSource.STATE_EST, in_message_file, out_message_file)

        self.server_messages    = MessageDeque()                                              
        self.cones_map_messages = MessageDeque(maxlen=1)        
        self.gps_messages       = MessageDeque(maxlen=1)        
        self.car_data_messages  = MessageDeque(maxlen=1)    
        self.ground_truth_message= MessageDeque(maxlen=1)    

    def _callback(self, msg):  
        if msg.data.Is(messages.perception.ConeMap.DESCRIPTOR):
            self.cones_map_messages.put(msg)
        elif msg.data.Is(messages.sensors.GPSSensor.DESCRIPTOR):
            self.gps_messages.put(msg)
        elif msg.data.Is(messages.sensors.CarData.DESCRIPTOR):
            self.car_data_messages.put(msg)
        elif msg.data.Is(messages.ground_truth.GroundTruth.DESCRIPTOR):
            self.ground_truth_message.put(msg)
        else:
            self.server_messages.put(msg)

    def get_cone_message(self, blocking=True, timeout=None):
        return self.cones_map_messages.get(blocking, timeout)

    def get_gps_message(self, blocking=True, timeout=None):
        return self.gps_messages.get(blocking, timeout)

    def get_car_data_message(self, blocking=True, timeout=None):
        return self.car_data_messages.get(blocking, timeout)

    def get_ground_truth_message(self , blocking=True, timeout=None):
        return self.ground_truth_message.get(blocking, timeout)

    def pop_server_message(self, blocking=False, timeout=None):
        return self.server_messages.get(blocking, timeout)