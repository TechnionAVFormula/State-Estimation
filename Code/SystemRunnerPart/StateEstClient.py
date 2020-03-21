from pyFormulaClientNoNvidia import FormulaClient, messages  
from pyFormulaClientNoNvidia.ModuleClient import ModuleClient
from pyFormulaClientNoNvidia.MessageDeque import MessageDeque

class StateEstClient(ModuleClient):
    def __init__(self, read_from_file, write_to_file):
        super().__init__(FormulaClient.ClientSource.STATE_EST, read_from_file, write_to_file)    
        self.server_messages = MessageDeque()                                              
        self.cones_map_messages = MessageDeque(maxlen=1)        
        self.gps_messages = MessageDeque(maxlen=1)        
        self.imu_messages = MessageDeque(maxlen=1)        

    def _callback(self, msg):  
        if msg.data.Is(messages.perception.ConeMap.DESCRIPTOR):
            self.cones_map_messages.put(msg)
        elif msg.data.Is(messages.sensors.GPSSensor.DESCRIPTOR):
            self.gps_messages.put(msg)
        elif msg.data.Is(messages.sensors.IMUSensor.DESCRIPTOR):
            self.imu_messages.put(msg)
        else:
            self.server_messages.put(msg)

    def get_cone_message(self, blocking=True, timeout=None):
        return self.cones_map_messages.get(blocking, timeout)

    def get_gps_message(self, blocking=True, timeout=None):
        return self.gps_messages.get(blocking, timeout)

    def get_imu_message(self, blocking=True, timeout=None):
        return self.imu_messages.get(blocking, timeout)

    def pop_server_message(self, blocking=False, timeout=None):
        return self.server_messages.get(blocking, timeout)