from pyFormulaClient import NoNvidiaFormulaClient, messages
# from pyFormulaClient import exceptions as dwexceptions
import threading

class ModuleClient(threading.Thread):
    def __init__(self, source, read_from_file, write_to_file):
        super().__init__()
        self._source = source
        self._client = NoNvidiaFormulaClient.FormulaClient(source, read_from_file, write_to_file) 
        self._running = False   
        self._conn = None

    def connect(self, timeout_us):                                                                         
        self._conn = self._client.connect(port=NoNvidiaFormulaClient.SYSTEM_RUNNER_IPC_PORT, timeout=timeout_us)           
        
    def set_read_delay(self, delay):
        self._conn.read_delay = 0.05

    def _callback(self, msg):
        pass

    def send_message(self, msg, timeout=NoNvidiaFormulaClient.DW_TIMEOUT_INFINITE):
        msg.header.source = self._source.value
        msg.header.timestamp.CopyFrom(messages.get_proto_system_timestamp())
        msg.header.steady_timestamp.CopyFrom(messages.get_proto_steady_timestamp())
        self._conn.send_message(msg, timeout)

    def stop(self):
        self._running = False

    def run(self):
        self._running = True
        while(self._running):
            try:
                new_msg = self._conn.read_message(10)
                if new_msg is not None:
                    self._callback(new_msg)
            except Exception as e:
            # except dwexceptions.DWStatusException as status_exception:
                # if status_exception.status != dwexceptions.dwStatus.DW_TIME_OUT:
                #     print(status_exception)
                pass