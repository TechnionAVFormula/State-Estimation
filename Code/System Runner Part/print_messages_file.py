from pyFormulaClient import messages
from pyFormulaClient.NoNvidiaFormulaClient import FormulaClient, ClientSource, SYSTEM_RUNNER_IPC_PORT

import os
import sys 

def main(message_file):
    client = FormulaClient(ClientSource.PERCEPTION, 
        read_from_file=message_file, write_to_file=os.devnull)
    conn = client.connect(SYSTEM_RUNNER_IPC_PORT)
    msg = messages.common.Message()
    while not msg.data.Is(messages.server.ExitMessage.DESCRIPTOR):
        msg = conn.read_message()
        print(msg)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("print_messages_file.py <message file>")
        exit(1)
    main(sys.argv[1])
