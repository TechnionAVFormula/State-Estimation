from pyFormulaClientNoNvidia import messages
from pyFormulaClientNoNvidia.FormulaClient import FormulaClient, ClientSource, SYSTEM_RUNNER_IPC_PORT

import os
import sys 
from google.protobuf import text_format, json_format, timestamp_pb2
import json
import argparse




def file_path(string):
    if os.path.isfile(string):
        return string
    else:
        raise argparse.ArgumentTypeError(f"Messages file {string} not found")

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('message_file', type=file_path)
    # parser.add_argument('-d', '--print-default-values', required=False)

    return parser.parse_args()


def parse_file( message_file ):
    
    client = FormulaClient(ClientSource.PERCEPTION, 
        read_from_file= message_file , write_to_file=os.devnull)
    conn = client.connect(SYSTEM_RUNNER_IPC_PORT)

    out = []
    msg = messages.common.Message()
    while not msg.data.Is(messages.server.ExitMessage.DESCRIPTOR):
        msg = conn.read_message()
        if msg.data.Is(messages.state_est.FormulaState.DESCRIPTOR):
            state= messages.state_est.FormulaState()
            msg.data.Unpack(state)
            # to be continud:
            msg_id = msg.header.id
            msg_time = msg.header.timestamp.ToDatetime()
            msg_data = state

            out.append( (msg_id , msg_time , msg_data) )

    return out
            

def main():
    args = parse_args()
    print( parse_file(args.message_file) )



if __name__ == '__main__':
    main()
