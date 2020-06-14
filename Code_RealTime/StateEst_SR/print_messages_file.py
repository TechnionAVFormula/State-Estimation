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

class FormulaProtoPrinter:
    def __init__(self, print_default_values):
        self.print_default_values = print_default_values

    def _message_formatter(self, message, indent, as_one_line):
        if message.DESCRIPTOR != messages.common.Header.DESCRIPTOR:
            return None
        
        indent_str = ' ' * indent
        field_end_str = ' ' if as_one_line else '\n'

        formatted_header = f'id: {message.id}{field_end_str}'
        formatted_header += f'{indent_str}timestamp: {message.timestamp.ToDatetime()}{field_end_str}'
        formatted_header += f'{indent_str}source: {ClientSource(message.source).name}{field_end_str}'
        formatted_header += f'{indent_str}priority: {message.priority}'

        return formatted_header

    def print_message(self, msg):
        msg_str = text_format.MessageToString(msg, message_formatter=self._message_formatter)
        msg_str = msg_str.replace('type.googleapis.com/', '')
        print(msg_str)


def print_file( message_file ):
    
    client = FormulaClient(ClientSource.PERCEPTION, 
        read_from_file= message_file , write_to_file=os.devnull)
    conn = client.connect(SYSTEM_RUNNER_IPC_PORT)

    printer = FormulaProtoPrinter(False)

    msg = messages.common.Message()
    while not msg.data.Is(messages.server.ExitMessage.DESCRIPTOR):
        msg = conn.read_message()
        printer.print_message(msg)

def save_as_json(msg):
    msg_str = json_format.MessageToDict(msg)  
    print(msg_str) 

def main():
    args = parse_args()
    print_file(args.message_file)



if __name__ == '__main__':
    main()
