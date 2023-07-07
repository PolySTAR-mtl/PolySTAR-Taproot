from argparse import ArgumentParser, SUPPRESS
import serial, serial.tools.list_ports
import os
import time

clear = lambda: os.system('cls')

def read_packet(ser):
    byte = ser.read(1)
    if byte != SOF:
        return None, None, None
    
    command_id = ser.read(1)

    data_length = ser.read(1)
    data_length = int.from_bytes(data_length, 'little')

    data = ser.read(data_length)

    return command_id, data_length, data

def print_display(counters, intervals):
    clear()
    print("Command ID"+"\tReceived"+"\tFrequency")
    for command, count in counters.items():
        if command in intervals:
            print(str(command)+"\t\t"+str(count)+"\t\t"+str(int(1000/intervals[command])))
        else:
            print(str(command)+"\t"+str(count))

parser = ArgumentParser(add_help=False, description="CS -> CV Packet Analyzer Tool")
required = parser.add_argument_group('required arguments')
optional = parser.add_argument_group('optional arguments')

required.add_argument('-p','--port', metavar='PORT', action='store', help='COM Port', type=str, required=True)
required.add_argument('-b','--baud', metavar='BAUDRATE', action='store', help='Serial Baud Rate', type=int, required=True)
required.add_argument('-s','--sof', metavar='SOF', action='store', help='Start of frame byte (in hex)', type=str, required=True)
optional.add_argument('-h','--help',action='help',default=SUPPRESS,help='show this help message and exit')
optional.add_argument('-o','--output', metavar='OUTPUT', action='store', help='Path to output file (will be overwritten)', type=str, required=False)
optional.add_argument('-v','--verbose', action='store_true', help='Print packets to console', required=False)

args = parser.parse_args()

PORT = args.port
BAUDRATE = args.baud
SOF = bytes.fromhex(args.sof)
VERBOSE = args.verbose
OUTPUT = args.output

available_ports = [tuple(p)[0] for p in list(serial.tools.list_ports.comports())]

if PORT not in available_ports:
    print("COM port '" + PORT + "' not found")
    print("Available ports: "+str(available_ports))
    exit()

try:
    ser = serial(PORT,BAUDRATE)
except:
    print("Error opening serial port")
    exit()

if OUTPUT != None:
    try:
        output_file = open(OUTPUT,'w')
    except:
        print("Error opening file "+OUTPUT)
        exit()

packet_counters = {"Bad Header": 0, b'\x01' : 1337}
packet_intervals = {b'\x01' : 250}
previous_packet_timestamps = {}

previous_display = time.time()
display_interval = 0.25

while True:
    try:
        if time.time() - previous_display > display_interval:
            previous_display = time.time()
            print_display(packet_counters,packet_intervals)

        command_id, data_length, data = read_packet()
        
    except KeyboardInterrupt:
        print_display(packet_counters,packet_intervals)
        exit()