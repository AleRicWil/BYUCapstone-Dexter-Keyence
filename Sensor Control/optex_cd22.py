import serial
import time
import numpy as np
import logging
import sys

log = logging.getLogger(__name__)
read_command = [0x43, 0xb0, 0x01]

class optex_cd22():
    int port
    int baudrate
    float delay
    conn = None

    def __init__(self, port, baudrate, delay):
        self.port = port
        self.baudrate = baudrate
        self.delay = delay
        
    def connect(port='COM6', baudrate=9600, timeout=1):
        # connects to device on {port} using {baudrate}, returns {ser} connection
        try:
            self.conn = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=timeout
                    )
            if ser.is_open:
                log.info(f"Connected to {port} at {baudrate} baud.")
        except serial.SerialException as e:
            log.error(e)
    
    def send(self, command):
        # send {command} to device using {comm}
        if self.conn and self.conn.is_open:
            self.conn.write(command)
    
    def wait():
        # waits for {delay} amoutn of time (used to allow for half-duplex flow changing)
        time.sleep(self.delay)

    def recv(self, delay):
        # reads and returns {response} from {conn}
        if self.conn and self.conn.is_open:
            response = ser.read_all()
            return response
        return None
    
    def generate_command(command_bytes):
        # generates a command using command_bytes, which consists of: {Command, Data1, Data2}
        STX = 0x02
        ETX = 0x03
        BCC = 0
        for byte in command_bytes:
            BCC ^= byte
        return bytes([STX, command_bytes[0], command_bytes[1], command_bytes[2], ETX, BCC])

    def read_value(self):
        # returns the value currently being read by the sensor
        global read_command # not sure how to better do a #DEFINE-like thing....
        command = generate_command(read_command)
        send(self, command)
        wait()
        return recv()


