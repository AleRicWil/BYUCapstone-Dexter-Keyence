import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import logging
import argparse
import sys

log = logging.getLogger(__name__)

def connect_rs485(port='COM6', baudrate=9600, timeout=1):
    # connects to device on {port} using {baudrate}, returns {ser} connection
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        if ser.is_open:
            log.info(f"Connected to {port} at {baudrate} baud.")
            return ser
    except serial.SerialException as e:
        log.error(e)
    return None

def send_command(ser, command):
    # sends {command} to device using {ser} connection
    if ser and ser.is_open:
        ser.write(command)
        log.debug(f"Sent:", " ".join(f"{byte:02X}" for byte in command))

def read_response(ser, delay):
    # reads and returns {response} from {ser} connection
    if ser and ser.is_open:
        time.sleep(delay)  # Small delay for device to respond
        response = ser.read_all()  # Read all available bytes
        log.debug(f"Received:", " ".join(f"{byte:02X}" for byte in response))
        return response
    return None

def calculate_bcc(data_bytes):
    # calculates Block Check Character (BCC) by XORing all bytes in data_bytes
    bcc = 0
    #log.info("Calculating BCC from: ", " ".join(f"{byte:02X}" for byte in data_bytes))
    for byte in data_bytes:
        bcc ^= byte  # XOR each byte in the sequence
    return bcc

def generate_command(command_bytes):
    STX = 0x02
    ETX = 0x03
    BCC = 0
    for byte in command_bytes:
        BCC ^= byte
    return bytes([STX, command_bytes[0], command_bytes[1], command_bytes[2], ETX, BCC])


if __name__ == "__main__":
    # argparse junk
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--num_points", action="store", default=100, type=int, help="Number of points to collect during measurement")
    parser.add_argument("-p", "--port", action="store", default='COM6', type=str, help="Port to bind to for serial connection (linux format=/dev/ttyUSBXX, windows format=COMXX)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Activates debugging & informational output to terminal")
    parser.add_argument("-r", "--rate", action="store", default=9600, type=int, help="Baud rate, default 9600")
    parser.add_argument("-d", "--delay", action="store", default=0.05, type=float, help="Delay in seconds between sending and receiving, default 0.05s")
    args = parser.parse_args()
    if args.verbose:
        logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
    else:
        logging.basicConfig(stream=sys.stdout, level=logging.ERROR)
   
    read_command = [0x43, 0xb0, 0x01]

    command = generate_command(read_command)
    print("Command to send:", " ".join(f"{byte:02X}" for byte in command))

    # Connect to RS485 device
    log.info(f"Connecting to device using port: {args.port} rate: {args.rate}")
    ser = connect_rs485(args.port, args.rate)

    if ser:
        print(f"Recording {args.num_points} points")
        x = np.linspace(0,1, args.num_points)
        vals = np.full(args.num_points, np.nan)
        for i in range(args.num_points):
            send_command(ser, command) # sending command to laser
            response = read_response(ser, 0.05) # receiving response from laser, parsing response
            measDec = int.from_bytes(response[2].to_bytes() + response[3].to_bytes(), 'big', signed=True)
            if measDec > 8000:
                measDec = np.nan
            vals[i] = measDec
            print(f'Measured: {measDec}')

        # Close the serial connection
        ser.close()
        print("Connection closed.")
        #plt.plot(x, vals)

    plt.show()
