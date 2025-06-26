import serial
import time
import matplotlib.pyplot as plt
import numpy as np

class sensorCtrl():
    def __init__(self, port: str, baudrate: int):
        # Adjust the COM port, baud rate, and command as needed
        self.port = port  # Replace with your actual COM port
        self.baudrate = baudrate  # Replace with your device's baud rate
        # Example usage
        self.STX = 0x02      # Start byte (not included in BCC calculation)
        Command = 0x43  # Command byte
        Data1 = 0xb0    # Data1 byte
        Data2 = 0x01    # Data2 byte
        self.ETX = 0x03 

    def connect_rs485(self, timeout=1):
        """
        Connect to the RS485 device via the specified COM port.
        
        :param port: The COM port for the USB-to-RS485 adapter, e.g., 'COM3'.
        :param baudrate: Baud rate for the communication, e.g., 9600.
        :param timeout: Read timeout in seconds.
        :return: A serial connection object.
        """
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout
            )
            if self.ser.is_open:
                print(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error: {e}")
        return None

    def calculate_bcc(self, cmdBytes):
        """
        Calculate the Block Check Character (BCC) by XORing all bytes in data_bytes.

        :param data_bytes: List of bytes (integers) to be XORed.
        :return: The BCC as a single byte.
        """
        self.BCC = 0
        for byte in cmdBytes:
            self.BCC ^= byte  # XOR each byte in the sequence

    def send_command(self, command):
        """
        Send a command to the RS485 device.

        :param ser: The serial connection object.
        :param command: The command to send as a byte string.
        """
        if self.ser and self.ser.is_open:
            self.ser.write(command)
            # print(f"Sent:", " ".join(f"{byte:02X}" for byte in command))

    def read_response(self):
        """
        Read the response from the RS485 device.

        :param ser: The serial connection object.
        :return: The response from the device.
        """
        if self.ser and self.ser.is_open:
            time.sleep(0.03)  # Small delay for device to respond
            self.response = self.ser.read_all()  # Read all available bytes
            # print(f"Received:", " ".join(f"{byte:02X}" for byte in response))
        else:    
            return None

    def check_connected(self):
        if self.ser:
            print('Sensor Connected')
            return True
        else:
            print('Sensor Not Connected')
            return False
        
    def get_measurement(self):
        Command = 0x43  # Command byte
        Data1 = 0xb0    # Data1 byte
        Data2 = 0x01
        cmdBytes = [Command, Data1, Data2]
        self.calculate_bcc(cmdBytes)
        command = bytes([self.STX, Command, Data1, Data2, self.ETX, self.BCC])
        self.send_command(command)
        self.read_response()

        measurement = self.response[2].to_bytes() + self.response[3].to_bytes()
        measDec = int.from_bytes(measurement, 'big', signed=True)
        if measDec > 8000:
                measDec = np.nan
        return measDec

    def disconnect_rs485(self):
        self.ser.close()
        print("Disconnected Sensor")