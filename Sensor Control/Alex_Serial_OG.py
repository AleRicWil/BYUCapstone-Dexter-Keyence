import serial
import time
import numpy as np
import matplotlib.pyplot as plt

def connect_rs485(port='COM3', baudrate=9600, timeout=1):
    """
    Connect to the RS485 device via the specified COM port.
    
    :param port: The COM port for the USB-to-RS485 adapter, e.g., 'COM3'.
    :param baudrate: Baud rate for the communication, e.g., 9600.
    :param timeout: Read timeout in seconds.
    :return: A serial connection object.
    """
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
            print(f"Connected to {port} at {baudrate} baud.")
            return ser
    except serial.SerialException as e:
        print(f"Error: {e}")
    return None

def send_command(ser, command):
    """
    Send a command to the RS485 device.

    :param ser: The serial connection object.
    :param command: The command to send as a byte string.
    """
    if ser and ser.is_open:
        ser.write(command)
        # print(f"Sent:", " ".join(f"{byte:02X}" for byte in command))

def read_response(ser):
    """
    Read the response from the RS485 device.

    :param ser: The serial connection object.
    :return: The response from the device.
    """
    if ser and ser.is_open:
        time.sleep(0.05)  # Small delay for device to respond
        response = ser.read_all()  # Read all available bytes
        # print(f"Received:", " ".join(f"{byte:02X}" for byte in response))
        return response
    return None

def calculate_bcc(data_bytes):
    """
    Calculate the Block Check Character (BCC) by XORing all bytes in data_bytes.

    :param data_bytes: List of bytes (integers) to be XORed.
    :return: The BCC as a single byte.
    """
    bcc = 0
    for byte in data_bytes:
        bcc ^= byte  # XOR each byte in the sequence
    return bcc


def main():
    # Adjust the COM port, baud rate, and command as needed
    port = 'COM5'  # Replace with your actual COM port
    baudrate = 9600  # Replace with your device's baud rate
    # Example usage
    STX = 0x02      # Start byte (not included in BCC calculation)
    Command = 0x43  # Command byte
    Data1 = 0xb0    # Data1 byte
    Data2 = 0x01    # Data2 byte
    ETX = 0x03      # End byte

    # List of bytes to include in BCC calculation
    data_bytes = [Command, Data1, Data2]

    # Calculate BCC
    BCC = calculate_bcc(data_bytes)
    print(f"BCC: {BCC:#02X}")
    
    command = bytes([STX, Command, Data1, Data2, ETX, BCC])
    # print(f"Command to send: {command}")
    print("Command to send:", " ".join(f"{byte:02X}" for byte in command))

    # Connect to RS485 device
    ser = connect_rs485(port, baudrate)

    if ser:
        # Send command
        send_command(ser, command)

        # Read response
        response = read_response(ser)

        print('Recording Data')
        n = 200
        x = np.linspace(0,1, n)
        vals = np.full(n, np.nan)
        for i in range(n):
            send_command(ser, command)
            response2 = read_response(ser)
            measurement = response2[2].to_bytes() + response2[3].to_bytes()
            measDec = int.from_bytes(measurement, 'big', signed=True)
            print(measDec)
            if measDec > 8000:
                measDec = np.nan
            vals[i] = measDec
            # print(f'measure? {measDec}')

        # Close the serial connection
        ser.close()
        print("Connection closed.")
        plt.plot(x, vals)


if __name__ == "__main__":
    main()
    plt.show()
