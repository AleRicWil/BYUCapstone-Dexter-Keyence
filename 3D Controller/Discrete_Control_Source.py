#to send a file of gcode to the printer
from printrun.printcore import printcore
from printrun import gcoder
import time

#to generate gcode
import fullcontrol as fc
import numpy as np

import serial
import time
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

if __name__ == "__main__":
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



  #create Gcode
  filename = 'XY_Circle'
  printer = 'generic' 
  # printer options: generic, ultimaker2plus, prusa_i3, ender_3, cr_10, bambulab_x1, toolchanger_T0, toolchanger_T1, toolchanger_T2, toolchanger_T3
  print_settings = {'primer': 'no_primer'}
  # 'extrusion_width' and 'extrusion_height' are the width and height of the printed line)
  home = 'G28'
  xhome = 65.0
  yhome = 115.0
  zhome = 10.0

  # create list of steps
  steps=[]
  steps.append(fc.Printer(print_speed=5000, travel_speed=5000))
  steps.append(fc.Point(x=xhome+40,y=yhome,z=zhome))
  #   Save design to file
  myGCode = fc.transform(steps, 'gcode', fc.GcodeControls(initialization_data=print_settings))
  print(myGCode)

  p=printcore('COM6', 115200) # or p.printcore('COM3',115200) on Windows
  gcode=[myGCode] # or pass in your own array of gcode lines instead of reading from a file
  gcode = gcoder.LightGCode(gcode)


  if ser:
        send_command(ser, command)
        response = read_response(ser)
        print('Connected to Sensor')

        while not p.online:
          time.sleep(0.1)
        p.startprint(gcode) # this will start a print
        print('Connected to Printer')
        print('Centering...')
        step=[fc.Point(x=xhome+40,y=yhome,z=10)]
        myGCode = fc.transform(step, 'gcode', fc.GcodeControls(initialization_data=print_settings))
        p.send_now(myGCode)
        time.sleep(5)

        
        xCent = xhome
        yCent = yhome
        n = 1000
        x = np.linspace(0,1, n)
        vals = np.full(n, np.nan)
        r = 45 # radius of circle
        theta = np.linspace(0, np.pi*2, n)
        print('Recording Data')
        for i in range(n):
            # print('New Point')
            step=[fc.Point(x=xCent+r*np.cos(theta[i]),y=yCent+r*np.sin(theta[i]),z=10)]
            myGCode = fc.transform(step, 'gcode', fc.GcodeControls(initialization_data=print_settings))
            p.send_now(myGCode)
            time.sleep(0.05)
            
            send_command(ser, command)
            response2 = read_response(ser)
            measurement = response2[2].to_bytes() + response2[3].to_bytes()
            measDec = int.from_bytes(measurement, 'big', signed=True)
            # print(measDec)
            if measDec > 8000:
                measDec = np.nan
            vals[i] = measDec
            
        print('Last Point')
        step=[fc.Point(x=xhome,y=yhome,z=zhome)]
        myGCode = fc.transform(step, 'gcode', fc.GcodeControls(initialization_data=print_settings))
        p.send_now(myGCode)
        time.sleep(2)

        p.disconnect()
        print('Disconnected Printer')
        ser.close()
        print("Disconnected Sensor")
        print("Writing to File")
        xVals = np.full(n, np.nan)
        yVals = np.full(n, np.nan)
        zVals = np.full(n, np.nan)
        for i in range(n):
          xVals[i] = xCent+r*np.cos(theta[i])
          yVals[i] = y=yCent+r*np.sin(theta[i])
          zVals[i] = vals[i]

        hubFacePoints = np.array([xVals, yVals, zVals])
        np.savetxt('measuredData.txt', hubFacePoints.T, fmt='%.6f', delimiter=' ', header='X Y Z', comments='')
        print("Plotting Data")
        plt.scatter(x, vals, s=5)
        plt.show()
