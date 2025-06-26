#to send a file of gcode to the printer
from printrun.printcore import printcore
from printrun import gcoder
import time

#to generate gcode
import fullcontrol as fc

class printerCtrl():
    def __init__(self, port: str, printer: str, home):
        self.port = port
        self.printer = printer
        self.print_settings = {'primer': 'no_primer'}
        self.homeCmd = 'G28'
        self.home = home
        
    def Start_Control(self):
        steps=[]
        steps.append(fc.Printer(print_speed=5000, travel_speed=5000))
        steps.append(fc.Point(x=self.home[0],y=self.home[1],z=self.home[2]))
        #   Save design to file
        myGCode = fc.transform(steps, 'gcode', fc.GcodeControls(initialization_data=self.print_settings))

        self.p=printcore(self.port, 115200) # or p.printcore('COM3',115200) on Windows
        gcode=[myGCode] # or pass in your own array of gcode lines instead of reading from a file
        gcode = gcoder.LightGCode(gcode)

        while not self.p.online:
          time.sleep(0.1)
        self.p.startprint(gcode) # this will start a print
        print('Connected to Printer')

    def Go_Center(self, center, delay):
        print('Centering...')
        step=[fc.Point(x=center[0],y=center[1],z=10)]
        myGCode = fc.transform(step, 'gcode', fc.GcodeControls(initialization_data=self.print_settings))
        self.p.send_now(myGCode)
        time.sleep(delay)

    def Go_Point(self, point, delay: float):
        # print('New Point')
        step=[fc.Point(x=point[0],y=point[1],z=point[2])]
        myGCode = fc.transform(step, 'gcode', fc.GcodeControls(initialization_data=self.print_settings))
        self.p.send_now(myGCode)
        time.sleep(delay)

    def Go_Home(self):
        print('Going Home')
        step=[fc.Point(x=self.home[0]+40,y=self.home[1],z=self.home[2])]
        myGCode = fc.transform(step, 'gcode', fc.GcodeControls(initialization_data=self.print_settings))
        self.p.send_now(myGCode)
        time.sleep(2)

    def disconnect(self):
        self.p.disconnect()
        print('Disconnected Printer')
