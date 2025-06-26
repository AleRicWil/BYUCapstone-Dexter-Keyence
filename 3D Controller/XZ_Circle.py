import fullcontrol as fc
import numpy as np

filename = 'XZ_Circle'
printer = 'generic' 
# printer options: generic, ultimaker2plus, prusa_i3, ender_3, cr_10, bambulab_x1, toolchanger_T0, toolchanger_T1, toolchanger_T2, toolchanger_T3
print_settings = {'extrusion_width': 0.5,'extrusion_height': 0.2, 'nozzle_temp': 0, 'bed_temp': 0, 'fan_percent': 0, 'primer': 'no_primer'}
# 'extrusion_width' and 'extrusion_height' are the width and height of the printed line)

# create an empty list called steps
steps=[]

# define settings
steps.append(fc.Printer(print_speed=9000, travel_speed=9000))

# go to center of circle
x_cent = 100
y_cent = 100
z_cent = 100
steps.append(fc.Point(x=x_cent, y=y_cent, z = 10))
steps.append(fc.Point(x=x_cent, y=y_cent,z=z_cent))

# trace out circle in xz plane
radius = 80
if radius > x_cent:
    radius = x_cent
if radius > z_cent:
    radius = z_cent

theta = np.linspace(0, np.pi*2, 2000)
for i in theta:
    steps.append(fc.Point(x= x_cent + radius*np.cos(i), z= z_cent + radius*np.sin(i)))

# return to home
steps.append(fc.Point(z=0.5))
steps.append(fc.Point(x=10, y=10))

# transform the design into a plot
# fc.transform(steps, 'plot')

#   Save design to file
fc.transform(steps, 'gcode', fc.GcodeControls(printer_name=printer, save_as=filename, initialization_data=print_settings))