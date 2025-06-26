import numpy as np
import matplotlib.pyplot as plt
import random

# Parameters for calculation
num_points = 10000
num_studs = 6
stud_height = .2
wheelmount_height = .1
stud_noise = .01
wheelmount_noise = .02
general_noise = .01
amplitude = .01

# Generate 10,000 evenly spaced points between 0 and 2*pi (one full period)
x_values = np.linspace(0, 2 * np.pi, num_points)

# Generate the sine wave values for each x
y_values = np.sin(x_values) * amplitude + general_noise*random.randrange(-1,1)

# adjust points to add studs, wheelmounts
for i in range (0,num_studs):
    one_stud = (int)(num_points / num_studs)
    wheelmount_width = (int)(num_points / 20)
    stud_width = (int)(num_points / 100)
    jstart = i*one_stud
    jfinish = jstart + wheelmount_width
    
    for j in range(jstart, jfinish):
        y_values[j] += wheelmount_height + wheelmount_noise*random.randrange(-1,1)
    
    jstart = (int)(jstart + (wheelmount_width/2) - (stud_width/2))
    jfinish = jstart + stud_width
    for j in range(jstart, jfinish):
        y_values[j] += stud_height + stud_noise*random.randrange(-1,1)


# Plot the sine wave
plt.figure(figsize=(10, 6))
plt.plot(x_values, y_values, label="Sine Wave")
plt.title("Sine Wave with 10,000 Points (1 Full Period)")
plt.xlabel("x (radians)")
plt.ylabel("sin(x)")
plt.grid(True)
plt.legend()

# Save the plot to a PNG file
plt.savefig('input.png', dpi=300)

# Show the plot
# plt.show()
