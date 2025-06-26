import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random

# parameters for calculation
num_points = 5000
num_studs = 6
stud_height = 200
wheelmount_height = 300
diameter = 15000
center_hub_diameter = 300
center_hub_height = 500

# array
matrix = numpy.empty(shape=(num_points, num_points), dtype='object')

def add_circle(center, radius, height):
    y_center, x_center = center
    radius_sqauared = radius ** 2

    for y in range(num_points):
        for x in range(num_points):
            if (x-x_center)**2 + (y-y_center)**2 <= radius_squared:
                matrix[y, x] += height

# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# 
# # Create a 2D array representing height values
# heightmap = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
# 
# # Create a figure and axes
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# 
# # Generate X and Y coordinates
# x, y = np.meshgrid(np.arange(heightmap.shape[1]), np.arange(heightmap.shape[0]))
# 
# # Plot the surface
# ax.plot_surface(x, y, heightmap, cmap='viridis')
# 
# plt.show()
# 
# 
# 
# from mayavi import mlab
# import numpy as np
# 
# # Create a 2D array representing height values
# heightmap = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
# 
# # Create a 3D surface
# mlab.surf(heightmap, warp_scale='auto')
# 
# # Show the plot
# mlab.show()
