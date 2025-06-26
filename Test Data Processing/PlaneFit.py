import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# # Generate some random test points
# m = 20  # number of points
# delta = 0.01  # size of random displacement
# origin = np.random.rand(3, 1)  # random origin for the plane
# basis = np.random.rand(3, 2)  # random basis vectors for the plane
# coefficients = np.random.rand(2, m)  # random coefficients for points on the plane

# # Generate random points on the plane and add random displacement
# points = basis @ coefficients \
#          + np.tile(origin, (1, m)) \
#          + delta * np.random.rand(3, m)

# Read in points
points = np.loadtxt('measuredData.txt', delimiter=' ', skiprows=1).T
# Now find the best-fitting plane for the test points

# Subtract out the centroid and take the SVD
svd = np.linalg.svd(points - np.mean(points, axis=1, keepdims=True))

# Extract the left singular vectors
left = svd[0]

# The normal vector to the plane is the last singular vector
normal_vector = left[:, -1]

# Calculate the centroid of the points
centroid = np.mean(points, axis=1)

# Generate points on the best-fitting plane
# Define a grid for the plane
xx, yy = np.meshgrid(np.linspace(np.min(points[0]), np.max(points[0]), 1000),
                     np.linspace(np.min(points[1]), np.max(points[1]), 1000))

# Calculate corresponding z values on the plane
# Using the plane equation: ax + by + cz = d
# Where the normal vector is [a, b, c] and d = normal_vector . centroid
d = -normal_vector.dot(centroid)
zz = (-normal_vector[0] * xx - normal_vector[1] * yy - d) * (1 / normal_vector[2])

# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Scatter plot for the points
ax.scatter(points[0], points[1], points[2], color='b', label='Data Points')

# Plot the best-fitting plane
ax.plot_surface(xx, yy, zz, alpha=0.5, color='r', label='Best Fit Plane')

# Set labels
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_title('Random Points and Best Fit Plane')
ax.legend()

plt.show()
