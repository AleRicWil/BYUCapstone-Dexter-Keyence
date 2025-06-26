import time
import numpy as np
import matplotlib.pyplot as plt
from Perform_Scan_LJS640 import perform_scan
import pyvista as pv

def main():
    data = perform_scan("png").astype(float)

    for i in data:
        i = (i-32768) * .0102

    cutOff=[-500, 500]
    max_x_width = 640
    min_x_width = 579
    reference_z_depth = 1116
    min_z_depth = 936

    numProfiles, numPoints = data.shape
    
    # Precompute x, y indices and ravel
    profile_indices = np.arange(numProfiles) - numProfiles / 2
    point_indices = (np.arange(numPoints) - numPoints / 2) * 0.2  # Scale applied here
    x, y = np.meshgrid(profile_indices, point_indices, indexing='ij')
    x = x.ravel()
    y = y.ravel()
        # Flatten z-values
    z = data.ravel() * 10
    valid_mask = (z >= -500) & (z <= 500)
    x = x[valid_mask]
    y = y[valid_mask]
    z = z[valid_mask]

    # Scaling correction
    xScaleSlope = ((min_x_width - max_x_width) / numPoints) / \
                        (reference_z_depth - min_z_depth)
    scale_factors = np.where(
        z <= 0,
        0.2,  # Default scale
        0.2 + xScaleSlope * z  # Adjusted scale
    )
    x *= scale_factors

    valid_mask = (x < 290) & (x > -270) & (z >= cutOff[0]) & (z <= cutOff[1])
    x = x[valid_mask]
    y = y[valid_mask]
    z = z[valid_mask]

    # Combine into a single array
    # cloud = np.stack((x, y, z), axis=0)
    cloud = np.array([y, -x, z])
    Plot_Cloud_PyVista(cloud)
    
    ## Create coordinate grids
    #x = np.linspace(0, 1, data.shape[1])
    #y = np.linspace(0, 1, data.shape[0])
    #X, Y = np.meshgrid(x, y)
    #
    #step = 2  # Adjust the step size to reduce the data size
    #X = X[::step, ::step]
    #Y = Y[::step, ::step]
    #data = data[::step, ::step]

    ## Plotting
    #fig = plt.figure()
    #ax = fig.add_subplot(111, projection='3d')

    ## Surface plot
    #ax.plot_surface(X, Y, data, cmap='viridis')

    ## Optional: Customize labels and angles
    #ax.set_title('3D Visualization of 3200x3200 Array')
    #ax.set_xlabel('X axis')
    #ax.set_ylabel('Y axis')
    #ax.set_zlabel('Values')

    #plt.show()

def Plot_Cloud_PyVista(points, pointSize=1.0):
    cloud = pv.PolyData(points.T)
    print(f'Displaying {points.shape[1]} points')

    # Map z-values to the HSV colormap
    z_values = points[2]  # Assuming the z-values are the third row of 'points'
    cloud.point_data['z'] = z_values
    cloud.point_data.set_array(z_values, 'z')

    # Create plotter
    plotter = pv.Plotter()
    plotter.set_background('gray')
    plotter.add_mesh(cloud, scalars='z', cmap='coolwarm', point_size=pointSize)
    
    # Define camera control callback functions
    def orbit_left():
        plotter.camera.azimuth -= 1  # Rotate left by 1 degree
        plotter.render()

    def orbit_right():
        plotter.camera.azimuth += 1  # Rotate right by 1 degree
        plotter.render()

    def orbit_up():
        plotter.camera.elevation += 1  # Rotate up by 1 degree
        plotter.render()

    def orbit_down():
        plotter.camera.elevation -= 1  # Rotate down by 1 degree
        plotter.render()

    def save_image():
        timestamp = time.strftime("%Y%m%d_%H%M%S") + f"{int(time.time() * 1000) % 1000:03d}"  # Format: YYYYMMDD_HHMMSS
        filename = f"point_cloud_{timestamp}.png"
        plotter.screenshot(filename)
        print(f"Image saved as {filename}")

    # Add key bindings
    plotter.add_key_event('4', orbit_left)
    plotter.add_key_event('6', orbit_right)
    plotter.add_key_event('8', orbit_up)
    plotter.add_key_event('2', orbit_down)
    plotter.add_key_event('s', save_image)

    # Set initial camera position to look down positive z-axis
    plotter.camera_position = 'xy'  # Sets view from positive z-axis
    plotter.camera_set = True  # Ensures the camera settings are applied
    plotter.reset_camera()     # Adjusts the view to fit the data

    # Display the plot
    plotter.show()


if __name__ == "__main__":
    main()
