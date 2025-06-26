import numpy as np
import matplotlib.pyplot as plt
import time

import pyvista as pv

plotWidth = 200 # width of plots

def Load_Scan(filename, maxPoints, cutOff):
    z_scale = 1     # 10*60/71
    # Load data from file
    data = np.loadtxt(filename, delimiter=',')

    # Get dimensions
    numProfiles, numPoints = data.shape
    print('Points per Profile: ', numPoints)
    print('Profiles: ', numProfiles)
    print('Raw Points: ', numPoints * numProfiles)

    # Generate x, y coordinates
    x, y = np.indices((numProfiles, numPoints), dtype=np.float64)
    x = x.ravel()
    y = y.ravel()
    z = data.ravel() * z_scale

    valid_mask = (z >= -500) & (z <= 500)
    x = x[valid_mask]
    y = y[valid_mask]
    z = z[valid_mask]

    # Scale xy
    scale = 0.2 # mm/point
    xScaleSlope = ((579 - 640)/numPoints) / (1116 - 936) # ((xWidthNear - xWidthRef)/numPoints)/(refZ - nearZ)

    x -= numProfiles/2
    y -= numPoints/2

    for i in range(len(x)):
        if z[i] <= 0:
            x[i] *= scale
        else:
            x[i] *= (scale + xScaleSlope*z[i])

    # x *= scale
    y *= scale

    valid_mask = (x > -1000) & (x < 1000) & (z >= cutOff[0]) & (z <= cutOff[1])
    x = x[valid_mask]
    y = y[valid_mask]
    z = z[valid_mask]

    # Combine into a single array
    points = np.array([y, -x, z])
    print('Valued Points: ', points.shape[1])

    # Sample points if necessary
    if points.shape[1] > maxPoints:
        sampled_indices = np.linspace(0, points.shape[1] - 1, maxPoints, dtype=int)
        points = points[:, sampled_indices]

    print('Sampled Points: ', points.shape[1])
    return points

def Load_Sim_Scan(filename):
    data = np.loadtxt(filename, skiprows=1)#[::10]   # Take every 10th point. Bottleneck at np.linalg.svd() in Calc_Plane()
    x = data[:,0]
    y = data[:,1]
    z = data[:,2]#/100
    cloud = np.array([x, y, z])

    return cloud

def close_all_on_esc(event):
    if event.key == 'escape':  # Check if 'Esc' key is pressed
        plt.close('all')  # Close all figures

def Plot_Cloud(Points, plotNum, title=0, label=0, center=False, s=1, c=0):
    # Check if the figure already exists
    if plt.fignum_exists(plotNum):
        fig = plt.figure(plotNum)
        ax = fig.gca()  # Get the current axes
    else:
        fig = plt.figure(plotNum)
        ax = fig.add_subplot(111, projection='3d')
    fig.canvas.mpl_connect('key_press_event', close_all_on_esc)

    
    # Plot the data
    if label != 0:
        if c !=0:
            ax.scatter(Points[0], Points[1], Points[2], label=label, s=s, c=c)
        else:
            ax.scatter(Points[0], Points[1], Points[2], label=label, s=s)
    else:
        if c !=0:
            ax.scatter(Points[0], Points[1], Points[2], s=s, c=c)
        else:
            ax.scatter(Points[0], Points[1], Points[2], s=s)
      
    # Add labels
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_box_aspect([1, 1, 1])
    
    if center:
        ax.set_xlim(-plotWidth, plotWidth)
        ax.set_ylim(-plotWidth, plotWidth)
        ax.set_zlim(-plotWidth, plotWidth)
    else:
        ax.set_xlim(-320, 320)
        ax.set_ylim(-320, 320)
        ax.set_zlim(-250, -250+640)
    if title != 0:
        ax.set_title(title)
    

    ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.tight_layout()

def Plot_Clouds_PyVista(points_list, pointSize=1.0):
    plotter = pv.Plotter(shape=(1, 3))  # Create a single plotter with 3 subplots
    
    for i, points in enumerate(points_list):
        cloud = pv.PolyData(points.T)
        print(f'Displaying {points.shape[1]} points')

        # Map z-values to the HSV colormap
        z_values = points[2]  
        cloud.point_data['z'] = z_values
        cloud.point_data.set_array(z_values, 'z')

        plotter.subplot(0, i)  # Select subplot
        plotter.set_background('gray')
        plotter.add_mesh(cloud, scalars='z', cmap='coolwarm', point_size=pointSize)
    
    plotter.show()

def Home_On_Hub(cloud, floorOffset=2, radius=100):
    """
    Filters out points outside a specified radius from the centroid of the hub
    and shifts points to the XY origin.
    
    Parameters:
        cloud (numpy.ndarray): 3xN array of point coordinates.
        floorOffset (float): Minimum z-value to consider a point part of the hub.
        radius (float): Radius in the XY plane to keep points around the hub centroid.

    Returns:
        numpy.ndarray: Filtered and shifted 3xN array of point coordinates.
    """
    # Identify hub points (z >= z_threshold)
    hub_mask = cloud[2] >= (np.min(cloud[2]) + floorOffset)
    hub_points = cloud[:, hub_mask]
    
    if hub_points.shape[1] == 0:
        raise ValueError("No points found above the z_threshold.")

    # Calculate the centroid of the hub in the XY plane
    centroid_x = np.mean(hub_points[0])
    centroid_y = np.mean(hub_points[1])
    print(f"Hub centroid: (X: {centroid_x:.1f}, Y: {centroid_y:.1f})")
    
    # Calculate distances of all points from the hub centroid in the XY plane
    distances = np.sqrt((cloud[0] - centroid_x)**2 + (cloud[1] - centroid_y)**2)
    
    # Create a mask for points within the radius
    within_radius_mask = distances <= radius
    
    # Filter the points
    filtered_points = cloud[:, within_radius_mask]
    print(f"Points around hub: {filtered_points.shape[1]}")

    # Shift the points to the XY origin
    filtered_points[0] -= centroid_x
    filtered_points[1] -= centroid_y
    
    return filtered_points

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

# import imageio.v3 as iio
# import os

def create_gif_from_pngs(folder_path, output_gif_path="output.gif", frame_rate=1):
    """
    Create an animated GIF from all PNG files in a specified folder.
    
    Parameters:
    - folder_path (str): Path to the folder containing PNG files
    - output_gif_path (str): Path where the GIF will be saved (default: "output.gif")
    - frame_rate (float): Frames per second for the GIF (default: 1 FPS)
    """
    # Check if folder exists
    if not os.path.exists(folder_path):
        print(f"Error: Folder '{folder_path}' does not exist")
        return
    
    # Get all PNG files in the folder
    png_files = [f for f in os.listdir(folder_path) if f.lower().endswith('.png')]
    if not png_files:
        print(f"Error: No PNG files found in '{folder_path}'")
        return
    
    # Sort files alphabetically to maintain consistent order
    png_files.sort()
    
    # Calculate duration per frame (in seconds)
    duration = 1.0 / frame_rate
    
    # Read all images into a list
    images = []
    for png_file in png_files:
        file_path = os.path.join(folder_path, png_file)
        try:
            image = iio.imread(file_path)
            images.append(image)
            print(f"Loaded: {png_file}")
        except Exception as e:
            print(f"Error loading {png_file}: {str(e)}")
    
    # Save as GIF
    if images:
        # Add reverse sequence (excluding the last frame to avoid double-playing it)
        full_sequence = images + images[-2::-1]
        try:
            iio.imwrite(
                output_gif_path,
                full_sequence,
                duration=duration,
                loop=0  # 0 means loop forever
            )
            print(f"GIF saved as '{output_gif_path}' with {frame_rate} FPS "
                  f"({len(images)} frames)")
        except Exception as e:
            print(f"Error saving GIF: {str(e)}")
    else:
        print("No images were loaded successfully")

if __name__ == "__main__":
    # scan1 = Load_Scan('Keyence Scans\Week Trial\FlatPlate\8201\8-201_1.csv', maxPoints=10000000, cutOff=[-150,-131])
    # scan1 = Home_On_Hub(scan1, radius=150)
    # Plot_Cloud(scan1, plotNum=1)
    scan1 = Load_Scan(r'arm.csv', maxPoints=11000000, cutOff=[-500, 500])
    Plot_Cloud_PyVista(scan1, pointSize=0.2)

    # create_gif_from_pngs(
    #     folder_path=r"c:\Users\Alex R. Williams\Documents\School\Capstone - 475\Cap Python Code",
    #     output_gif_path="point_cloud_animation.gif",
    #     frame_rate=1
    # )

    # scan1 = Load_Sim_Scan(r'ARM_simScan0a.txt')
    # scan2 = Load_Sim_Scan(r'ARM_simScan0b.txt')
    # scan3 = Load_Sim_Scan(r'ARM_simScan0c.txt')

    # Plot_Clouds_PyVista([scan1, scan2, scan3], pointSize=2)


    plt.show()
