import numpy as np
import pyvista as pv
import matplotlib.pyplot as plt
from time import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from psutil import Process, HIGH_PRIORITY_CLASS
from os import getpid

# Set current Python process to high priority. THIS WILL BRICK YOUR COMPUTER UNTIL COMPLETED. Disable to avoid
inst = Process(getpid())
inst.nice(HIGH_PRIORITY_CLASS)

def Scan_Profile(mesh, meshBounds, offset=0, numPoints=10, profileNum=0, render=False, noise=False):
    ray_length = 1380
    sub_to_scnr_Dist = 580
    
    # Check if model is centered at origin. Adjust scan window
    if np.isclose(meshBounds[0], 0, atol=0.1):
        x_scnr = meshBounds[0] + meshBounds[1]*offset
    else:
        x_scnr = meshBounds[0] + 0.5*meshBounds[1]*offset
    y_scnr = meshBounds[3] + sub_to_scnr_Dist
    z_scnr = meshBounds[4]
    view = 28.85
    # Define start and stop points of each ray
    theta = np.linspace((90-view/2)*np.pi/180, (90+view/2)*np.pi/180, numPoints)  # angle from vertical
    # alpha = np.pi/2 - theta                     # angle from horizontal, positive from scanner to object
    starts = np.array([[x_scnr, y_scnr, z_scnr]] * numPoints)
    stops = np.stack([starts[:, 0],
                      starts[:, 1] - ray_length * np.sin(theta),
                      starts[:, 2] + ray_length * np.cos(theta)], axis=-1)
    
    profile = []        # Holds np.array(z,y,z) of geomtery calculated from simulated scanner data
    true_profile = []   # Holds np.array(x,y,z) of stl geometry at intersection
    if not render:
        for start, stop, i in zip(starts, stops, range(numPoints)):
            point, _ = mesh.ray_trace(start, stop, first_point=True)
            if point.shape[0] != 0:
                scnr_dist = np.linalg.norm(start - point)
                y_coord = start[1] - scnr_dist * np.sin(theta[i])
                z_coord = start[2] + scnr_dist * np.cos(theta[i])
                profile.append([start[0], y_coord, z_coord])
                true_profile.append(point)
    else:
        intersections = []  # Holds pyVista dataType
        
        for start, stop, i in zip(starts, stops, range(numPoints)):
            point, _ = mesh.ray_trace(start, stop, first_point=True)
            if point.shape[0] != 0:
                scnr_dist = np.linalg.norm(start - point)
                y_coord = start[1] - scnr_dist * np.sin(theta[i])
                z_coord = start[2] + scnr_dist * np.cos(theta[i])
                profile.append([start[0], y_coord, z_coord])
                true_profile.append(point)
                intersections.append(pv.PolyData(point))
                # print(f'Ray {i} intersected at {point[0]:.3f} {point[1]:.3f} {point[2]:.3f}') 
            else:
                # print(f"Ray {i} did not intersect")
                intersections.append(None)
        
        
        # Render the result
        print(f'Rendering Profile {profileNum+1}')
        p = pv.Plotter()
        p.add_mesh(mesh, show_edges=True, opacity=0.5, color="w", lighting=False, label="Test Mesh")
        for i in range(numPoints):
            p.add_mesh(pv.Line(starts[i], stops[i]), color="blue", line_width=0.1, label="Ray Segment")
            if intersections[i]:
                p.add_mesh(intersections[i], color="maroon", point_size=5, label="Intersection Points")
        p.add_axes()
        print('Finished Rendering')
        p.show()
    
    return profile, true_profile

def Scan_Profile_Curve(mesh, meshBounds, alpha=0, numPoints=10, profileNum=0, render=False, noise=False):
    ray_length = 1380       # Maximum distance the scanner can see subject
    r = 1000                # Radius of track scanner moves on.
    sub_to_scnr_Dist = 580  # Closest the scanner can be to subject

    mmDecPrecision = 2  # truncates to ## decimal places after mm. 1- 100 micron, 2- 10 micron, 3- 1 micron
    accuracy = 0.050    # mm. Scanner's accuracy specification. Assumed, accuracy = z_95 * standardDeviation
    alpha_accuracy = 1.000/r # alpha(radians) = arcLength(mm)/radius(mm)
    z_CI = 1.96         # z-score for 95% CI

    # x,y,z coordinate of scanner for particular profile. x-horizontal, y-depth, z-vertical
    x_scnr = meshBounds[0] + r*np.sin(alpha)
    y_scnr = meshBounds[3] + sub_to_scnr_Dist - r*(1-np.cos(alpha))
    z_scnr = meshBounds[4]
    view = 15    # degrees. +/- scanner field of view from horizontal

    # Define start and stop points of each ray
    theta = np.linspace((90-view/2)*np.pi/180, (90+view/2)*np.pi/180, numPoints)    # angle from vertical for each ray
    starts = np.array([[x_scnr, y_scnr, z_scnr]] * numPoints)                       # each ray start at scanner
    stops = np.stack([starts[:, 0] - ray_length*np.sin(theta)*np.sin(alpha),        # end point of each ray
                      starts[:, 1] - ray_length*np.sin(theta)*np.cos(alpha),
                      starts[:, 2] + ray_length*np.cos(theta)], axis=-1)            # alpha is constant, theta is an array w/len(numPoints)
    
    profile = []        # Holds np.array(z,y,z) of geomtery calculated from simulated scanner data
    true_profile = []   # Holds np.array(x,y,z) of true stl geometry at intersection
    if not render:
        for start, stop, i in zip(starts, stops, range(numPoints)):
            point, _ = mesh.ray_trace(start, stop, first_point=True)                    # raytrace function. Returns true coordinate of point
            if point.shape[0] != 0:
                # simulate scanner's output inaccuracy
                scnr_dist = np.linalg.norm(start - point)                               # true distance from scanner to point
                scnr_dist = np.trunc(scnr_dist*10**mmDecPrecision) / 10**mmDecPrecision # truncate the value to specified precision. Adds discretized effect
                scnr_dist = np.random.normal(scnr_dist, accuracy/z_CI)                  # simulate scanner inaccuracy on top of discretized value
                alpha_rand = np.random.normal(alpha, alpha_accuracy/z_CI)
                # use innacurate distance to calcuate point's coordinates
                x_coord = start[0] - scnr_dist*np.sin(theta[i])*np.sin(alpha_rand)
                y_coord = start[1] - scnr_dist*np.sin(theta[i])*np.cos(alpha_rand)
                z_coord = start[2] + scnr_dist*np.cos(theta[i])
                # add point to lists
                profile.append([x_coord, y_coord, z_coord])
                true_profile.append(point)
    
    else:   # same as above for render=True option. Displays each profile result
        intersections = []  # Holds pyVista dataType
        
        for start, stop, i in zip(starts, stops, range(numPoints)):
            point, _ = mesh.ray_trace(start, stop, first_point=True)
            if point.shape[0] != 0:
                # simulate scanner's output inaccuracy
                scnr_dist = np.linalg.norm(start - point)                               # true distance from scanner to point
                scnr_dist = np.trunc(scnr_dist*10**mmDecPrecision) / 10**mmDecPrecision # truncate the value to specified precision. Adds discretized effect
                scnr_dist = np.random.normal(scnr_dist, accuracy/z_CI)                  # simulate scanner inaccuracy on top of discretized value
                alpha_rand = np.random.normal(alpha, alpha_accuracy/z_CI)
                # use innacurate distance to calcuate point's coordinates
                x_coord = start[0] - scnr_dist*np.sin(theta[i])*np.sin(alpha_rand)
                y_coord = start[1] - scnr_dist*np.sin(theta[i])*np.cos(alpha_rand)
                z_coord = start[2] + scnr_dist*np.cos(theta[i])
                profile.append([x_coord, y_coord, z_coord])
                true_profile.append(point)
                intersections.append(pv.PolyData(point))
                # print(f'Ray {i} intersected at {point[0]:.3f} {point[1]:.3f} {point[2]:.3f}') 
            else:
                # print(f"Ray {i} did not intersect")
                intersections.append(None)
        
        
        # Render the result
        print(f'Rendering Profile {profileNum+1}')
        p = pv.Plotter()
        p.add_mesh(mesh, show_edges=True, opacity=0.5, color="w", lighting=False, label="Test Mesh")
        for i in range(numPoints):
            p.add_mesh(pv.Line(starts[i], stops[i]), color="blue", line_width=0.1, label="Ray Segment")
            if intersections[i]:
                p.add_mesh(intersections[i], color="maroon", point_size=5, label="Intersection Points")
        p.add_axes()
        print('Finished Rendering')
        p.show()
    
    return profile, true_profile

def set_axes_equal(ax):
        """
        Make axes of 3D plot have equal scale so that spheres appear as spheres,
        cubes as cubes, etc.

        Input
        ax: a matplotlib axis, e.g., as output from plt.gca().
        """

        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        # The plot bounding box is a sphere in the sense of the infinity
        # norm, hence I call half the max range the plot radius.
        plot_radius = 0.5*max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
        ax.set_box_aspect([1, 1, 1])

def Plot_Profile(profile, trueProfile, plotNum=10):
    # Extract valid intersection points
    valid_points = np.array([points for points in profile if points is not None])
    valid_TruePoints = np.array([points for points in trueProfile if points is not None])
    if valid_points.size == 0:
        print("No intersection points to plot.")
        return

    # print(f'points {valid_points}')    
    # Prepare data for plotting
    x = valid_points[:, 0]
    y = valid_points[:, 1]
    z = valid_points[:, 2]

    xTrue = valid_TruePoints[:, 0]
    yTrue = valid_TruePoints[:, 1]
    zTrue = valid_TruePoints[:, 2]

    # Calculate the center of the valid points
    center_x = np.mean(x)
    center_y = 0
    center_z = np.mean(z)  # Circle lies in this z-plane

    # Create a 3D scatter plot
    fig = plt.figure(num=plotNum, figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, color='red', s=1, label="Profile")
    ax.scatter(xTrue, yTrue, zTrue, color='green', s=1, label='True Profile')

    # Add labels and legend
    ax.set_title("Intersection Points in 3D")
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.set_zlabel("Z Coordinate")
    ax.legend()
    set_axes_equal(ax)
    
def Plot_Scan(profiles, trueProfiles, plotNum=100, plotTrue=False):
    # Create a 3D scatter plot
    fig = plt.figure(num=plotNum, figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Loop through profiles and true profiles
    for idx, (profile, trueProfile) in enumerate(zip(profiles, trueProfiles)):
        valid_points = np.array([points for points in profile if points is not None])
        valid_true_points = np.array([points for points in trueProfile if points is not None])
        
        if valid_points.size == 0:
            print(f"No intersections for profile {idx}.")
            continue

        # Extract data for current profile
        x = valid_points[:, 0]
        y = valid_points[:, 1]
        z = valid_points[:, 2]

        x_true = valid_true_points[:, 0]
        y_true = valid_true_points[:, 1]
        z_true = valid_true_points[:, 2]

        # Plot profile and true profile
        ax.scatter(x, y, z, s=1, color='red')
        if plotTrue:
            ax.scatter(x_true, y_true, z_true, s=1, label=f'True {idx}', color='blue')

    # Add labels, legend, and set equal axes
    ax.set_title("Intersection Points in 3D")
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.set_zlabel("Z Coordinate")
    # plt.legend()
    set_axes_equal(ax)

def Save_Scan(profiles, filename):
      # Initialize arrays to hold all x, y, and z coordinates
    xVals = []
    yVals = []
    zVals = []
    # Loop through profiles and true profiles
    for idx, profile in enumerate(profiles):
        valid_points = np.array([points for points in profile if points is not None])
        
        if valid_points.size == 0:
            print(f"No intersections for profile {idx}.")
            continue

        # Extract x, y, and z data for the current profile and append to the respective arrays
        xVals.extend(valid_points[:, 0])
        yVals.extend(valid_points[:, 2])
        zVals.extend(valid_points[:, 1])

    # Convert to numpy arrays
    xVals = np.array(xVals)
    yVals = np.array(yVals)
    zVals = np.array(zVals)



    fullScan = np.array([xVals, yVals, zVals])
    np.savetxt(filename, fullScan.T, fmt='%.6f', delimiter=' ', header='X Y Z', comments='')

if __name__ == '__main__':
    # myMesh = pv.Sphere(radius=200, theta_resolution=100, phi_resolution=100)
    # p = pv.Plotter()
    # camera = pv.Camera()
    # camera.position = (3000.0, 3000.0, 3000.0)
    # camera.focal_point = (0.0, 0.0, 0.0)
    # p.camera = camera
    # p.add_mesh(myMesh, show_edges=True, color="white")
    # p.add_axes()  # Add axes for visualization
    # p.show()

    # load in stl and orient axes
    filename = 'Profile Sensor C_Star_1.5deg-CAM_1.0deg-TOE.STL'
    myMesh = pv.read(filename)
    myMesh.translate([-myMesh.center[0], -myMesh.center[1], -myMesh.center[2]])
    myMesh = myMesh.rotate_y(90)  
    myMesh = myMesh.rotate_x(90) 
    myMesh = myMesh.rotate_z(180)

    # store position information of stl. Used to position scanner
    x_cent = np.mean(myMesh.points[:,0])
    xMax = np.max(myMesh.points[:,0])
    y_cent = np.mean(myMesh.points[:,1])
    yMax = np.max(myMesh.points[:,1])
    z_cent = np.mean(myMesh.points[:,2])
    zMax = np.max(myMesh.points[:,2])
    myMeshBounds = np.array([x_cent, xMax, y_cent, yMax, z_cent, zMax])

    # set up how many profiles and how many points in each profile
    myProfiles = []
    myTrueProfiles = []
    numProfiles = 101
    numPoints = 101
    offsets = np.linspace(-1,1,numProfiles)                                     # for linear rail. Most negative to most positive x position on stl
    railView = 30                                                               # degrees. Angular span of rail
    alphas = np.linspace(railView*np.pi/180, -railView*np.pi/180, numProfiles)  # for curved rail. +/- railView
    print('Scanning')
    start_time = time()
    # multiprocessing on CPU. This program will use ALL of your system's resources. NO MULTITASKING WHILE RUNNING
    with ProcessPoolExecutor() as executor:
        # futures = [executor.submit(Scan_Profile, myMesh, myMeshBounds, numPoints=numPoints, offset=offset, render=False) for offset in offsets]
        futures = [executor.submit(Scan_Profile_Curve, myMesh, myMeshBounds, numPoints=numPoints, alpha=alpha, render=False, noise=True) for alpha in alphas]
        for future in as_completed(futures):
            profile, true_profile = future.result()
            myProfiles.append(profile)
            myTrueProfiles.append(true_profile)

    end_time = time()
    print('Finished Scanning')
    Plot_Scan(myProfiles, myTrueProfiles, plotTrue=False)
    Save_Scan(myProfiles, 'simScan.txt')
    print(f'Duration: {end_time-start_time:.3f}')
    plt.show()
    