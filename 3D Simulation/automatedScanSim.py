import numpy as np
import pyvista as pv
import matplotlib.pyplot as plt
from time import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from psutil import Process #, HIGH_PRIORITY_CLASS
from os import getpid
import open3d as o3d
import pointCloudMerge

# Set current Python process to high priority. THIS WILL BRICK YOUR COMPUTER UNTIL COMPLETED. Disable to avoid
# inst = Process(getpid())
# inst.nice(HIGH_PRIORITY_CLASS)

def Scan_Profile_Full_Functionality(mesh, meshBounds, alpha=0, beta=0,  numPoints=10, profileNum=0, render=False, noise=False, curved=False):
    if curved:
        # if curved, alpha will be used to calculate position on rail. beta is not used
        # if not curved, alpha is still used to calculate position on rail, and beta is used to calculate the scan angle of the profile 
        beta = alpha

    ray_length = 1000        # Maximum distance the scanner can see subject
    r = 1000                # Radius of track scanner moves on.
    sub_to_scnr_Dist = 400  # Closest the scanner can be to subject
    mmDecPrecision = 2  # truncates to ## decimal places after mm. 1- 100 micron, 2- 10 micron, 3- 1 micron
    accuracy = 0.200    # mm. Scanner's accuracy specification. Assumed, accuracy = z_95 * standardDeviation
    z_CI = 1.96         # z-score for 95% CI

    # x,y,z coordinate of scanner for particular profile. x-horizontal, y-depth, z-vertical
    x_scnr = meshBounds[0] + r*np.sin(alpha)
    y_scnr = meshBounds[3] + sub_to_scnr_Dist - r*(1-np.cos(alpha))
    z_scnr = meshBounds[4]
    view = 60 #28.85    # degrees. +/- scanner field of view from horizontal

    # Define start and stop points of each ray
    theta = np.linspace((90-view/2)*np.pi/180, (90+view/2)*np.pi/180, numPoints)    # angle from vertical for each ray
    starts = np.array([[x_scnr, y_scnr, z_scnr]] * numPoints)                       # each ray start at scanner
    stops = np.stack([starts[:, 0] - ray_length*np.sin(theta)*np.sin(beta),         # end point of each ray
                      starts[:, 1] - ray_length*np.sin(theta)*np.cos(beta),
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
                # use innacurate distance to calcuate point's coordinates
                x_coord = start[0] - scnr_dist*np.sin(theta[i])*np.sin(beta)
                y_coord = start[1] - scnr_dist*np.sin(theta[i])*np.cos(beta)
                z_coord = start[2] + scnr_dist*np.cos(theta[i])
                # add point to lists
                profile.append([x_coord, y_coord, z_coord])
                true_profile.append(point)
    
    else:   # same as above for render=True option. Displays each profile result
        intersections = []  # Holds pyVista dataType
        
        for start, stop, i in zip(starts, stops, range(numPoints)):
            point, _ = mesh.ray_trace(start, stop, first_point=True)
            if point.shape[0] != 0:
                scnr_dist = np.linalg.norm(start - point)
                scnr_dist = np.trunc(scnr_dist*10**mmDecPrecision) / 10**mmDecPrecision
                scnr_dist = np.random.normal(scnr_dist, accuracy/z_CI)
                x_coord = start[0] - scnr_dist*np.sin(theta[i])*np.sin(beta)
                y_coord = start[1] - scnr_dist*np.sin(theta[i])*np.cos(beta)
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

if __name__ == '__main__':
    # # mesh = pv.Sphere(radius=200, theta_resolution=100, phi_resolution=100)
    # p = pv.Plotter()
    # # camera = pv.Camera()
    # # camera.position = (3000.0, 3000.0, 3000.0)
    # # camera.focal_point = (0.0, 0.0, 0.0)
    # # p.camera = camera
    # p.add_mesh(mesh, show_edges=True, color="white")
    # p.add_axes()  # Add axes for visualization
    # # p.show()

    # load in stl and orient axes
    filename = 'Profile Sensor C_Star_1.5deg-CAM_1.0deg-TOE.STL'
    mesh = pv.read(filename)
    mesh.translate([-mesh.center[0], -mesh.center[1], -mesh.center[2]])
    mesh = mesh.rotate_y(90)  
    mesh = mesh.rotate_x(90) 
    mesh = mesh.rotate_z(180)

    # store position information of stl. Used to position scanner
    x_cent = np.mean(mesh.points[:,0])
    y_cent = np.mean(mesh.points[:,1])
    z_cent = np.mean(mesh.points[:,2])
    xMax = np.max(mesh.points[:,0])
    yMax = np.max(mesh.points[:,1])
    zMax = np.max(mesh.points[:,2])
    meshBounds = np.array([x_cent, xMax, y_cent, yMax, z_cent, zMax])

    # set up how many profiles and how many points in each profile
    myProfiles = []
    myTrueProfiles = []
    numProfiles = 300
    numPoints = 300
    railView = 30                                                              # degrees. Angular span of rail
    alphas = np.linspace(railView*np.pi/180, -railView*np.pi/180, numProfiles) # for curved rail. +/- railView
    
    offset = np.pi/12
    print("Doing fixed-position scan")
    myProfiles1 = []
    myTrueProfiles1 = []
    scanWindow = 40                                                              # for stationary scanner. defines the most negative and most positive angles of the field of view
    betas = np.linspace(scanWindow*np.pi/180, -scanWindow*np.pi/180, numProfiles)  # for stationary scanner. +/- scanWindow
    with ProcessPoolExecutor() as executor:
        futures = [executor.submit(Scan_Profile_Full_Functionality, mesh, meshBounds, alpha=offset, beta=beta, numPoints=numPoints, render=False, noise=True, curved=False) for beta in betas]
        
        for future in as_completed(futures):
            profile, true_profile = future.result()
            myProfiles1.append(profile)
            myTrueProfiles1.append(true_profile)

    np_profile1 = np.empty((0,3))
    for profile in myProfiles1:
        if len(profile) > 1:
            for unit in profile:
                np_profile1 = np.vstack([np_profile1, np.array(unit).reshape(1,3)])
    
    print("done")

    # np.save('top_side_pi_over_12', np_profile1)