''' MAKE YOUR OWN COPY. DON'T PLAY WITH THIS SPAGHETTI
    Capitalized functions are written or adapted from AI
    Lower-case function are copy-paste from AI              
'''
import numpy as np
import open3d as o3d
from psutil import Process, HIGH_PRIORITY_CLASS
from os import getpid
import pyvista as pv
from functools import reduce

# Set current Python process to high priority
inst = Process(getpid())
inst.nice(HIGH_PRIORITY_CLASS)

# Data management
def Load_Scan(filename, maxPoints, cutOff, correction=True):
    ''' Loads real data from rectangular CSV and rejects outliers 
        with cutOff and applies scaling factors according to KEYENCE 
        LJ-S640 measurement window. Uniformly samples raw scan to 
        limit array size to maxPoints 
    '''
    data = np.loadtxt(filename, delimiter=',', dtype=np.float64)
    numProfiles, numPoints = data.shape
    print(f'Points per Profile: {numPoints}')
    print(f'Profiles: {numProfiles}')
    print(f'Raw Points: {numPoints * numProfiles}')

    # Precompute x, y indices and ravel
    profile_indices = np.arange(numProfiles) - numProfiles / 2
    point_indices = (np.arange(numPoints) - numPoints / 2) * 0.2  # Scale applied here
    x, y = np.meshgrid(profile_indices, point_indices, indexing='ij')
    x = x.ravel()
    y = y.ravel()

    # Flatten z-values and filter by cutOff
    z = data.ravel() * 10
    valid_mask = (z >= cutOff[0]) & (z <= cutOff[1])
    x = x[valid_mask]
    y = y[valid_mask]
    z = z[valid_mask]

    # xScale correction
    if correction:
        print('Applying xScale correction')
        xScaleSlope = ((579 - 640) / numPoints) / (1116 - 936)
        scale_factors = np.where(z <= 0, 0.2, 0.2 + xScaleSlope*z) # default if <0.2, corrected if >0.2
        x *= scale_factors
    else:
        x *= 0.2

    # Packaged coordinates
    points = np.stack((x, y, z), axis=0)
    print(f'Valued Points: {points.shape[1]}')

    # Downsample if too big
    if points.shape[1] > maxPoints:
        sampled_indices = np.linspace(0, points.shape[1] - 1, maxPoints, dtype=int)
        points = points[:, sampled_indices]

    print(f'Sampled Points: {points.shape[1]}\n')
    return points

def Load_Sim_Scan(filename, maxPoints):
    ''' Loads synthetic data from txt. Uniformly samples raw scan 
        to limit array size to maxPoints 
    '''
    data = np.loadtxt(filename, skiprows=1)#[::10]   # Take every 10th point
    x = data[:,0]
    y = data[:,1]
    z = data[:,2]
    cloud = np.array([x, y, z])
    print(f'Valued Points: {cloud.shape[1]}')

    # Disable writing to cloud0
    cloud0 = np.copy(cloud) 
    cloud0.flags.writeable = False  

    # Downsample points if necessary
    if cloud.shape[1] > maxPoints:
        sampled_indices = np.linspace(0, cloud.shape[1] - 1, maxPoints, dtype=int)
        cloud = cloud[:, sampled_indices]

    print(f'Sampled Points: {cloud.shape[1]}\n')
    return cloud0, cloud

def Plot_Cloud_PyVista(cloud, pointSize=1.0):
    ''' Displays cloud in blocking window
    '''
    cloudPV = pv.PolyData(cloud.T)
    print(f'\nDisplaying {cloud.shape[1]} points')

     # Map z to colormap
    z = cloud[2] 
    cloudPV.point_data['z'] = z
    cloudPV.point_data.set_array(z, 'z')

    # Create plot
    plotter = pv.Plotter()
    plotter.set_background('gray')
    plotter.add_mesh(cloudPV, scalars='z', cmap='coolwarm', point_size=pointSize)
    plotter.show()

def Numpy_to_Open3D(cloud):
    """Converts a 3xN numpy array to an Open3D point cloud."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud.T)  # Transpose to Nx3
    return pcd

def Open3D_to_Numpy(pcd):
    """Converts an Open3D point cloud to a 3xN numpy array."""
    return np.asarray(pcd.points).T

# Basic 3D operations
def Rotate(cloud, axis, angle):
    """ Rotate a cloud around the specified axis by the 
        angle given in degrees.
    """
    angle = np.radians(angle)
    
    # provide rotation matrix
    if axis == 'x':
        R = np.array([[1, 0, 0],
                      [0, np.cos(angle), -np.sin(angle)],
                      [0, np.sin(angle), np.cos(angle)]])    
    elif axis == 'y':
        R = np.array([[np.cos(angle), 0, np.sin(angle)],
                      [0, 1, 0],
                      [-np.sin(angle), 0, np.cos(angle)]])
    elif axis == 'z':
        R = np.array([[np.cos(angle), -np.sin(angle), 0],
                      [np.sin(angle), np.cos(angle), 0],
                      [0, 0, 1]])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'.")

    return np.dot(R, np.array(cloud))

def Random_Translate(cloud, alpha):
    ''' Random translate cloud in x, y, and z 
        directions, scaled by alpha
    '''
    xStep = np.random.uniform(-alpha, alpha)
    yStep = np.random.uniform(-alpha, alpha)
    zStep = np.random.uniform(-alpha, alpha)

    translation_vector = np.array([xStep, yStep, zStep])
    
    return cloud + translation_vector[:, np.newaxis]

def Random_Rotate(cloud, alpha):
    ''' Randomly rotate some angle given in degrees aboout
        some randomly oriented axis
    '''
    # Generate a random axis of rotation (3D unit vector)
    axis = np.random.randn(3)  # Random 3D vector
    axis /= np.linalg.norm(axis)  # Normalize to make it a unit vector
    
    # Generate a random angle scaled by alpha (in radians)
    angle = np.random.uniform(-alpha, alpha) * np.pi / 180  # Convert degrees to radians
    
    # Rodrigues' rotation formula to compute the rotation matrix
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)
    ux, uy, uz = axis  # Components of the rotation axis
    
    # Rotation matrix
    R = np.array([[cos_theta + ux**2 * (1 - cos_theta), ux * uy * (1 - cos_theta) - uz * sin_theta, ux * uz * (1 - cos_theta) + uy * sin_theta],
                  [uy * ux * (1 - cos_theta) + uz * sin_theta, cos_theta + uy**2 * (1 - cos_theta), uy * uz * (1 - cos_theta) - ux * sin_theta],
                  [uz * ux * (1 - cos_theta) - uy * sin_theta, uz * uy * (1 - cos_theta) + ux * sin_theta, cos_theta + uz**2 * (1 - cos_theta)]])
    
    return np.dot(R, cloud)

def Rotation_to_Zaxis(normal):
    """ Computes and returns the rotation matrix to align 
        a given normal vector with the z-axis.
    """
    # Normalize the input vector
    normal = np.array(normal[:3])
    normal = normal / np.linalg.norm(normal)
    
    # Define the target z-axis
    z_axis = np.array([0, 0, 1])

    # Compute the cross product and dot product
    v = np.cross(normal, z_axis)
    cos_theta = np.dot(normal, z_axis)
    sin_theta = np.linalg.norm(v)

    # Handle the case where the normal is already aligned with the z-axis
    if np.isclose(sin_theta, 0):
        return np.eye(3) if cos_theta > 0 else -np.eye(3)

    # Skew-symmetric cross-product matrix
    v_x, v_y, v_z = v
    K = np.array([[0, -v_z, v_y],
                  [v_z, 0, -v_x],
                  [-v_y, v_x, 0]])
        

    # Rodrigues' rotation formula, adapted
    R = np.eye(3) + K + (1 - cos_theta)*np.dot(K, K)/sin_theta

    # Prevent distortion of the rest of the vectors in the cloud
    i = 0
    while True:
        R = enforce_rotation_properties(R)

        if np.allclose(np.dot(R.T, R), np.eye(3)) and np.isclose(np.linalg.det(R), 1.0):
            return R
        if i > 10:
            return R
        i += 1

def make_orthogonal(matrix):
    """ Enforces orthogonality of a given matrix using 
        Singular Value Decomposition (SVD).
    """
    U, _, Vt = np.linalg.svd(matrix)
    R = np.dot(U, Vt)
    
    # Ensure determinant is 1 (right-handed system)
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = np.dot(U, Vt)
    
    return R

def enforce_rotation_properties(matrix):
    """ Ensures a matrix is a valid rotation matrix by checking 
        orthogonality and determinant. Prevents cloud warping
    """
    # Check orthogonality
    if not np.allclose(np.dot(matrix.T, matrix), np.eye(3), atol=1e-6):
        matrix = make_orthogonal(matrix)
    
    # Check determinant
    if not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-6):
        U, _, Vt = np.linalg.svd(matrix)
        matrix = np.dot(U, Vt)
    
    return matrix

def Check_Scaling(base_cloud, transformed_cloud):
    ''' Checks if a tranformed cloud has been warped compared
        to its unaltered state'''
    base_cloud = base_cloud - np.mean(base_cloud, axis=1)[:, np.newaxis]
    transformed_cloud = transformed_cloud - np.mean(transformed_cloud, axis=1)[:, np.newaxis]
    original_norm = np.linalg.norm(base_cloud, axis=0).mean()
    transformed_norm = np.linalg.norm(transformed_cloud, axis=0).mean()
    
    if np.isclose(original_norm, transformed_norm):
        # print("No distortion detected")
        pass
    else:
        print("Scaling distortion detected") 

def Normal_of_Rotated_Plane(axis, angle):
    """ Returns the normal vector of a standard plane after being rotated 
        a given angle in degrees about a specified axis.
    """
    angle_rad = np.radians(angle)
    
    # Standard normal vectors and rotations for each default plane
    if axis == 'x':
        initial_normal = np.array([0, 0, 1])  # XY plane's normal
        R = np.array([[1, 0, 0],
                      [0, np.cos(angle_rad), -np.sin(angle_rad)],
                      [0, np.sin(angle_rad), np.cos(angle_rad)]])
    elif axis == 'y':
        initial_normal = np.array([0, 0, 1])  # YX plane's normal
        R = np.array([[np.cos(angle_rad), 0, np.sin(angle_rad)],
                      [0, 1, 0],
                      [-np.sin(angle_rad), 0, np.cos(angle_rad)]])
    elif axis == 'z':
        initial_normal = np.array([1, 0, 0])  # YZ plane's normal
        R = np.array([[np.cos(angle_rad), -np.sin(angle_rad), 0],
                      [np.sin(angle_rad), np.cos(angle_rad), 0],
                      [0, 0, 1]])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'.")

    # Apply rotation
    rotated_normal = np.dot(R, np.array(initial_normal))

    # Normalize the result and set small values to 0
    normalized_normal = rotated_normal / np.linalg.norm(rotated_normal)
    normalized_normal[np.abs(normalized_normal) < 1e-12] = 0.0

    return normalized_normal

# Low-level feature detection
def Find_Ledges_Along_Normal(cloud, normal, ledgeThreshold, shortLedge, closeLedges):
    """ Groups points into ledges based on proximity along a direction parallel to the normal.

        :param cloud: List of arrays representing the x, y, and z coordinates of points.
        :param normal: The normal vector defining the direction for grouping.
        :param ledgeThreshold: Threshold for grouping points into the same ledge.
        :param shortLedge: Minimum size of a ledge to include
        :param closeLedges: Threshold for merging close ledges.
        :return: (bigLedges, bigLedgeAvgs), ledges sorted by average location along normal.
    """
    normal = normal / np.linalg.norm(normal)  # Ensure normal is a unit vector

    # Compute the projected distance along the normal direction
    projected_dist = np.dot(normal, np.vstack(cloud))

    # Initialize first ledge
    first_dist = projected_dist[0]
    currentPoint = np.array([[cloud[0][0]], [cloud[1][0]], [cloud[2][0]]])

    ledges = [currentPoint]
    numLedges = 1
    ledgeAvgs = [first_dist]

    for i in range(1, len(cloud[0])):
        foundSimilarLedge = False
        proj_dist = projected_dist[i]
        currentPoint = np.array([[cloud[0][i]], [cloud[1][i]], [cloud[2][i]]])

        for k in range(numLedges):
            avg = ledgeAvgs[k]

            if proj_dist > avg - ledgeThreshold and proj_dist < avg + ledgeThreshold:
                ledges[k] = np.column_stack((ledges[k], currentPoint))
                ledgeAvgs[k] = avg - (avg - proj_dist) / (ledges[k].size)
                foundSimilarLedge = True
                break

        if not foundSimilarLedge:
            ledges.append(currentPoint)
            ledgeAvgs.append(proj_dist)
            numLedges += 1

    # Filter out small ledges
    total_points = sum(len(ledge[0]) for ledge in ledges)
    
    bigLedges = []
    bigLedgeAvgs = []
    for i in range(len(ledges)):
        ledge_size = len(ledges[i][0])  # Number of points in the current ledge
        if ledge_size > total_points * shortLedge:  # Compare to a fraction of the total points
            bigLedges.append(ledges[i])
            bigLedgeAvgs.append(ledgeAvgs[i])

    """ Check if ledges need stitching to form a continuous plane. If stitched once, check again """
    closeBigLedges = []
    needStitch = False
    for i in range(len(bigLedges)):
        for j in range(i + 1, len(bigLedges)):
            if abs(bigLedgeAvgs[i] - bigLedgeAvgs[j]) <= closeLedges:
                closeBigLedges.append((i, j))
                needStitch = True

    if needStitch:
        bigLedges, bigLedgeAvgs = Stitch_Close_Ledges(bigLedges, bigLedgeAvgs, normal, closeBigLedges)

        closeBigLedges = []
        needStitch = False
        for i in range(len(bigLedges)):
            for j in range(i + 1, len(bigLedges)):
                if abs(bigLedgeAvgs[i] - bigLedgeAvgs[j]) <= closeLedges:
                    closeBigLedges.append((i, j))
                    needStitch = True

        bigLedges, bigLedgeAvgs = Stitch_Close_Ledges(bigLedges, bigLedgeAvgs, normal,  closeBigLedges)
    
    # Sort ledges by the projected distance along the normal direction
    sorted_indices = np.argsort(bigLedgeAvgs)
    bigLedges = [bigLedges[i] for i in sorted_indices]
    bigLedgeAvgs = [bigLedgeAvgs[i] for i in sorted_indices]

    return bigLedges, bigLedgeAvgs

def Stitch_Close_Ledges(bigLedges, bigLedgeAvgs, normal, closeBigLedges):
    """ Stitch together ledges that were mistakenly separated in the initial sorting.

        :param bigLedges: List of large ledges (arrays of points).
        :param bigLedgeAvgs: List of average projected distances along the normal for each ledge.
        :param closeBigLedges: List of (i, j) pairs of ledges that need to be stitched.
        :return: Tuple (stitchedBigLedges, stitchedBigLedgeAvgs) with merged ledges and their updated averages.
    """
    stitchedBigLedges = []
    stitchedBigLedgeAvgs = []
    stitched_indices = set()  # Tracks indices that have been merged

    for i in range(len(bigLedges)):
        if i in stitched_indices:
            continue  # Skip if this ledge was already stitched

        involved_pairs = [pair for pair in closeBigLedges if i in pair]  # Find all pairs this ledge is part of
        
        if involved_pairs:
            # Merge all ledges connected to this one
            combined_ledge = bigLedges[i]
            combined_avg = bigLedgeAvgs[i]
            stitched_indices.add(i)

            for pair in involved_pairs:
                _, j = pair if pair[0] == i else pair[::-1]  # Get the other index
                if j not in stitched_indices:
                    combined_ledge = np.column_stack((combined_ledge, bigLedges[j]))
                    stitched_indices.add(j)

            # Compute the new average projected distance along the normal
            combined_avg = np.mean(np.dot(combined_ledge.T, normal))

            # Add the combined ledge and its average
            stitchedBigLedges.append(combined_ledge)
            stitchedBigLedgeAvgs.append(combined_avg)
                
        else:  # If the index isn't part of any pair, add it as-is
            stitchedBigLedges.append(bigLedges[i])
            stitchedBigLedgeAvgs.append(bigLedgeAvgs[i])
    
    return stitchedBigLedges, stitchedBigLedgeAvgs

def Cloud_Expected_Normal_Filter(cloud, expected_normal, angle_threshold=10):
    """ Discards points whose estimated normal is outside 
        `angle_threshold` degrees of `expected_normal`
    """
    # Generate polygons from points and store normals
    pcd = Numpy_to_Open3D(cloud)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
    normals = np.asarray(pcd.normals)
    
    # apply direction filter
    cos_theta = np.dot(normals, expected_normal) / (np.linalg.norm(normals, axis=1) * np.linalg.norm(expected_normal))
    angles = np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))
    valid_indices = np.where(angles < angle_threshold)[0]
    
    return Open3D_to_Numpy(pcd.select_by_index(valid_indices))

# Open3D feature processing
def RANSAC_Broadface_Plane(cloud, expected_normal, max_angle_deviation=10, distance_threshold=1.0):
    """ Fits a plane using RANSAC, ignoring surfaces whose normals deviate too much from 
        `expected_normal`. Typically defaults to surface with largest number of points.
        Used to find a prominent surface on the axle in the scan's local frame which should 
        be approx normal to the axle's z axis
    """
    pcd = Numpy_to_Open3D(cloud)
    max_angle_rad = np.radians(max_angle_deviation)
    
    for _ in range(1000):
        plane_model,_ = pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=3, num_iterations=10)
        normal = np.array(plane_model[:3])
        dot_product = np.dot(normal, expected_normal) / (np.linalg.norm(normal) * np.linalg.norm(expected_normal))
        angle = np.arccos(np.clip(dot_product, -1.0, 1.0))
        
        if angle < max_angle_rad:
            normal[np.abs(normal) < 1e-11] = 0.0
            plane_model[np.abs(plane_model) < 1e-11] = 0.0
            return normal, np.array(plane_model)
    
    print("Warning: No valid plane found within angle constraint.")
    return None, None

def RANSAC_ICP_Best_Fit(adjCloud, baseCloud, threshold=10.0, max_iteration=50):
    """ Aligns a source point cloud to a base point cloud using the ICP algorithm.

        Parameters:
            adjCloud (o3d.geometry.PointCloud): The cloud to align (adjust).
            baseCloud (o3d.geometry.PointCloud): The cloud to align to.
            threshold (float): Distance threshold for considering correspondences.
            max_iteration (int): Maximum number of iterations for the ICP algorithm.
        Returns:
            o3d.geometry.PointCloud: The transformed source point cloud.
            np.ndarray: The 4x4 transformation matrix.
            float: Final overlap score (higher is better, ranges from 0 to 1).
            float: Final local error score (lower is beter)
    """
    source_cloud = o3d.geometry.PointCloud()
    source_cloud.points = o3d.utility.Vector3dVector(adjCloud.T)
    source_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    target_cloud = o3d.geometry.PointCloud()
    target_cloud.points = o3d.utility.Vector3dVector(baseCloud.T)
    target_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Convert point clouds to Open3D format (if not already)
    if not isinstance(source_cloud, o3d.geometry.PointCloud):
        raise ValueError("source_cloud must be an Open3D PointCloud")
    if not isinstance(target_cloud, o3d.geometry.PointCloud):
        raise ValueError("target_cloud must be an Open3D PointCloud")
    
    # Initial alignment guess (identity matrix)
    initial_transform = np.eye(4)
    
    # Perform ICP registration
    reg_result = o3d.pipelines.registration.registration_icp(
        source_cloud,
        target_cloud,
        threshold,
        initial_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration)
    )
    
    return source_cloud.transform(reg_result.transformation), reg_result.transformation, reg_result.fitness, reg_result.inlier_rmse

# High-level feature alignment
def Broadface_to_Z(cloud, cloud_expected_normal, degTolerance=10, show=False):
    ''' Finds the lowest surface approx normal to axle's z axis in cloud's local
        frame and rotates it to be aligned with global z axis. Then apply that 
        rotation to the entire cloud. Many debugging steps commented out.
    '''
    print("\nExpected Broadface Normal: ", cloud_expected_normal)
    cloud_broadface_surfaces = Cloud_Expected_Normal_Filter(cloud, cloud_expected_normal, angle_threshold=degTolerance)
    # Plot_Cloud_PyVista(cloud_broadface_surfaces, pointSize=2)
    cloud_broadface_ledges,_ = Find_Ledges_Along_Normal(cloud_broadface_surfaces, normal=cloud_expected_normal, ledgeThreshold=1, shortLedge=0.01, closeLedges=10)
    cloud_broadface_surfaces = np.hstack(cloud_broadface_ledges)
    # Plot_Cloud_PyVista(cloud_broadface_surfaces, pointSize=2)
    # for i in range(len(cloud_broadface_ledges)):
        # Plot_Cloud_PyVista(cloud_broadface_ledges[i], pointSize=5)

    cloud_lowest_broadface_ledge = cloud_broadface_ledges[0]
    cloud_bracket_normal, cloud_lowest_broadface_plane = RANSAC_Broadface_Plane(cloud_broadface_ledges[0], cloud_expected_normal)
    print("RANSAC's Broadface Plane: ", cloud_lowest_broadface_plane)

    # align lowest broadface surface (assumed the bracket plane) with the global Z
    # then apply resulting transformation (composed of all R_to_Z matrices) to entire cloud1
    cloud_broadface = np.copy(cloud_lowest_broadface_ledge)
    # print('Showing lowest broadface surface')
    # Plot_Cloud_PyVista(cloud_broadface)
    cloud_Rs_to_Z = [] # all the rotations used to get cloud1's bracket aligned with z
    i = 0
    while True:
        # print(bracketNormal)
        R_to_Z = Rotation_to_Zaxis(cloud_bracket_normal)
        cloud_Rs_to_Z.append(R_to_Z)   
        cloud_bracket_normal = np.dot(R_to_Z, cloud_bracket_normal)
        # print(f"Aligned Normal {i}:", bracketNormal0)
        cloud_broadface = np.dot(R_to_Z, cloud_broadface)
        skew = abs(cloud_bracket_normal[0] + cloud_bracket_normal[1])
        # print('skew: ', skew)
        # print(i)
        Check_Scaling(cloud_lowest_broadface_ledge, cloud_broadface)
        i += 1
        if skew < 1e-6 or i >= 10:
            # print('Showing leveled lowest broadface surface')
            # Plot_Cloud_PyVista(cloud_broadface)
            break

    total_cloud_R_to_Z = reduce(np.matmul, reversed(cloud_Rs_to_Z))
    if show:
        Plot_Cloud_PyVista(np.dot(total_cloud_R_to_Z, cloud), pointSize=2)

    return np.dot(total_cloud_R_to_Z, cloud), total_cloud_R_to_Z

def Align_Clouds(baseCloud, adjCloud, radius=1.0, maxInterations=1000, T=0, show=False):
    ''' Aligns to clouds using RANSAC ICP algorithm if no transformation matrix provided
        If transformation is provided, simply applies it to adjCloud'''
    if T == 0:
        _, T, a, b  = RANSAC_ICP_Best_Fit(adjCloud=adjCloud, baseCloud=baseCloud, threshold=radius, max_iteration=maxInterations)
        T = T[:-1, :]
        print(f'Overlap Score: {a} Error Score: {b}')
    
    adjCloud_aligned = np.dot(T[:, :3], adjCloud) + T[:, 3][:, np.newaxis]
    cloud = np.hstack((baseCloud, adjCloud_aligned))
    if show:
        Plot_Cloud_PyVista(cloud)

    return adjCloud_aligned, cloud, T

if __name__ == "__main__":
    alphaDeg = 10       # degrees
    alphaDist = 15      # mm
    
    # Load scans
    print('Loading Scans')
    cloud1_ground, cloud1 = Load_Sim_Scan(filename=r'Simulated Scans\simScan0c.txt', maxPoints=1000000)
    cloud1_expected_normal = Normal_of_Rotated_Plane(axis='x', angle=0)
        # Apply rotation and translation for uncertain scanner view
    cloud1 = Random_Rotate(cloud1, alpha=alphaDeg)     # degrees
    cloud1 = Random_Translate(cloud1, alpha=alphaDist) # mm
    # Plot_Cloud_PyVista(cloud1, pointSize=2)
    
    cloud2_ground, cloud2 = Load_Sim_Scan(filename=r'Simulated Scans\simScan2c.txt', maxPoints=1000000)
    cloud2 = Rotate(cloud2, axis='y', angle=20)
    cloud2_expected_normal = Normal_of_Rotated_Plane(axis='y', angle=20)
        # Apply rotation and translation for uncertain scanner view
    cloud2 = Random_Rotate(cloud2, alpha=alphaDeg)
    cloud2 = Random_Translate(cloud2, alpha=alphaDist)
    # Plot_Cloud_PyVista(cloud2, pointSize=2)
    
    cloud3_ground, cloud3 = Load_Sim_Scan(filename=r'Simulated Scans\simScan1c.txt', maxPoints=1000000)
    cloud3 = Rotate(cloud3, axis='y', angle=-20)
    cloud3_expected_normal = Normal_of_Rotated_Plane(axis='y', angle=-20)
        # Apply rotation and translation for uncertain scanner view
    cloud3 = Random_Rotate(cloud3, alpha=alphaDeg)
    cloud3 = Random_Translate(cloud3, alpha=alphaDist)
    # Plot_Cloud_PyVista(cloud3, pointSize=2)
    
    # Align all scans with z axis
    cloud1, R1 = Broadface_to_Z(cloud1, cloud1_expected_normal, degTolerance=alphaDeg)
    cloud2, R2 = Broadface_to_Z(cloud2, cloud2_expected_normal, degTolerance=alphaDeg)
    cloud3, R3 = Broadface_to_Z(cloud3, cloud3_expected_normal, degTolerance=alphaDeg)
    # Plot_Cloud_PyVista(np.hstack((cloud1, cloud2, cloud3)), pointSize=2)

        
    # # RANSAC scans 1 and 2
    print(f'\nRANSAC scan 2 into 1, r={alphaDist}mm')
    cloud2_aligned, cloud1_2, T_12 = Align_Clouds(cloud1, cloud2, radius=alphaDist)
    print('RANSAC scan 2 into 1, r=1mm')
    cloud2_aligned, cloud1_2, T_12 = Align_Clouds(cloud1, cloud2_aligned, radius=1)
    print('RANSAC scan 2 into 1, r=0.1mm')
    cloud2_aligned, cloud1_2, T_12 = Align_Clouds(cloud1, cloud2_aligned, radius=0.1)
    Check_Scaling(cloud2_ground, cloud2_aligned)

    #     # Currently sometimes ends a bit off when alphaDist is high, likely because initial orientation
    #         # is separated from global best alignment by a local best alignment, which ICP falls into
    #         # and can't tunnel out of. 
    #     # After initial alignment, add a check with radius=1mm to align everything except the hub.
    #     # cut out hub using simple cutOff from highest point and store each to new clouds
    #     # align those clouds and apply required T to uncut clouds by passing T into Align_Clouds()

    # # RANSAC scans _12 and 3
    print(f'\nRANSAC scan 3 into _12, r={alphaDist}mm')
    cloud3_aligned, cloud1_2_3, T_123 = Align_Clouds(cloud1_2, cloud3, radius=alphaDist)
    print('RANSAC scan 3 into _12, r=1mm')
    cloud3_aligned, cloud1_2_3, T_123 = Align_Clouds(cloud1_2, cloud3_aligned, radius=1.0)
    print('RANSAC scan 3 into _12, r=0.1mm')
    cloud3_aligned, cloud1_2_3, T_123 = Align_Clouds(cloud1_2, cloud3_aligned, radius=0.1, show=True)
    Check_Scaling(cloud3_ground, cloud3_aligned)
