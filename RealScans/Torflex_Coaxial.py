import numpy as np
import matplotlib.pyplot as plt
import pyvista as pv
import open3d as o3d
from copy import deepcopy
from functools import reduce
import time

class Torsion_Arm_LJS640:
    def __init__(self, filename, view_angle_horizontal=0.0, scanType='real', cutOff=[-500, 500]):
        self.minPoints = 10000
        self.anglePoints = 1000
        self.norm_tolerance_deg = 10.0
        self.dist_tolerange_mm = 15.0
        self.exp_norm = Normal_of_Rotated_Plane(axis='y', angle=view_angle_horizontal)
        
        if scanType == 'real':
            self.max_x_width = 640
            self.min_x_width = 579
            self.reference_z_depth = 1116
            self.min_z_depth = 936
            data = np.loadtxt(filename, delimiter=',', dtype=np.float64)
            self.numProfiles, self.numPoints = data.shape
           
            # Precompute x, y indices and ravel
            profile_indices = np.arange(self.numProfiles) - self.numProfiles / 2
            point_indices = (np.arange(self.numPoints) - self.numPoints / 2) * 0.2  # Scale applied here
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
            xScaleSlope = ((self.min_x_width - self.max_x_width) / self.numPoints) / \
                                (self.reference_z_depth - self.min_z_depth)
            scale_factors = np.where(
                z <= 0,
                0.2,  # Default scale
                0.2 + xScaleSlope * z  # Adjusted scale
            )
            x *= scale_factors
            z *= 60/71

            valid_mask = (x < 290) & (x > -270) & (z >= cutOff[0]) & (z <= cutOff[1])
            x = x[valid_mask]
            y = y[valid_mask]
            z = z[valid_mask]

            # Combine into a single array
            # self.cloud = np.stack((x, y, z), axis=0)
            self.cloud = np.array([y, x, z])
            self.numPoints = self.cloud.shape[1]
        elif scanType == 'sim':
            data = np.loadtxt(filename, skiprows=1)
            self.numProfiles, self.numPoints = data.shape
            
            x = data[:,0]
            y = data[:,1]
            z = data[:,2]
            self.cloud = np.array([x, y, z])
            self.numPoints = self.cloud.shape[1]
        
    def downsample_cloud(self, maxPoints):
        if self.numPoints > maxPoints:
            sampled_indices = np.linspace(0, self.numPoints - 1, maxPoints, dtype=int)
            self.cloud = self.cloud[:, sampled_indices]
            self.numPoints = self.cloud.shape[1]

    def center_cloud(self):
        centroid_xy = [np.mean(self.cloud[0]), np.mean(self.cloud[1])]
        self.cloud = self.cloud - np.array([centroid_xy[0], centroid_xy[1], 0])[:, np.newaxis]

    def trim_cloud_z(self, cutOff=[-500, 500]):
        valid_mask = (self.cloud[2] >= cutOff[0]) & (self.cloud[2] <= cutOff[1])
        myCloud = self.cloud[:, valid_mask]
        if myCloud.shape[1] >= self.minPoints:
            self.cloud = myCloud
            self.numPoints = self.cloud.shape[1]
        else:    
            print(f'Trimming Error: Less than {self.minPoints} points in trimmed cloud. Returning untrimmed cloud')
        
    def show_cloud(self, altCloud=0):
        if isinstance(altCloud, int):
            Plot_Cloud_PyVista(self.cloud, pointSize=0.5)
        else:
            Plot_Cloud_PyVista(altCloud, pointSize=0.5)

    def fit_bar_faces(self, axialCutOff=[-500, 0], plotNum=0, show=False):
        valid_mask = (self.cloud[1] >= axialCutOff[0]) & (self.cloud[1] <= axialCutOff[1])
        barCloud = self.cloud[:, valid_mask]
        # self.show_cloud(barCloud)

        barPrimaryFaces = Cloud_Expected_Normal_Filter(barCloud, self.exp_norm, angle_threshold=6)
        # self.show_cloud(barPrimaryFaces)

        primaryLedges, primaryLedgeAvgs = Find_Ledges_Along_Normal(barPrimaryFaces, normal=self.exp_norm, ledgeThreshold=1, shortLedge=0.01, closeLedges=10)
        self.barPrimaryFace = Sort_Ledges(primaryLedges, primaryLedgeAvgs)[0][-1]
        # self.show_cloud(barPrimaryFace)

        barPrimaryPlane, _, _ = Calc_Plane(self.barPrimaryFace, plotNum=plotNum, numPoints=self.anglePoints)
        barPrimaryNormal = barPrimaryPlane[0:3]
        # print(barPrimaryPlane)

        exp_secondary_norm = Rotate(barPrimaryNormal, axis='y', angle=90.0)
        barSecondaryFaces = Cloud_Expected_Normal_Filter(barCloud, exp_secondary_norm, 3)
        # self.show_cloud(barSecondaryFaces)

        secondaryLedges, secondaryLedgeAvgs = Find_Ledges_Along_Normal(barSecondaryFaces, normal=exp_secondary_norm, ledgeThreshold=1, shortLedge=0.01, closeLedges=10)
        self.barSecondaryFace = Sort_Ledges(secondaryLedges, secondaryLedgeAvgs)[0][0]
        # self.show_cloud(barSecondaryFace)

        barSecondaryPlane, _, _ = Calc_Plane(self.barSecondaryFace, plotNum=plotNum*2, numPoints=self.anglePoints)
        barSecondaryNormal = barSecondaryPlane[0:3]
        # print(barSecondaryPlane)

        self.bar_axis = np.cross(barPrimaryNormal, barSecondaryNormal)

        if show:
            self.show_cloud(np.hstack((self.barPrimaryFace, self.barSecondaryFace)))

    def fit_spindle(self, axialCutOff=[0, 500], num_bins=20, min_points_per_bin=10, bounding_box=[None, None, None, None], show=False, plot=False):
        s_start, width, length, thickness = bounding_box
        # Initial filtering along y-axis
        valid_mask = (self.cloud[1] >= axialCutOff[0]) & (self.cloud[1] <= axialCutOff[1])
        spindleCloud = self.cloud[:, valid_mask]
        # self.show_cloud(spindleCloud)

        approx_axis = self.bar_axis
        
        # Step 1: Compute centroid and center the points
        c = np.mean(spindleCloud.T, axis=0)
        spindle_centered = spindleCloud.T #- c
        # self.show_cloud(spindle_centered.T)
        
        # Define axes and apply bounding box
        n = approx_axis / np.linalg.norm(approx_axis)
        if s_start is not None and length is not None and width is not None and thickness is not None:
            # Define perpendicular vectors u and v for the bounding box
            basis_vectors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
            dots = np.abs(basis_vectors @ n)
            min_idx = np.argmin(dots)
            e_k = basis_vectors[min_idx]
            u_box = e_k - (e_k @ n) * n
            u_box = u_box / np.linalg.norm(u_box)
            v_box = np.cross(n, u_box)
            
            # Compute coordinates in the n, u, v basis
            delta = spindle_centered - s_start
            s = delta @ n
            a = delta @ u_box
            b = delta @ v_box
            
            # Select points within the bounding box
            mask = (s >= 0) & (s <= length) & (a >= 0) & (a <= width) & (b >= 0) & (b <= thickness)
            spindle_bounded = spindle_centered[mask, :]
        else:
            spindle_bounded = spindle_centered
        
        if show:
            self.show_cloud(spindle_bounded.T)

        # Step 3: Define perpendicular vectors u and v (for later projection)
        if abs(n[0]) < min(abs(n[1]), abs(n[2])):
            u = np.array([1, 0, 0])
        elif abs(n[1]) < abs(n[2]):
            u = np.array([0, 1, 0])
        else:
            u = np.array([0, 0, 1])
        u = u - np.dot(u, n) * n  # Make u perpendicular to n
        u = u / np.linalg.norm(u)  # Normalize
        v = np.cross(n, u)  # v is perpendicular to both n and u

        # print(f'\n\nn:axis: {n}')
        # print(f'u:perp to axis: {u}')
        # print(f'v:perp to both: {v}\n')
        
        # Step 4: Project points onto the approximate axis
        t = np.dot(spindle_bounded, n)
        t_min, t_max = np.min(t), np.max(t)
        # plt.scatter(t, np.zeros_like(t), s=1)
        # plt.show()
        
        # Step 5: Bin points along the axis
        bin_edges = np.linspace(t_min, t_max, num_bins + 1)
        centers = []
        
        for i in range(num_bins):
            # Select points in the current bin
            mask = (t >= bin_edges[i]) & (t < bin_edges[i + 1])
            if np.sum(mask) < min_points_per_bin:
                continue  # Skip bins with too few points
            points_bin = spindle_bounded[mask]
            # self.show_cloud(points_bin.T)
            
            # Project points onto the u-v plane (2D coordinates)
            p_2d = np.dot(points_bin, np.array([u, v]).T)
            maxC = np.max(p_2d)
            
            # Fit a circle using least squares
            A = np.hstack([p_2d, np.ones((len(p_2d), 1))])
            b = -(p_2d[:, 0]**2 + p_2d[:, 1]**2)
            try:
                abc = np.linalg.lstsq(A, b, rcond=None)[0]
                a, b, c = abc
                # Check if the circle equation is valid (radius > 0)
                discriminant = a**2 + b**2 - 4*c
                if discriminant > 0:
                    center_2d = [-a / 2, -b / 2]
                    t_center = (bin_edges[i] + bin_edges[i + 1]) / 2
                    # Map 2D center back to 3D
                    center_3d = t_center * n + center_2d[0] * u + center_2d[1] * v
                    centers.append(center_3d)
                    radius = np.sqrt((a/2)**2 + (b/2)**2 - c)
            except np.linalg.LinAlgError:
                continue  # Skip if the least squares solution fails

            # Generate circle points
            if plot:
                theta = np.linspace(0, 2 * np.pi, 100)  # 100 points for smooth circle
                x_circle = center_2d[0] + radius * np.cos(theta)
                y_circle = center_2d[1] + radius * np.sin(theta)
                plt.plot(x_circle, y_circle, 'r-', label='Best-fit circle', linewidth=0.5)
                plt.scatter(p_2d[:, 0], p_2d[:, 1], s=1)
                plt.scatter(center_2d[0], center_2d[1])
                plt.title(f"Projected Point {c}")
                plt.xlim(-maxC, maxC)
                plt.ylim(-maxC, maxC)
                plt.axis('equal')
                plt.xlabel("u-axis")
                plt.ylabel("v-axis")
                plt.show()
        
        if len(centers) < 2:
            raise ValueError("Not enough valid circle fits to determine the axis.")
        
        # Step 6: Fit a line to the 3D centers using PCA
        centers = np.array(centers)
        c_axis = np.mean(centers, axis=0)  # Centroid of centers
        cov = np.cov(centers - c_axis, rowvar=False)
        eigvals, eigvecs = np.linalg.eigh(cov)
        axis_dir = eigvecs[:, np.argmax(eigvals)]  # Direction with largest variance
        
        # Ensure direction aligns with approximate axis
        if np.dot(axis_dir, n) < 0:
            axis_dir = -axis_dir

        self.axis_loc = c_axis
        self.spindle_axis = axis_dir
        self.spindle_cloud = spindle_bounded.T

        # visualize_axis(Numpy_to_Open3D(spindle_centered.T), c_axis, axis_dir, length=500)

    def calc_angles(self):
        B = np.array(self.bar_axis) / np.linalg.norm(self.bar_axis)
        S = np.array(self.spindle_axis) / np.linalg.norm(self.spindle_axis)

        # Step 1: Verify unit vectors (optional, assuming they are unit vectors)
        if not np.isclose(np.linalg.norm(B), 1.0) or not np.isclose(np.linalg.norm(S), 1.0):
            raise ValueError("Inputs must be unit vectors")

        # Step 2: Compute angles of B and S with global axes (in degrees)
        # Angles with x, y, z axes for Bar Axis
        self.bar_angle = np.degrees( np.array([ np.arccos(B[0]), np.arccos(B[1]), np.arccos(B[2]) ]) )
        
        # Angles with x, y, z axes for Spindle Axis
        self.spindle_angle = np.degrees( np.array([ np.arccos(S[0]), np.arccos(S[1]), np.arccos(S[2]) ]) )

        # Step 3: Compute the cross product (rotation axis)
        R = np.cross(B, S)

        # Step 4: Total angular difference
        sin_theta = np.linalg.norm(R)
        theta_deg = np.degrees(np.arcsin(sin_theta))
        self.total_angle = theta_deg
        
        # Step 5: Resolve into global x, y, z components
        Rx_deg, Ry_deg, Rz_deg = np.degrees(R)
        self.relative_angle = np.array([Rx_deg, Ry_deg, Rz_deg])

    def print_angles(self):
        np.set_printoptions(precision=4, suppress=True)
        print(f'\nBar Axis:\t{self.bar_axis}')
        print(f'Spindle Axis:\t{self.spindle_axis}')

        print(f'\n---DEGREES---\nBar Angle:\t{self.bar_angle}')
        print(f'Spindle Angle:\t{self.spindle_angle}')
        print(f'Relative Angle:\t{self.relative_angle}')
        print(f'Total Angle:\t{self.total_angle:.4f}')

# Data management
def Numpy_to_Open3D(cloud):
    """Converts a 3xN numpy array to an Open3D point cloud."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud.T)  # Transpose to Nx3
    return pcd

def Open3D_to_Numpy(pcd):
    """Converts an Open3D point cloud to a 3xN numpy array."""
    return np.asarray(pcd.points).T

def visualize_axis(pcd, c_axis, axis_dir, length=0.1):
    """
    Visualizes the point cloud with the fitted axis.
    
    Parameters:
    - pcd: Open3D PointCloud object
    - c_axis: Point on the axis
    - axis_dir: Direction of the axis
    - length: Length of the axis line for visualization
    """
    axis_points = np.array([c_axis - length * axis_dir, c_axis + length * axis_dir])
    axis_line = o3d.geometry.LineSet()
    axis_line.points = o3d.utility.Vector3dVector(axis_points)
    axis_line.lines = o3d.utility.Vector2iVector([[0, 1]])
    axis_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]])  # Red color
    o3d.visualization.draw_geometries([pcd, axis_line])

# Basic 3D operations
def Rotate(threeD_Object, axis, angle):
    """ Rotate a 3D object around the specified axis by the 
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
    
    return np.dot(R, threeD_Object)

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

def Calc_Plane(points, title=0, plotNum=0, numPoints=1000):
    # Downsample points for plane fitting, ~10,000 points is optimal
    sampled_indices = np.linspace(0, points.shape[1] - 1, numPoints, dtype=int)
    points_sampled = points[:, sampled_indices]
    print(f'Fitting plane to {points_sampled.shape[1]} points')

    # Function to fit a plane and filter outliers
    def fit_plane(points):
        """Method 1. Slow, accurate but limited maxPoints"""
        # Perform SVD to get the normal vector
        centroid = np.mean(points, axis=1, keepdims=True)
        svd = np.linalg.svd(points - centroid)
        normal_vector = svd[0][:, -1]
        d = -normal_vector.dot(centroid.flatten())

        """Method 2. Fast, Less accurate, but no limit maxPoints"""
        # # Convert to Open3D point cloud
        # centroid = np.mean(points, axis=1, keepdims=True)
        # o3d_cloud = o3d.geometry.PointCloud()
        # o3d_cloud.points = o3d.utility.Vector3dVector((points-centroid).T)
        # # RANSAC to fit a plane
        # plane, inliers = o3d_cloud.segment_plane(distance_threshold=0.1,
        #                                                     ransac_n=3,
        #                                                     num_iterations=1000)
        # if plane is None:
        #     raise ValueError("No planar feature detected in the point cloud.")
        # normal_vector = plane[0:3]
        # d = plane[3]
        plane = np.hstack((normal_vector, d))
        return plane, centroid.flatten()

    def filter_plane(points, normal_vector, iqr_scale):
        d = -normal_vector.dot(centroid.flatten())
        # Calculate orthogonal distances
        ortho_dist = np.abs(normal_vector[0] * points[0] +
                            normal_vector[1] * points[1] +
                            normal_vector[2] * points[2] + d) / np.linalg.norm(normal_vector)

        # Filter using IQR
        Q1, Q3 = np.percentile(ortho_dist, [25, 75])
        IQR = Q3 - Q1
        lower_bound = Q1 - iqr_scale * IQR
        upper_bound = Q3 + iqr_scale * IQR
        filtered_points = points[:, (ortho_dist >= lower_bound) & (ortho_dist <= upper_bound)]

        return filtered_points

    # Iterative plane fitting and filtering
    filt_points = points_sampled
    for iteration, iqr_scale in enumerate([0.5, 1.0], start=1):  # Two iterations with increasing IQR scale
        print(f"\tIteration {iteration}: filtering {filt_points.shape[1]} points")
        plane, centroid = fit_plane(filt_points)
        filt_points = filter_plane(filt_points, plane[0:3], iqr_scale)

    # Final iteration with all points, using filtered indices
    final_filt_points = filt_points
    print(f"\tIteration {iteration+1}: final {final_filt_points.shape[1]} points")
    plane_final, centroid_final = fit_plane(final_filt_points)
    normal_vector_final = plane_final[0:3]
    d_final = -normal_vector_final.dot(centroid_final.flatten())
    
    # Create a meshgrid for the final plane
    xx, yy = np.meshgrid(np.linspace(np.min(final_filt_points[0]), np.max(final_filt_points[0]), 10),
                         np.linspace(np.min(final_filt_points[1]), np.max(final_filt_points[1]), 10))
    zz = (-normal_vector_final[0] * xx - normal_vector_final[1] * yy - d_final) / normal_vector_final[2]

    # Calculate angles with x, y, and z axes
    x_angle = np.degrees(np.arccos(abs(normal_vector_final[0]) / np.linalg.norm(normal_vector_final)))
    y_angle = np.degrees(np.arccos(abs(normal_vector_final[1]) / np.linalg.norm(normal_vector_final)))
    z_angle = np.degrees(np.arccos(abs(normal_vector_final[2]) / np.linalg.norm(normal_vector_final)))

    ortho_dist_final = np.abs(normal_vector_final[0] * final_filt_points[0] +
                              normal_vector_final[1] * final_filt_points[1] +
                              normal_vector_final[2] * final_filt_points[2] + d_final) / np.linalg.norm(normal_vector_final)
    uncertainty_95 = np.std(ortho_dist_final) * 1.96    # 95% confidence
    if plotNum != 0:
        fig = plt.figure(plotNum)
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(points_sampled[0], points_sampled[1], points_sampled[2], color='red', s=1)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title('Best Fit Plane w/Filtering')
        ax.scatter(final_filt_points[0], final_filt_points[1], final_filt_points[2], color='green', s=10)

        # Plot the best-fitting plane
        ax.plot_surface(xx, yy, zz, alpha=0.5, color='yellow')
        if title:
            ax.set_title(title)
            fig.suptitle(f'x: {90 - x_angle:.3f}, y: {90 - y_angle:.3f}, z: {z_angle:.3f}')

        # Histogram of orthogonal distances
        plt.figure(10+plotNum)
        plt.hist(ortho_dist_final, bins=17, density=True)
        plt.axvline(uncertainty_95, c='red')
        plt.title(f'Distribution of Plane Error (95% Spread: ±{uncertainty_95:.2f} mm)')
        plt.xlabel('Orthogonal Distance - Point to Plane (mm)')
        plt.show()

    return plane_final, np.array([x_angle, y_angle, z_angle]), uncertainty_95

def Plot_Cloud_PyVista(points, pointSize=1.0):
    cloud = pv.PolyData(points.T)
    print(f'Displaying {points.shape[1]} points')

    # Map z-values to the HSV colormap
    z_values = points[2]  # Assuming the z-values are the third row of 'points'
    cloud.point_data['z'] = z_values
    cloud.point_data.set_array(z_values, 'z')

    # Create plotter
    plotter = pv.Plotter(off_screen=False)
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
    plotter.show(auto_close=False, interactive=True)


# Low-level feature detection
class UnionFind:
    """Simple union-find structure for merging connected components."""
    def __init__(self, size):
        self.parent = list(range(size))
    
    def find(self, x):
        if self.parent[x] != x:
            self.parent[x] = self.find(self.parent[x])
        return self.parent[x]
    
    def union(self, x, y):
        rootX = self.find(x)
        rootY = self.find(y)
        if rootX != rootY:
            self.parent[rootX] = rootY

def Find_Ledges_Along_Normal(cloud, normal, ledgeThreshold, shortLedge, closeLedges):
    """Groups points into ledges based on proximity along a direction parallel to the normal.

    :param cloud: List of arrays representing the x, y, and z coordinates of points.
    :param normal: The normal vector defining the direction for grouping.
    :param ledgeThreshold: Threshold for grouping points into the same ledge.
    :param shortLedge: Minimum size of a ledge to include (fraction of total points).
    :param closeLedges: Threshold for merging close ledges.
    :return: (bigLedges, bigLedgeAvgs), ledges sorted by average location along normal.
    """
    print('Grouping filtered surfaces into ledges')
    normal = normal / np.linalg.norm(normal)  # Ensure normal is a unit vector

    # Compute projected distances along the normal
    proj_dist = np.dot(np.vstack(cloud).T, normal)

    # Sort points by projected distance
    sorted_indices = np.argsort(proj_dist)
    sorted_proj_dist = proj_dist[sorted_indices]

    # Step 1: Group consecutive points into ledges
    ledges = []
    current_ledge = [sorted_indices[0]]
    for i in range(1, len(sorted_indices)):
        if sorted_proj_dist[i] - sorted_proj_dist[i-1] < ledgeThreshold:
            current_ledge.append(sorted_indices[i])
        else:
            ledges.append(np.array(current_ledge))
            current_ledge = [sorted_indices[i]]
    if current_ledge:
        ledges.append(np.array(current_ledge))

    # Step 2: Compute average projected distance for each ledge
    ledge_avgs = []
    for ledge in ledges:
        ledge_points = np.vstack([cloud[0][ledge], cloud[1][ledge], cloud[2][ledge]]).T
        avg = np.mean(np.dot(ledge_points, normal))
        ledge_avgs.append(avg)

    # Step 3: Sort ledges by their averages
    sorted_ledge_indices = np.argsort(ledge_avgs)
    sorted_ledges = [ledges[i] for i in sorted_ledge_indices]
    sorted_ledge_avgs = [ledge_avgs[i] for i in sorted_ledge_indices]

    # Step 4: Merge close ledges using union-find
    uf = UnionFind(len(sorted_ledges))
    for i in range(len(sorted_ledges) - 1):
        if sorted_ledge_avgs[i + 1] - sorted_ledge_avgs[i] < closeLedges:
            uf.union(i, i + 1)

    # Step 5: Collect merged ledges in order
    merged_ledges = []
    merged_ledge_avgs = []
    current_root = uf.find(0)
    current_group = [sorted_ledges[0]]
    for i in range(1, len(sorted_ledges)):
        if uf.find(i) == current_root:
            current_group.append(sorted_ledges[i])
        else:
            # Merge the current group
            combined_ledge = np.concatenate(current_group)
            ledge_points = np.vstack([cloud[0][combined_ledge], cloud[1][combined_ledge], cloud[2][combined_ledge]]).T
            avg = np.mean(np.dot(ledge_points, normal))
            merged_ledges.append(combined_ledge)
            merged_ledge_avgs.append(avg)
            current_root = uf.find(i)
            current_group = [sorted_ledges[i]]
    # Handle the last group
    if current_group:
        combined_ledge = np.concatenate(current_group)
        ledge_points = np.vstack([cloud[0][combined_ledge], cloud[1][combined_ledge], cloud[2][combined_ledge]]).T
        avg = np.mean(np.dot(ledge_points, normal))
        merged_ledges.append(combined_ledge)
        merged_ledge_avgs.append(avg)

    # Step 6: Filter out small ledges
    total_points = len(cloud[0])
    big_ledges = []
    big_ledge_avgs = []
    for ledge, avg in zip(merged_ledges, merged_ledge_avgs):
        if len(ledge) > total_points * shortLedge:
            big_ledges.append(np.vstack([cloud[0][ledge], cloud[1][ledge], cloud[2][ledge]]))
            big_ledge_avgs.append(avg)

    # Since merged ledges are processed in sorted order, no final sort is needed
    return big_ledges, big_ledge_avgs

def Sort_Ledges(ledges, ledgeAvgs):
    sortedIndices = np.argsort(ledgeAvgs)
    sortedLedges = []
    sortedLedgeAvgs = []
    for i in sortedIndices:
        sortedLedges.append(ledges[i])
        sortedLedgeAvgs.append(ledgeAvgs[i])

    # if len(sortedLedges[1].shape[1]) / len(sortedLedges[0].shape[1]) < 0.05:
    #     pass

    return sortedLedges, sortedLedgeAvgs

def Cloud_Expected_Normal_Filter(cloud, expected_normal, angle_threshold=10):
    """ Discards points whose estimated normal is outside 
        `angle_threshold` degrees of `expected_normal`
    """
    print('Filtering by surface normals')
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


# High-level feature processing
def Broadface_to_Z(cloud, expected_normal, degTolerance=10):
    ''' Finds the lowest surface approx normal to axle's z axis in cloud's local
        frame and rotates it to be aligned with global z axis. Then apply that 
        rotation to the entire cloud. Many debugging steps commented out.
    '''
    print("\nExpected Broadface Normal: ", expected_normal)
    broadface_surfaces = Cloud_Expected_Normal_Filter(cloud, expected_normal, angle_threshold=degTolerance)
    # Plot_Cloud_PyVista(broadface_surfaces, pointSize=2)
    broadface_ledges,_ = Find_Ledges_Along_Normal(broadface_surfaces, normal=expected_normal, ledgeThreshold=1, shortLedge=0.01, closeLedges=10)
    broadface_surfaces = np.hstack(broadface_ledges)
    # Plot_Cloud_PyVista(broadface_surfaces, pointSize=2)
    # for i in range(len(broadface_ledges)):
    #     Plot_Cloud_PyVista(broadface_ledges[i], pointSize=5)

    lowest_broadface_ledge = broadface_ledges[0]
    # bracket_normal, lowest_broadface_plane = RANSAC_Broadface_Plane(broadface_ledges[0], expected_normal)
    lowest_broadface_plane, _, _ = Calc_Plane(broadface_ledges[0])
    bracket_normal = lowest_broadface_plane[:3]
    # bracket_normal[np.abs(bracket_normal) < 1e-11] = 0.0

    print("Broadface Plane: ", lowest_broadface_plane)

    # align lowest broadface surface (assumed the bracket plane) with the global Z
    # then apply resulting transformation (composed of all R_to_Z matrices) to entire cloud1
    cloud_broadface = np.copy(lowest_broadface_ledge)
    # print('Showing lowest broadface surface')
    # Plot_Cloud_PyVista(cloud_broadface)
    cloud_Rs_to_Z = [] # all the rotations used to get cloud1's bracket aligned with z
    i = 0
    while True:
        # print(bracketNormal)
        R_to_Z = Rotation_to_Zaxis(bracket_normal)
        cloud_Rs_to_Z.append(R_to_Z)   
        bracket_normal = np.dot(R_to_Z, bracket_normal)
        # print(f"Aligned Normal {i}:", bracketNormal0)
        cloud_broadface = np.dot(R_to_Z, cloud_broadface)
        skew = abs(bracket_normal[0] + bracket_normal[1])
        # print('skew: ', skew)
        # print(i)
        Check_Scaling(lowest_broadface_ledge, cloud_broadface)
        i += 1
        if skew < 1e-6 or i >= 10:
            # print('Showing leveled lowest broadface surface')
            # Plot_Cloud_PyVista(cloud_broadface)
            break

    total_cloud_R_to_Z = reduce(np.matmul, reversed(cloud_Rs_to_Z))

    return np.dot(total_cloud_R_to_Z, cloud), total_cloud_R_to_Z
