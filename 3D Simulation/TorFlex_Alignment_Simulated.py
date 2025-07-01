import numpy as np
import matplotlib.pyplot as plt
import pyvista as pv
import open3d as o3d
from copy import deepcopy
from functools import reduce
import time

class Axle_Hub_LJS640:  
    def __init__(self, filename, view_angle_horizontal=0.0, scanType='real', cutOff=[-500, 500], ui=None):
        self.ui = ui
        self.minPoints = 10000
        self.anglePoints = 2000
        self.norm_tolerance_deg = 10.0
        self.dist_tolerange_mm = 15.0
        self.max_x_width = 640
        self.min_x_width = 579
        self.reference_z_depth = 1116
        self.min_z_depth = 936
        self.exp_norm = Normal_of_Rotated_Plane(axis='y', angle=view_angle_horizontal)
        
        if scanType == 'real' or scanType == 'live':
            data = np.loadtxt(filename, delimiter=',', dtype=np.float64)
            if scanType == 'real':
                z_scaling_factor = 1
            elif scanType == 'live':
                z_scaling_factor = 1
           
            self.numProfiles, self.numPoints = data.shape
            profile_indices = np.arange(self.numProfiles) - self.numProfiles / 2
            point_indices = (np.arange(self.numPoints) - self.numPoints / 2) * 0.2
            x, y = np.meshgrid(profile_indices, point_indices, indexing='ij')
            x = x.ravel()
            y = y.ravel()
            z = data.ravel() * z_scaling_factor
            valid_mask = (z >= -500) & (z <= 500)
            x = x[valid_mask]
            y = y[valid_mask]
            z = z[valid_mask]

            xScaleSlope = ((self.min_x_width - self.max_x_width) / self.numPoints) / \
                          (self.reference_z_depth - self.min_z_depth)
            scale_factors = np.where(z <= 0, 0.2, 0.2 + xScaleSlope*z)
            x *= scale_factors

            valid_mask = (x > -1000) & (x < 1000) & (z >= cutOff[0]) & (z <= cutOff[1])
            x = x[valid_mask]
            y = y[valid_mask]
            z = z[valid_mask]

            self.cloud = np.array([y, -x, z])
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

    def trim_cloud_z(self, cutOff=[-500, 500]):
        valid_mask = (self.cloud[2] >= cutOff[0]) & (self.cloud[2] <= cutOff[1])
        myCloud = self.cloud[:, valid_mask]
        if myCloud.shape[1] >= self.minPoints:
            self.cloud = myCloud
            self.numPoints = self.cloud.shape[1]
        else:    
            if self.ui:
                self.ui.log_message(f'Trimming Error: Less than {self.minPoints} points in trimmed cloud. Returning untrimmed cloud')
            else:
                print(f'Trimming Error: Less than {self.minPoints} points in trimmed cloud. Returning untrimmed cloud')
        
    def show_cloud(self, altCloud=0):
        if isinstance(altCloud, int):
            Plot_Cloud_PyVista(self.cloud, pointSize=0.5, ui=self.ui)
        else:
            Plot_Cloud_PyVista(altCloud, pointSize=0.5, ui=self.ui)

    def align_z(self, auto=True):
        myCloud, R_matrix = Broadface_to_Z(self.cloud, self.exp_norm, self.norm_tolerance_deg, ui=self.ui)
        self.cloud = myCloud
        if auto == False:
            if self.ui:
                self.ui.log_message('Showing rotated cloud')
            else:
                print('Showing rotated cloud')
            for row in R_matrix:
                if self.ui:
                    self.ui.log_message(' '.join(f'{elem:.6f}' for elem in row))
                else:
                    print(' '.join(f'{elem:.6f}' for elem in row))
            self.show_cloud(myCloud)
            ans = input('Keep cloud after this rotation: y/n')
            if ans == 'n':
                if self.ui:
                    self.ui.log_message('Rejecting cloud. Terminating attempt')
                else:
                    print('Rejecting cloud. Terminating attempt')
                self.cloud = np.nan
        
    def rotate(self, axis, angle):
        angle = np.radians(angle)
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
        self.cloud = np.dot(R, np.array(self.cloud))

    def sort_ledges(self):
        self.cloud = Cloud_Expected_Normal_Filter(self.cloud, self.exp_norm, angle_threshold=self.norm_tolerance_deg, ui=self.ui)
        myLedges, myLedgeAvgs = Find_Ledges_Along_Normal(self.cloud, normal=[0, 0, 1], ledgeThreshold=2.0, shortLedge=0.01, closeLedges=4.5, ui=self.ui)
        self.sorted_ledges, self.sorted_ledge_avgs = Sort_Ledges(myLedges, myLedgeAvgs)

    def calc_ref_angle(self, index=0, plotNum=0):
        self.ref_ledge = self.sorted_ledges[index]
        self.ref_plane, self.ref_angle, _ = Calc_Plane(self.ref_ledge, numPoints=self.anglePoints, plotNum=plotNum, ui=self.ui)

    def calc_hub_angle(self, index=None, auto=True, plotNum=0, deleteGround=False):
        if index != None:
            self.hub_plane, self.hub_angle, _ = Calc_Plane(self.sorted_ledges[index], numPoints=self.anglePoints, plotNum=plotNum, ui=self.ui)
        else:
            self.hub_ledge, self.hub_avg = Find_HubFace(self.sorted_ledges, self.sorted_ledge_avgs, deleteGround=deleteGround, ui=self.ui)
            
            if auto == True:
                self.hub_plane, self.hub_angle, _ = Calc_Plane(self.hub_ledge, numPoints=self.anglePoints, plotNum=plotNum, ui=self.ui)
            
            elif auto == False:
                ans = 'No'
                while ans != 'Yes':
                    self.show_cloud(self.hub_ledge)
                    ans = self.ui.get_input(message='Is this the hub face?')
                    if ans == 'Yes':
                        break
                    if self.hub_avg in self.sorted_ledge_avgs:
                        current_index = self.sorted_ledge_avgs.index(self.hub_avg)
                    else:
                        if self.ui:
                            self.ui.log_message("Error: Hub average not found in sorted ledge averages.")
                        else:
                            print("Error: Hub average not found in sorted ledge averages.")
                        return
                    
                    ans2 = self.ui.get_input(message='Move to next ledge up or down?', options=["Up", "Down"])
                    while ans2 not in ('Up', 'Down'):
                        ans2 = input('Input not allowed. Move to ledge up or down: u/d ')
                    if ans2 == 'Up' and current_index < len(self.sorted_ledges) - 1:
                        current_index += 1
                    elif ans2 == 'Down' and current_index > 0:
                        current_index -= 1
                    else:
                        if self.ui:
                            self.ui.log_message("Cannot move further in that direction.")
                        else:
                            print("Cannot move further in that direction.")
                    self.hub_ledge = self.sorted_ledges[current_index]
                    self.hub_avg = self.sorted_ledge_avgs[current_index]
                self.hub_plane, self.hub_angle, _ = Calc_Plane(self.hub_ledge, numPoints=self.anglePoints, plotNum=plotNum, ui=self.ui)

    def calc_hub_relative_angle(self):
        self.hub_relative_angle = self.hub_angle - self.ref_angle

class Torsion_Arm_LJS640:
    def __init__(self, filename, view_angle_horizontal=0.0, scanType='real', cutOff=[-500, 500], ui=None):
        self.ui = ui
        self.closeLedges = 0.1
        self.ledgeThreshold = 0.1
        self.barFaceRadius = 120
        self.minPoints = 10000
        self.anglePoints = 2000
        self.norm_tolerance_deg = 10.0
        self.dist_tolerange_mm = 15.0
        self.max_x_width = 640
        self.min_x_width = 579
        self.reference_z_depth = 1116
        self.min_z_depth = 936
        self.exp_norm = Normal_of_Rotated_Plane(axis='y', angle=view_angle_horizontal)
        
        if scanType == 'real' or scanType == 'live':
            data = np.loadtxt(filename, delimiter=',', dtype=np.float64)
            if scanType == 'real':
                z_scaling_factor = 1
            elif scanType == 'live':
                z_scaling_factor = 1
        
            self.numProfiles, self.numPoints = data.shape
            profile_indices = np.arange(self.numProfiles) - self.numProfiles / 2
            point_indices = (np.arange(self.numPoints) - self.numPoints / 2) * 0.2
            x, y = np.meshgrid(profile_indices, point_indices, indexing='ij')
            x = x.ravel()
            y = y.ravel()
            z = data.ravel() * z_scaling_factor
            valid_mask = (z >= -500) & (z <= 500)
            x = x[valid_mask]
            y = y[valid_mask]
            z = z[valid_mask]

            xScaleSlope = ((self.min_x_width - self.max_x_width) / self.numPoints) / \
                        (self.reference_z_depth - self.min_z_depth)
            scale_factors = np.where(z <= 0, 0.2, 0.2 + xScaleSlope*z)
            x *= scale_factors

            valid_mask = (x > -1000) & (x < 1000) & (z >= cutOff[0]) & (z <= cutOff[1])
            x = x[valid_mask]
            y = y[valid_mask]
            z = z[valid_mask]

            self.cloud = np.array([y, -x, z])
            self.numPoints = self.cloud.shape[1]
        else:
            data = np.loadtxt(filename, skiprows=1)#[::10]   # Take every 10th point
            self.numProfiles, self.numPoints = data.shape
            x = data[:,0]
            y = data[:,1]
            z = data[:,2]#/100
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

    def rotate_cloud(self, axis, angle):
        self.cloud = Rotate(self.cloud, axis, angle)

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

    def fit_bar_faces(self, plotNum=0, show=False):
        barCloud = self.cloud
        # self.show_cloud(barCloud)

        # Find primary face
        barPrimaryFaces = Cloud_Expected_Normal_Filter(barCloud, self.exp_norm, angle_threshold=6)
        primaryLedges, primaryLedgeAvgs = Find_Ledges_Along_Normal(barPrimaryFaces, normal=self.exp_norm, ledgeThreshold=self.ledgeThreshold, shortLedge=0.01, closeLedges=self.closeLedges)
        self.barPrimaryFace = Sort_Ledges(primaryLedges, primaryLedgeAvgs, sortType='size')[0][-1]
        # self.show_cloud(self.barPrimaryFace)
        self.barPrimaryFace = Clean_Bar_Face(self.barPrimaryFace, radius=self.barFaceRadius)
        # self.show_cloud(self.barPrimaryFace)
        barPrimaryPlane, _, _ = Calc_Plane(self.barPrimaryFace, plotNum=plotNum, numPoints=self.anglePoints)
        barPrimaryNormal = barPrimaryPlane[0:3]

        # Find secondary face, which is perpendicular to primary
        exp_secondary_norm = Rotate(barPrimaryNormal, axis='y', angle=90.0)
        barSecondaryFaces = Cloud_Expected_Normal_Filter(barCloud, exp_secondary_norm, 3)
        secondaryLedges, secondaryLedgeAvgs = Find_Ledges_Along_Normal(barSecondaryFaces, normal=exp_secondary_norm, ledgeThreshold=self.ledgeThreshold, shortLedge=0.1, closeLedges=self.closeLedges)
        
        # for ledge in secondaryLedges:
        #     self.show_cloud(ledge)
        
        self.barSecondaryFace = Sort_Ledges(secondaryLedges, secondaryLedgeAvgs, sortType='size')[0][-1]
        # self.show_cloud(self.barSecondaryFace)
        self.barSecondaryFace = Clean_Bar_Face(self.barSecondaryFace, radius=self.barFaceRadius)
        # self.show_cloud(self.barSecondaryFace)
        barSecondaryPlane, _, _ = Calc_Plane(self.barSecondaryFace, plotNum=plotNum*2, numPoints=self.anglePoints)
        barSecondaryNormal = barSecondaryPlane[0:3]

        # Bar's axis is the intersection of primary and secondary faces
        self.bar_axis = np.cross(barPrimaryNormal, barSecondaryNormal)
        self.bar_faces = np.hstack((self.barPrimaryFace, self.barSecondaryFace))
        highest_y_idx = np.argmax(self.bar_faces[1])
        self.bar_faces_highest_point = self.bar_faces[:, highest_y_idx]
        if show:
            self.show_cloud(np.hstack((self.barPrimaryFace, self.barSecondaryFace)))

    def fit_spindle(self, num_bins=20, min_points_per_bin=10, show=False, plot=False):
        '''Start of finding spindle within cloud'''
        # Set up frame based on bar axis
        approx_axis = self.bar_axis
        if abs(approx_axis[0]) < min(abs(approx_axis[1]), abs(approx_axis[2])):
            u = np.array([1, 0, 0])
        elif abs(approx_axis[1]) < abs(approx_axis[2]):
            u = np.array([0, 1, 0])
        else:
            u = np.array([0, 0, 1])
        u = u - np.dot(u, approx_axis) * approx_axis
        u = u / np.linalg.norm(u)
        v = np.cross(approx_axis, u)
        
        # Select half of scan which includes spindle
            # Calculate distances along axis from starting_point
        starting_point = self.bar_faces_highest_point
        delta = self.cloud.T - starting_point
        s = delta @ approx_axis
        distance_threshold = -100
        mask = (s <= distance_threshold)
        spindle_half = self.cloud.T[mask, :]
        if show:
            self.show_cloud(spindle_half.T)
        
        # Separate spindle from other objects
            # Project all points onto plane orthogonal to bar axis and find spindle
        plane_points = np.dot(spindle_half, np.array([u, v]).T)
        plane_spindle = Bound_Spindle_2D(plane_points, show=show)
        
        # Map planar spindle points back to original 3D cloud
        plane_points = np.dot(spindle_half, np.array([u, v]).T)  # Recalculate plane_points
        # Create a mask for points in plane_spindle
        mask = np.isin(plane_points, plane_spindle).all(axis=1)
        spindle_bounded = spindle_half[mask]
        if show:
            self.show_cloud(spindle_bounded.T)
        '''End of finding spindle within cloud'''
        ''''Start of fitting axis to spindle'''
        # Project spindle points onto bar axis and slice into bins
        t = np.dot(spindle_bounded, approx_axis)
        t_min, t_max = np.min(t), np.max(t)
        bin_edges = np.linspace(t_min, t_max, num_bins + 1)
        centers = []
        for i in range(num_bins):
            mask = (t >= bin_edges[i]) & (t < bin_edges[i + 1])
            if np.sum(mask) < min_points_per_bin:
                continue
            points_bin = spindle_bounded[mask]
            
            # Project points in bin onto the plane orthogonal to bar axis
            points_2d = np.dot(points_bin, np.array([u, v]).T)
            maxC = np.max(points_2d)
            A = np.hstack([points_2d, np.ones((len(points_2d), 1))])
            b = -(points_2d[:, 0]**2 + points_2d[:, 1]**2)
            
            # Fit a circle to projected points and record its center in global 3D frame
            try:
                abc = np.linalg.lstsq(A, b, rcond=None)[0]
                a, b, c = abc
                discriminant = a**2 + b**2 - 4*c
                if discriminant > 0:
                    center_2d = [-a / 2, -b / 2]
                    radius = np.sqrt((a/2)**2 + (b/2)**2 - c)
                    residuals = np.sqrt((points_2d[:, 0] - center_2d[0])**2 + (points_2d[:, 1] - center_2d[1])**2) - radius
                    rmse = np.sqrt(np.mean(residuals**2))
                    if rmse < 0.1:
                        t_center = (bin_edges[i] + bin_edges[i + 1]) / 2
                        center_3d = t_center * approx_axis + center_2d[0] * u + center_2d[1] * v
                        centers.append(center_3d)
                    print(f'Slice {i} with rmse {rmse}')
            except np.linalg.LinAlgError:
                continue
            if plot:# and i < 10:
                # self.show_cloud(points_bin.T)
                theta = np.linspace(0, 2 * np.pi, 100)
                x_circle = center_2d[0] + radius * np.cos(theta)
                y_circle = center_2d[1] + radius * np.sin(theta)
                plt.plot(x_circle, y_circle, 'r-', label='Best-fit circle', linewidth=0.5)
                plt.scatter(points_2d[:, 0], points_2d[:, 1], s=1)
                plt.scatter(center_2d[0], center_2d[1])
                plt.title(f"Projected Slice {i}")
                plt.xlim(-maxC, maxC)
                plt.ylim(-maxC, maxC)
                plt.axis('equal')
                plt.xlabel("u-axis")
                plt.ylabel("v-axis")
                plt.show()
        
        # Fit a line to 3D centers and evaluate fit quality
        if len(centers) < 2:
            raise ValueError("Not enough valid circle fits to determine the axis.")
        centers = np.array(centers)
        c_axis = np.mean(centers, axis=0)
        # PCA for line direction
        U, S, Vt = np.linalg.svd(centers - c_axis, full_matrices=False)
        axis_dir = Vt[0]  # Principal component (largest singular value)
        if np.dot(axis_dir, approx_axis) < 0:
            axis_dir = -axis_dir
        # Calculate fit quality (RMSE of perpendicular distances)
        projections = np.dot(centers - c_axis, axis_dir)
        points_on_line = c_axis + np.outer(projections, axis_dir)
        distances = np.linalg.norm(centers - points_on_line, axis=1)
        rmse = np.sqrt(np.mean(distances**2))
        print(rmse)
        self.axis_loc = c_axis
        self.spindle_axis = axis_dir
        self.spindle_cloud = spindle_bounded.T
        self.line_fit_rmse = rmse  # Store fit quality
        if show:
            pcd = Numpy_to_Open3D(self.spindle_cloud)
            visualize_axis(pcd, c_axis, axis_dir, length=100)

    def calc_angles(self):
        B = np.array(self.bar_axis) / np.linalg.norm(self.bar_axis)
        S = np.array(self.spindle_axis) / np.linalg.norm(self.spindle_axis)
        if not np.isclose(np.linalg.norm(B), 1.0) or not np.isclose(np.linalg.norm(S), 1.0):
            raise ValueError("Inputs must be unit vectors")
        self.bar_angle = np.degrees(np.array([np.arccos(B[0]), np.arccos(B[1]), np.arccos(B[2])]))
        self.spindle_angle = np.degrees(np.array([np.arccos(S[0]), np.arccos(S[1]), np.arccos(S[2])]))
        R = np.cross(B, S)
        sin_theta = np.linalg.norm(R)
        theta_deg = np.degrees(np.arcsin(sin_theta))
        self.total_angle = theta_deg
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


def Numpy_to_Open3D(cloud):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud.T)
    return pcd

def Open3D_to_Numpy(pcd):
    return np.asarray(pcd.points).T

def visualize_axis(pcd, c_axis, axis_dir, length=0.1):
    axis_points = np.array([c_axis - length * axis_dir, c_axis + length * axis_dir])
    axis_line = o3d.geometry.LineSet()
    axis_line.points = o3d.utility.Vector3dVector(axis_points)
    axis_line.lines = o3d.utility.Vector2iVector([[0, 1]])
    axis_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]])
    o3d.visualization.draw_geometries([pcd, axis_line])

def Rotate(threeD_Object, axis, angle):
    print(angle)
    angle = np.radians(angle)
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
    normal = np.array(normal[:3])
    normal = normal / np.linalg.norm(normal)
    z_axis = np.array([0, 0, 1])
    v = np.cross(normal, z_axis)
    cos_theta = np.dot(normal, z_axis)
    sin_theta = np.linalg.norm(v)
    if np.isclose(sin_theta, 0):
        return np.eye(3) if cos_theta > 0 else -np.eye(3)
    v_x, v_y, v_z = v
    K = np.array([[0, -v_z, v_y],
                  [v_z, 0, -v_x],
                  [-v_y, v_x, 0]])
    R = np.eye(3) + K + (1 - cos_theta)*np.dot(K, K)/sin_theta
    i = 0
    while True:
        R = enforce_rotation_properties(R)
        if np.allclose(np.dot(R.T, R), np.eye(3)) and np.isclose(np.linalg.det(R), 1.0):
            return R
        if i > 10:
            return R
        i += 1

def make_orthogonal(matrix):
    U, _, Vt = np.linalg.svd(matrix)
    R = np.dot(U, Vt)
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = np.dot(U, Vt)
    return R

def enforce_rotation_properties(matrix):
    if not np.allclose(np.dot(matrix.T, matrix), np.eye(3), atol=1e-6):
        matrix = make_orthogonal(matrix)
    if not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-6):
        U, _, Vt = np.linalg.svd(matrix)
        matrix = np.dot(U, Vt)
    return matrix

def Check_Scaling(base_cloud, transformed_cloud, ui=None):
    base_cloud = base_cloud - np.mean(base_cloud, axis=1)[:, np.newaxis]
    transformed_cloud = transformed_cloud - np.mean(transformed_cloud, axis=1)[:, np.newaxis]
    original_norm = np.linalg.norm(base_cloud, axis=0).mean()
    transformed_norm = np.linalg.norm(transformed_cloud, axis=0).mean()
    if not np.isclose(original_norm, transformed_norm):
        if ui:
            ui.log_message("Scaling distortion detected")
        else:
            print("Scaling distortion detected")

def Normal_of_Rotated_Plane(axis, angle):
    angle_rad = np.radians(angle)
    if axis == 'x':
        initial_normal = np.array([0, 0, 1])
        R = np.array([[1, 0, 0],
                      [0, np.cos(angle_rad), -np.sin(angle_rad)],
                      [0, np.sin(angle_rad), np.cos(angle_rad)]])
    elif axis == 'y':
        initial_normal = np.array([0, 0, 1])
        R = np.array([[np.cos(angle_rad), 0, np.sin(angle_rad)],
                      [0, 1, 0],
                      [-np.sin(angle_rad), 0, np.cos(angle_rad)]])
    elif axis == 'z':
        initial_normal = np.array([1, 0, 0])
        R = np.array([[np.cos(angle_rad), -np.sin(angle_rad), 0],
                      [np.sin(angle_rad), np.cos(angle_rad), 0],
                      [0, 0, 1]])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'.")
    rotated_normal = np.dot(R, np.array(initial_normal))
    normalized_normal = rotated_normal / np.linalg.norm(rotated_normal)
    normalized_normal[np.abs(normalized_normal) < 1e-12] = 0.0
    return normalized_normal

def Calc_Plane(points, title=0, plotNum=0, numPoints=1000, ui=None):
    sampled_indices = np.linspace(0, points.shape[1] - 1, numPoints, dtype=int)
    sampled_points = points[:, sampled_indices]
    if ui:
        ui.log_message(f'Fitting plane to {sampled_points.shape[1]} points')
    else:
        print(f'Fitting plane to {sampled_points.shape[1]} points')

    def fit_plane(points):
        centroid = np.mean(points, axis=1, keepdims=True)
        svd = np.linalg.svd(points - centroid)
        normal_vector = svd[0][:, -1]
        d = -normal_vector.dot(centroid.flatten())
        plane = np.hstack((normal_vector, d))
        return plane, centroid.flatten()

    def filter_plane(points, normal_vector, iqr_scale):
        d = -normal_vector.dot(centroid.flatten())
        ortho_dist = np.abs(normal_vector[0] * points[0] +
                            normal_vector[1] * points[1] +
                            normal_vector[2] * points[2] + d) / np.linalg.norm(normal_vector)
        Q1, Q3 = np.percentile(ortho_dist, [25, 75])
        IQR = Q3 - Q1
        lower_bound = Q1 - iqr_scale * IQR
        upper_bound = Q3 + iqr_scale * IQR
        filtered_points = points[:, (ortho_dist >= lower_bound) & (ortho_dist <= upper_bound)]
        return filtered_points

    filt_points = sampled_points
    for i, iqr_scale in enumerate([1.0, 0.5]):
        if ui:
            ui.log_message(f"\tIteration {i}: filtering {filt_points.shape[1]} points")
        else:
            print(f"\tIteration {i}: filtering {filt_points.shape[1]} points")
        plane, centroid = fit_plane(filt_points)
        filt_points = filter_plane(filt_points, plane[0:3], iqr_scale)
    final_filt_points = filt_points
    if ui:
        ui.log_message(f"\tIteration {i+1}: final {final_filt_points.shape[1]} points")
    else:
        print(f"\tIteration {i+1}: final {final_filt_points.shape[1]} points")
    plane_final, centroid_final = fit_plane(final_filt_points)
    normal_final = plane_final[0:3]
    d_final = -normal_final.dot(centroid_final.flatten())
    
    x_angle = np.degrees(np.arccos(abs(normal_final[0]) / np.linalg.norm(normal_final)))
    y_angle = np.degrees(np.arccos(abs(normal_final[1]) / np.linalg.norm(normal_final)))
    z_angle = np.degrees(np.arccos(abs(normal_final[2]) / np.linalg.norm(normal_final)))

    ortho_dist_final = np.abs(normal_final[0] * final_filt_points[0] +
                              normal_final[1] * final_filt_points[1] +
                              normal_final[2] * final_filt_points[2] + d_final) / np.linalg.norm(normal_final)
    uncertainty_95 = np.std(ortho_dist_final) * 1.96
    
    if plotNum != 0:
        fig = plt.figure(plotNum)
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(sampled_points[0], sampled_points[1], sampled_points[2], color='red', s=1)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title('Best Fit Plane w/Filtering')
        ax.scatter(final_filt_points[0], final_filt_points[1], final_filt_points[2], color='green', s=10)
        xx, yy = np.meshgrid(np.linspace(np.min(final_filt_points[0]), np.max(final_filt_points[0]), 10),
                            np.linspace(np.min(final_filt_points[1]), np.max(final_filt_points[1]), 10))
        zz = (-normal_final[0] * xx - normal_final[1] * yy - d_final) / normal_final[2]
        ax.plot_surface(xx, yy, zz, alpha=0.5, color='yellow')
        if title:
            ax.set_title(title)
            fig.suptitle(f'x: {90 - x_angle:.3f}, y: {90 - y_angle:.3f}, z: {z_angle:.3f}')
        plt.figure(10+plotNum)
        plt.hist(ortho_dist_final, bins=17, density=True)
        plt.axvline(uncertainty_95, c='red')
        plt.title(f'Distribution of Plane Error (95% Spread: ±{uncertainty_95:.2f} mm)')
        plt.xlabel('Orthogonal Distance - Point to Plane (mm)')
        plt.show()

    return plane_final, np.array([x_angle, y_angle, z_angle]), uncertainty_95

def Plot_Cloud_PyVista(points, pointSize=1.0, ui=None):
    cloud = pv.PolyData(points.T)
    if ui:
        ui.log_message(f'Displaying {points.shape[1]} points in PyVista window\n\tTo continue, select PyVista window and press q\n\tDo not close PyVista window')
    else:
        print(f'Displaying {points.shape[1]} points')
    time.sleep(0.1)
    z_values = points[2]
    cloud.point_data['z'] = z_values
    cloud.point_data.set_array(z_values, 'z')
    plotter = pv.Plotter()
    plotter.set_background('gray')
    plotter.add_mesh(cloud, scalars='z', cmap='coolwarm', point_size=pointSize)
    
    def orbit_left():
        plotter.camera.azimuth -= 1
        plotter.render()

    def orbit_right():
        plotter.camera.azimuth += 1
        plotter.render()

    def orbit_up():
        plotter.camera.elevation += 1
        plotter.render()

    def orbit_down():
        plotter.camera.elevation -= 1
        plotter.render()

    def save_image():
        timestamp = time.strftime("%Y%m%d_%H%M%S") + f"{int(time.time() * 1000) % 1000:03d}"
        filename = f"point_cloud_{timestamp}.png"
        plotter.screenshot(filename)
        if ui:
            ui.log_message(f"Image saved as {filename}")
        else:
            print(f"Image saved as {filename}")

    plotter.add_key_event('4', orbit_left)
    plotter.add_key_event('6', orbit_right)
    plotter.add_key_event('8', orbit_up)
    plotter.add_key_event('2', orbit_down)
    plotter.add_key_event('s', save_image)
    plotter.camera_position = 'xy'
    plotter.camera_set = True
    plotter.reset_camera()
    plotter.show(auto_close=False, interactive=True)

class UnionFind:
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

def Find_Ledges_Along_Normal(cloud, normal, ledgeThreshold, shortLedge, closeLedges, ui=None):
    if ui:
        ui.log_message('Grouping filtered surfaces into ledges')
    else:
        print('Grouping filtered surfaces into ledges')
    normal = normal / np.linalg.norm(normal)
    proj_dist = np.dot(np.vstack(cloud).T, normal)
    sorted_indices = np.argsort(proj_dist)
    sorted_proj_dist = proj_dist[sorted_indices]
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
    ledge_avgs = []
    for ledge in ledges:
        ledge_points = np.vstack([cloud[0][ledge], cloud[1][ledge], cloud[2][ledge]]).T
        avg = np.mean(np.dot(ledge_points, normal))
        ledge_avgs.append(avg)
    sorted_ledge_indices = np.argsort(ledge_avgs)
    sorted_ledges = [ledges[i] for i in sorted_ledge_indices]
    sorted_ledge_avgs = [ledge_avgs[i] for i in sorted_ledge_indices]
    uf = UnionFind(len(sorted_ledges))
    for i in range(len(sorted_ledges) - 1):
        if sorted_ledge_avgs[i + 1] - sorted_ledge_avgs[i] < closeLedges:
            uf.union(i, i + 1)
    merged_ledges = []
    merged_ledge_avgs = []
    current_root = uf.find(0)
    current_group = [sorted_ledges[0]]
    for i in range(1, len(sorted_ledges)):
        if uf.find(i) == current_root:
            current_group.append(sorted_ledges[i])
        else:
            combined_ledge = np.concatenate(current_group)
            ledge_points = np.vstack([cloud[0][combined_ledge], cloud[1][combined_ledge], cloud[2][combined_ledge]]).T
            avg = np.mean(np.dot(ledge_points, normal))
            merged_ledges.append(combined_ledge)
            merged_ledge_avgs.append(avg)
            current_root = uf.find(i)
            current_group = [sorted_ledges[i]]
    if current_group:
        combined_ledge = np.concatenate(current_group)
        ledge_points = np.vstack([cloud[0][combined_ledge], cloud[1][combined_ledge], cloud[2][combined_ledge]]).T
        avg = np.mean(np.dot(ledge_points, normal))
        merged_ledges.append(combined_ledge)
        merged_ledge_avgs.append(avg)
    total_points = len(cloud[0])
    big_ledges = []
    big_ledge_avgs = []
    for ledge, avg in zip(merged_ledges, merged_ledge_avgs):
        if len(ledge) > total_points * shortLedge:
            big_ledges.append(np.vstack([cloud[0][ledge], cloud[1][ledge], cloud[2][ledge]]))
            big_ledge_avgs.append(avg)
    return big_ledges, big_ledge_avgs

def Sort_Ledges(ledges, ledgeAvgs, sortType='location'):
    if sortType == 'location':
        sortedIndices = np.argsort(ledgeAvgs)
    elif sortType == 'size':
        sortedIndices = np.argsort([ledge.shape[1] for ledge in ledges])

    sortedLedges = [ledges[i] for i in sortedIndices]
    sortedLedgeAvgs = [ledgeAvgs[i] for i in sortedIndices]
    return sortedLedges, sortedLedgeAvgs

def Clean_Bar_Face(face, radius):
    centroid_xy = [np.mean(face[0]), np.mean(face[1])]
    distances = np.sqrt((face[0] - centroid_xy[0])**2 + (face[1] - centroid_xy[1])**2)
    mask = distances <= radius
    return face[:, mask]

def Bound_Spindle_2D(plane_points, show=False):
    # Define quadrant clustering function
    def cluster_quadrants(points, min_points=100, max_levels=5, tolerance=1e1):
        def divide_quadrants(points, u_mean, v_mean, level=0, prefix=''):
            if len(points) < min_points or level >= max_levels:
                return {}
            
            quadrants = {
                f'{prefix}Q1': {'points': [], 'center': None},  # u > u_mean, v > v_mean
                f'{prefix}Q2': {'points': [], 'center': None},  # u < u_mean, v > v_mean
                f'{prefix}Q3': {'points': [], 'center': None},  # u < u_mean, v < v_mean
                f'{prefix}Q4': {'points': [], 'center': None}   # u > u_mean, v < v_mean
            }
            
            # Assign points to quadrants
            for point in points:
                u, v = point
                if u > u_mean and v > v_mean:
                    quadrants[f'{prefix}Q1']['points'].append(point)
                elif u < u_mean and v > v_mean:
                    quadrants[f'{prefix}Q2']['points'].append(point)
                elif u < u_mean and v < v_mean:
                    quadrants[f'{prefix}Q3']['points'].append(point)
                elif u > u_mean and v < v_mean:
                    quadrants[f'{prefix}Q4']['points'].append(point)
            
            results = {}
            for q in quadrants:
                points_q = np.array(quadrants[q]['points'])
                if len(points_q) >= min_points:
                    center = np.mean(points_q, axis=0)
                    quadrants[q]['center'] = center
                    # Recursively divide quadrants
                    sub_results = divide_quadrants(points_q, center[0], center[1], level + 1, f'{q}_')
                    # Check if sub-quadrant center aligns with current center
                    for sub_q, sub_data in sub_results.items():
                        if sub_data['center'] is not None and np.allclose(sub_data['center'], center, atol=tolerance):
                            continue
                        results[sub_q] = sub_data
                    results[q] = quadrants[q]
            
            return results

        u_mean, v_mean = np.mean(points[:, 0]), np.mean(points[:, 1])
        return divide_quadrants(points, u_mean, v_mean)

    # Apply quadrant clustering
    min_points = 100
    clusters = cluster_quadrants(plane_points, min_points=min_points)

    # Optional: Plot clusters for visualization
    for q, data in clusters.items():
        points = np.array(data['points'])
        if len(points) >= min_points:
            plt.scatter(points[:, 0], points[:, 1], s=1, label=q)
            if data['center'] is not None:
                plt.scatter(data['center'][0], data['center'][1], c='black', marker='x')
    # Extract cluster centers
    cluster_centers = {q: data['center'] for q, data in clusters.items() if data['center'] is not None}
    # Combine cluster centers within 50 units
    combined_clusters = []
    processed = set()
    threshold = 50

    for q1, c1 in cluster_centers.items():
        if q1 in processed:
            continue
        combined_points = clusters[q1]['points']
        for q2, c2 in cluster_centers.items():
            if q2 != q1 and q2 not in processed:
                if np.linalg.norm(c1 - c2) < threshold:
                    combined_points.extend(clusters[q2]['points'])
                    processed.add(q2)
        combined_clusters.append(np.array(combined_points))
        processed.add(q1)

    if show:
        for cluster in combined_clusters:
            if len(cluster) > 0:
                center = np.mean(cluster, axis=0)
                plt.scatter(center[0], center[1], c='red', s=50)
        plt.xlabel("u-axis")
        plt.ylabel("v-axis")
        plt.axis('equal')
        plt.legend()
        plt.show()

    plane_spindle = max(combined_clusters, key=len)
    return plane_spindle

def Cloud_Expected_Normal_Filter(cloud, expected_normal, angle_threshold=10, ui=None):
    if ui:
        ui.log_message('Filtering by surface normals')
    else:
        print('Filtering by surface normals')
    pcd = Numpy_to_Open3D(cloud)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))
    normals = np.asarray(pcd.normals)
    cos_theta = np.dot(normals, expected_normal) / (np.linalg.norm(normals, axis=1) * np.linalg.norm(expected_normal))
    angles = np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))
    valid_indices = np.where(angles < angle_threshold)[0]
    return Open3D_to_Numpy(pcd.select_by_index(valid_indices))

def RANSAC_Broadface_Plane(cloud, expected_normal, max_angle_deviation=10, distance_threshold=1.0, ui=None):
    pcd = Numpy_to_Open3D(cloud)
    max_angle_rad = np.radians(max_angle_deviation)
    for _ in range(1000):
        plane_model, _ = pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=3, num_iterations=10)
        normal = np.array(plane_model[:3])
        dot_product = np.dot(normal, expected_normal) / (np.linalg.norm(normal) * np.linalg.norm(expected_normal))
        angle = np.arccos(np.clip(dot_product, -1.0, 1.0))
        if angle < max_angle_rad:
            normal[np.abs(normal) < 1e-11] = 0.0
            plane_model[np.abs(plane_model) < 1e-11] = 0.0
            return normal, np.array(plane_model)
    if ui:
        ui.log_message("Warning: No valid plane found within angle constraint.")
    else:
        print("Warning: No valid plane found within angle constraint.")
    return None, None

def RANSAC_ICP_Best_Fit(adjCloud, baseCloud, threshold=10.0, max_iteration=50):
    source_cloud = o3d.geometry.PointCloud()
    source_cloud.points = o3d.utility.Vector3dVector(adjCloud.T)
    source_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target_cloud = o3d.geometry.PointCloud()
    target_cloud.points = o3d.utility.Vector3dVector(baseCloud.T)
    target_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    if not isinstance(source_cloud, o3d.geometry.PointCloud) or not isinstance(target_cloud, o3d.geometry.PointCloud):
        raise ValueError("Clouds must be Open3D PointCloud objects")
    initial_transform = np.eye(4)
    reg_result = o3d.pipelines.registration.registration_icp(
        source_cloud, target_cloud, threshold, initial_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration)
    )
    return source_cloud.transform(reg_result.transformation), reg_result.transformation, reg_result.fitness, reg_result.inlier_rmse

def Broadface_to_Z(cloud, expected_normal, degTolerance=10, ui=None):
    if ui:
        ui.log_message("\nExpected Broadface Normal: " + str(expected_normal))
    else:
        print("\nExpected Broadface Normal: ", expected_normal)
    broadface_surfaces = Cloud_Expected_Normal_Filter(cloud, expected_normal, angle_threshold=degTolerance, ui=ui)
    broadface_ledges, _ = Find_Ledges_Along_Normal(broadface_surfaces, normal=expected_normal, ledgeThreshold=1, shortLedge=0.01, closeLedges=10, ui=ui)
    broadface_surfaces = np.hstack(broadface_ledges)
    lowest_broadface_ledge = broadface_ledges[0]
    lowest_broadface_plane, _, _ = Calc_Plane(broadface_ledges[0], ui=ui)
    bracket_normal = lowest_broadface_plane[:3]
    if ui:
        ui.log_message("Broadface Plane: " + str(lowest_broadface_plane))
    else:
        print("Broadface Plane: ", lowest_broadface_plane)
    cloud_broadface = np.copy(lowest_broadface_ledge)
    cloud_Rs_to_Z = []
    i = 0
    while True:
        R_to_Z = Rotation_to_Zaxis(bracket_normal)
        cloud_Rs_to_Z.append(R_to_Z)   
        bracket_normal = np.dot(R_to_Z, bracket_normal)
        cloud_broadface = np.dot(R_to_Z, cloud_broadface)
        skew = abs(bracket_normal[0] + bracket_normal[1])
        Check_Scaling(lowest_broadface_ledge, cloud_broadface, ui=ui)
        i += 1
        if skew < 1e-6 or i >= 10:
            break
    total_cloud_R_to_Z = reduce(np.matmul, reversed(cloud_Rs_to_Z))
    return np.dot(total_cloud_R_to_Z, cloud), total_cloud_R_to_Z

def Check_Not_Casting(ledge, ui=None):
    if ui:
        ui.log_message('Testing if bottom ledge is casting or hubface')
    else:
        print('Testing if bottom ledge is casting or hubface')
    x, y, z = ledge[0], ledge[1], ledge[2]
    center_x, center_y = np.mean(x), np.mean(y)
    radii = np.sqrt((x - center_x)**2 + (y - center_y)**2)
    Q1 = np.percentile(radii, 25)
    Q3 = np.percentile(radii, 75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.0 * IQR
    upper_bound = Q3 + 1.0 * IQR
    valid_indices = (radii >= lower_bound) & (radii <= upper_bound)
    x, y, z, radii = x[valid_indices], y[valid_indices], z[valid_indices], radii[valid_indices]
    max_radius = np.max(radii)
    min_radius = np.min(radii)
    valid_indices_outer = radii >= 0.8 * max_radius
    valid_indices_inner = radii <= 1.3 * min_radius
    x_outer, y_outer, z_outer = x[valid_indices_outer], y[valid_indices_outer], z[valid_indices_outer]
    x_inner, y_inner, z_inner = x[valid_indices_inner], y[valid_indices_inner], z[valid_indices_inner]
    angles_outer = np.arctan2(y_outer - center_y, x_outer - center_x)
    angles_inner = np.arctan2(y_inner - center_y, x_inner - center_x)
    angles_outer = np.mod(angles_outer, 2*np.pi)
    angles_inner = np.mod(angles_inner, 2*np.pi)
    sorted_indices_outer = np.argsort(angles_outer)
    sorted_indices_inner = np.argsort(angles_inner)
    sorted_angles_outer = angles_outer[sorted_indices_outer]
    sorted_angles_inner = angles_inner[sorted_indices_inner]
    angular_gaps_outer = np.diff(sorted_angles_outer, append=sorted_angles_outer[0] + 2*np.pi)
    angular_gaps_inner = np.diff(sorted_angles_inner, append=sorted_angles_inner[0] + 2*np.pi)
    gap_threshold = 2*np.pi/15
    large_gaps_outer = np.sum(angular_gaps_outer > gap_threshold)
    large_gaps_inner = np.sum(angular_gaps_inner > gap_threshold)
    if ui:
        ui.log_message(f'Large angular gaps (wheel mounts) on outer,inner perimeter: {large_gaps_outer},{large_gaps_inner}')
    else:
        print(f'Large angular gaps (wheel mounts) on outer,inner perimeter: {large_gaps_outer},{large_gaps_inner}')
    if large_gaps_inner < 4 or large_gaps_outer >= 5:
        return True
    else:
        return False

def Find_HubFace(ledges, ledgeAvgs, reverse=False, deleteGround=False, ui=None):
    ledges = deepcopy(ledges)
    ledgeAvgs = ledgeAvgs[:]
    ledgeNumPoints = [len(ledge[0]) for ledge in ledges]
    descendIndices_numPoints = np.argsort(ledgeNumPoints)[::-1]
    biggestLedge_numPoints = len(ledges[descendIndices_numPoints[0]][0])
    sortedIndices = np.argsort(ledgeAvgs)[::-1] if reverse else np.argsort(ledgeAvgs)
    while len(ledges[sortedIndices[0]][0]) / biggestLedge_numPoints <= 0.50:
        del ledges[sortedIndices[0]]
        del ledgeAvgs[sortedIndices[0]]
        del ledgeNumPoints[sortedIndices[0]]
        sortedIndices = np.argsort(ledgeAvgs)[::-1] if reverse else np.argsort(ledgeAvgs)
        ledgeNumPoints = [len(ledge[0]) for ledge in ledges]
        descendIndices_numPoints = np.argsort(ledgeNumPoints)[::-1]
        biggestLedge_numPoints = len(ledges[descendIndices_numPoints[0]][0])
    if deleteGround:
        if ui:
            ui.log_message('Ignoring Ground in Find_HubFace()')
        else:
            print('Ignoring Ground in Find_HubFace()')
        del ledges[sortedIndices[0]]
        del ledgeAvgs[sortedIndices[0]]
        del ledgeNumPoints[sortedIndices[0]]
        sortedIndices = np.argsort(ledgeAvgs)[::-1] if reverse else np.argsort(ledgeAvgs)
        ledgeNumPoints = [len(ledge[0]) for ledge in ledges]
        descendIndices_numPoints = np.argsort(ledgeNumPoints)[::-1]
        biggestLedge_numPoints = len(ledges[descendIndices_numPoints[0]][0])
    while len(ledges[sortedIndices[0]][0]) / biggestLedge_numPoints <= 0.50:
        del ledges[sortedIndices[0]]
        del ledgeAvgs[sortedIndices[0]]
        del ledgeNumPoints[sortedIndices[0]]
        sortedIndices = np.argsort(ledgeAvgs)[::-1] if reverse else np.argsort(ledgeAvgs)
        ledgeNumPoints = [len(ledge[0]) for ledge in ledges]
        descendIndices_numPoints = np.argsort(ledgeNumPoints)[::-1]
        biggestLedge_numPoints = len(ledges[descendIndices_numPoints[0]][0])
    numPoints0 = len(ledges[sortedIndices[0]][0])
    numPoints1 = len(ledges[sortedIndices[1]][0])
    if numPoints1 / numPoints0 >= 0.15:
        if ui:
            ui.log_message('Checking if hubface spread is OK')
        else:
            print('Checking if hubface spread is OK')
        _, _, spread = Calc_Plane(ledges[sortedIndices[1]], numPoints=5000, ui=ui)
        if ui:
            ui.log_message(str(spread))
        else:
            print(spread)
        if spread <= 0.15:
            hubFace = ledges[sortedIndices[1]]              
            hubFaceAvg = ledgeAvgs[sortedIndices[1]]
        else:
            hubFace = ledges[sortedIndices[2]]              
            hubFaceAvg = ledgeAvgs[sortedIndices[2]]
    else:
        if Check_Not_Casting(ledges[sortedIndices[0]], ui=ui):
            hubFace = ledges[sortedIndices[0]]
            hubFaceAvg = ledgeAvgs[sortedIndices[0]]
        else:
            if ui:
                ui.log_message('Checking if hubface spread is OK')
            else:
                print('Checking if hubface spread is OK')
            _, _, spread = Calc_Plane(ledges[sortedIndices[1]], numPoints=5000, ui=ui)
            if ui:
                ui.log_message(str(spread))
            else:
                print(spread)
            if spread <= 0.15:
                hubFace = ledges[sortedIndices[1]]          
                hubFaceAvg = ledgeAvgs[sortedIndices[1]]
            else:
                hubFace = ledges[sortedIndices[2]]              
                hubFaceAvg = ledgeAvgs[sortedIndices[2]]
    return hubFace, hubFaceAvg