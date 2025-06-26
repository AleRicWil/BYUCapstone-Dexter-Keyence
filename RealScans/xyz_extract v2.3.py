import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import time
from scipy.sparse.linalg import svds
import scipy.stats
import pyvista as pv
from collections import defaultdict
import open3d as o3d
from copy import deepcopy

# import numba, cython

plotWidth = 200

def Filt_Raw_Points(Points, threshold, shortSeg):
    print('Filtering noisy points')
    z = Points[2]

    # Calculate the difference between consecutive elements
    diff = np.diff(z)

    # Identify shift indices where the difference exceeds the threshold
    shift_indices = np.flatnonzero(np.abs(diff) > threshold) + 1  # Add 1 for alignment
    shift_indices = np.r_[0, shift_indices, len(z)]  # Include start and end indices efficiently

    # Filter noisy points and merge valid segments
    filtered_segments = []

    for i in range(len(shift_indices) - 1):
        start, end = shift_indices[i], shift_indices[i + 1]
        segment = Points[:, start:end]

        # Skip processing if the segment is too small
        if segment.shape[1] < shortSeg:
            continue

        # Compute IQR bounds for z-values in the segment
        Q1, Q3 = np.percentile(segment[2], [25, 75])
        IQR = Q3 - Q1
        lower_bound, upper_bound = Q1 - 1.5 * IQR, Q3 + 1.5 * IQR

        # Apply filtering directly with NumPy boolean masking
        mask = (segment[2] >= lower_bound) & (segment[2] <= upper_bound)

        # Append filtered segment if it meets size criteria
        if np.count_nonzero(mask) >= shortSeg:
            filtered_segments.append(segment[:, mask])

    # Concatenate all filtered segments into a single array
    if filtered_segments:
        filtPoints = np.concatenate(filtered_segments, axis=1)
    else:
        filtPoints = np.empty((3, 0))  # Empty array if no valid segments

    print(f'Filtered Points: {filtPoints.shape[1]}')
    return filtPoints

def detect_hubFace(ledges, threshold):
    diff = np.diff(ledges)   
    shift_indices = np.where(np.abs(diff) > threshold)[0] + 1  # +1 to get the index of the larger value
    # print('ledge Groups', shift_indices)
    # If no shifts are detected, handle accordingly
    if len(shift_indices) == 0:
        print("No significant shifts detected.")
    else:
        # The start and end of the second group can be inferred from shifts
        # Assuming the first group is up to the first shift
        # The second group starts after the first shift and ends before the next shift
        start_index = shift_indices[0]  # Start of the second group
        end_index = shift_indices[1] if len(shift_indices) > 1 else len(ledges)  # End of the second group

        # Extract the second group values
        hubFace = ledges[start_index:end_index]

        return hubFace

def calculate_segment_averages(z, shift_indices):
    # Add 0 at the beginning and len(z) at the end to cover all segments
    segment_boundaries = np.concatenate(([0], shift_indices, [len(z)]))
    
    # Preallocate an array for the averages
    averages = np.zeros(len(segment_boundaries) - 1)
    
    for i in range(len(segment_boundaries) - 1):
        # Slice the vector between each pair of boundaries
        segment = z[segment_boundaries[i]:segment_boundaries[i + 1]]
        # Calculate the average of the segment and assign it to the preallocated array
        averages[i] = np.mean(segment)
    
    return averages

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
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')
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

    return plane_final, np.array([x_angle, y_angle, z_angle]), uncertainty_95

def Plot_All(Points, plotNum, title=0):
    fig = plt.figure(plotNum)
    ax = fig.add_subplot(111, projection='3d')

    # Plot the data
    ax.scatter(Points[0], Points[1], Points[2], s=1)
    # Add labels
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_box_aspect([1, 1, 1])
    if title != 0:
        ax.set_title(title)
    # plt.show()

def Filt_nan(Points):
    # print('len(Points): ', len(Points[2]))
    newPoints = Points[:, ~np.isnan(Points[2])]
    # print('len(newPoints): ', len(newPoints[2]))

    return newPoints

def Stitch_Close_Ledges(bigLedges, bigLedgeAvgs, closeBigLedges):
    """
    Optimized function to stitch ledges that are close in z-averages.
    """
    # print('closeLedgePairs', closeBigLedges)
    
    # Union-Find data structure for grouping ledges
    parent = list(range(len(bigLedges)))

    def find(x):
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]

    def union(x, y):
        rootX = find(x)
        rootY = find(y)
        if rootX != rootY:
            parent[rootY] = rootX

    # Combine connected ledges
    for i, j in closeBigLedges:
        union(i, j)

    # Group ledges based on root parent
    ledge_groups = defaultdict(list)
    for i in range(len(bigLedges)):
        root = find(i)
        ledge_groups[root].append(i)

    stitchedBigLedges = []
    stitchedBigLedgeAvgs = []

    for group in ledge_groups.values():
        combined_ledge = np.hstack([bigLedges[i] for i in group])
        combined_avg = np.mean(combined_ledge[2])
        stitchedBigLedges.append(combined_ledge)
        stitchedBigLedgeAvgs.append(combined_avg)

    # print('\nbigLedgeAvgs: ', bigLedgeAvgs)
    # print('\nstitchedBigLedgeAvgs: ', stitchedBigLedgeAvgs)

    return stitchedBigLedges, stitchedBigLedgeAvgs

def Find_Ledges(allPoints, ledgeThreshold, shortLedge, closeLedges, numPoints = 50000):
    """
    Optimized function to find and group ledges based on z-values.
    """
    print('\nSorting points into ledges')
    sampled_indices = np.linspace(0, allPoints.shape[1] - 1, numPoints, dtype=int)
    Points = allPoints[:, sampled_indices]
    
    ledges = []
    ledgeAvgs = []
    numLedges = 0

    for i in range(len(Points[0])):
        zVal = Points[2][i]
        currentPoint = Points[:, i:i+1]  # Extract a column as a point

        foundSimilarLedge = False
        for k in range(numLedges):
            avg = ledgeAvgs[k]
            if abs(zVal - avg) <= ledgeThreshold:
                ledges[k] = np.hstack((ledges[k], currentPoint))
                ledgeAvgs[k] = avg + (zVal - avg) / ledges[k].shape[1]
                foundSimilarLedge = True
                break

        if not foundSimilarLedge:
            ledges.append(currentPoint)
            ledgeAvgs.append(zVal)
            numLedges += 1

    # Filter short ledges
    print(f'Discarding ledges with less than {shortLedge} points')
    bigLedges = [ledge for ledge in ledges if ledge.shape[1] > shortLedge]
    bigLedgeAvgs = [ledgeAvgs[i] for i, ledge in enumerate(ledges) if ledge.shape[1] > shortLedge]

    # Find close ledges
    closeBigLedges = [
        (i, j)
        for i in range(len(bigLedges))
        for j in range(i + 1, len(bigLedges))
        if abs(bigLedgeAvgs[i] - bigLedgeAvgs[j]) <= closeLedges
    ]

    if closeBigLedges:
        print('Combining inclined ledge segments')
        bigLedges, bigLedgeAvgs = Stitch_Close_Ledges(bigLedges, bigLedgeAvgs, closeBigLedges)

    # Map back to the original points
    originalLedges = []
    for avg in bigLedgeAvgs:
        mask = np.abs(allPoints[2] - avg) <= closeLedges
        originalLedges.append(allPoints[:, mask])

    # Update total points based on original data
    total_original_points = sum(ledge.shape[1] for ledge in originalLedges)
    print(f'Found {len(originalLedges)} ledges with {total_original_points} total points')

    return originalLedges, bigLedgeAvgs

def Plot_Ledges(ledges, ledgeAvgs, pointsPerLedge=1000):
    numLedges = len(ledges)
    print(f'Displaying {numLedges} ledges with {pointsPerLedge} points per ledge')
   
    fig = plt.figure(42)
    ax = fig.add_subplot(111, projection='3d')
    for i in range(numLedges):
        # Sample points if necessary
        if ledges[i].shape[1] > pointsPerLedge:
            sampled_indices = np.linspace(0, ledges[i].shape[1] - 1, pointsPerLedge, dtype=int)
            ledge = ledges[i][:, sampled_indices]
        else:
            ledge = ledges[i]

        ax.scatter(ledge[0], ledge[1], ledge[2], s=1, label=f'{ledgeAvgs[i]:.2f}')
        ax.set_title(f'numLedges: {numLedges}')
    
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

def Check_Not_Casting(ledge):
    """
    Determines whether the outer perimeter of the ledge forms a continuous circle or a star-like pattern.

    Parameters:
        ledge (list of numpy arrays): A list containing the x, y, and z coordinates of the ledge points.

    Returns:
        bool: True if the perimeter forms a star-like pattern (with large, consistently spaced gaps),
              False if the perimeter forms a mostly continuous circle.
    """
    print('Testing if bottom ledge is casting or hubface')
    # Extract x, y coordinates
    x, y, z = ledge[0], ledge[1], ledge[2]

    # Calculate the center of the bounding circle as the mean of the points
    center_x, center_y = np.mean(x), np.mean(y)

    # Calculate the radii of the points from the center
    radii = np.sqrt((x - center_x)**2 + (y - center_y)**2)

    # Filter radii outliers
    Q1 = np.percentile(radii, 25)
    Q3 = np.percentile(radii, 75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.0 * IQR
    upper_bound = Q3 + 1.0 * IQR
    # Filter out values outside the bounds
    valid_indices = (radii >= lower_bound) & (radii <= upper_bound)
    x, y, z, radii = x[valid_indices], y[valid_indices], z[valid_indices], radii[valid_indices]

    # Plot_All([x,y,z], plotNum=131, title='Filtered Check-Casting Ledge')

    # Approximate the smallest bounding circle's radius as the maximum radius   
    max_radius = np.max(radii)
    min_radius = np.min(radii)

    # Filter points whose radius is less than 90% of the max radius
    valid_indices_outer = radii >= 0.8 * max_radius
    valid_indices_inner = radii <= 1.3 * min_radius
    x_outer, y_outer, z_outer = x[valid_indices_outer], y[valid_indices_outer], z[valid_indices_outer]
    x_inner, y_inner, z_inner = x[valid_indices_inner], y[valid_indices_inner], z[valid_indices_inner]

    # Plot_All([x_outer,y_outer,z_outer], plotNum=132, title='outer ring')
    # Plot_All([x_inner,y_inner,z_inner], plotNum=133, title='inner ring')

    # Calculate angles (in radians) of the points relative to the center
    angles_outer = np.arctan2(y_outer - center_y, x_outer - center_x)
    angles_inner = np.arctan2(y_inner - center_y, x_inner - center_x)
    angles_outer = np.mod(angles_outer, 2*np.pi)  # Normalize to [0, 2π]
    angles_inner = np.mod(angles_inner, 2*np.pi)

    # Sort the points by angle
    sorted_indices_outer = np.argsort(angles_outer)
    sorted_indices_inner = np.argsort(angles_inner)
    sorted_angles_outer = angles_outer[sorted_indices_outer]
    sorted_angles_inner = angles_inner[sorted_indices_inner]

    # Calculate the angular gaps between consecutive points
    angular_gaps_outer = np.diff(sorted_angles_outer, append=sorted_angles_outer[0] + 2*np.pi)
    angular_gaps_inner = np.diff(sorted_angles_inner, append=sorted_angles_inner[0] + 2*np.pi)

    # Define a threshold for large gaps: Significant deviations from the mean
    gap_threshold = 2*np.pi/15 # angular distance between studs for 12 stud hub

    # Count large gaps
    large_gaps_outer = np.sum(angular_gaps_outer > gap_threshold)
    large_gaps_inner = np.sum(angular_gaps_inner > gap_threshold)
    print(f'Large angular gaps (wheel mounts) on outer,inner perimeter:  {large_gaps_outer},{large_gaps_inner}')
    
    # If there are consistently spaced large gaps, it indicates a star-like pattern
    if large_gaps_inner < 4:
        return True
    if large_gaps_outer >= 5:  # Adjust this threshold based on your pattern characteristics
        return True  # Star-like pattern
    else:
        return False  # Continuous circle

def Find_HubFace(ledges, ledgeAvgs, reverse=False, deleteGround=True):
    # Make copies of the input lists to avoid modifying the originals
    ledges = deepcopy(ledges)
    ledgeAvgs = ledgeAvgs[:]
        # Count and sort ledges by the largest number of points
    ledgeNumPoints = [len(ledge[0]) for ledge in ledges]
    descendIndices_numPoints = np.argsort(ledgeNumPoints)[::-1]
    biggestLedge_numPoints = len(ledges[descendIndices_numPoints[0]][0])
    # print(f'\n\nledgeNumPoints: {ledgeNumPoints}')
    # print(f'descendIndices_numPoints: {descendIndices_numPoints}')

    # Sort ledges by z-value
    sortedIndices = np.argsort(ledgeAvgs)[::-1] if reverse else np.argsort(ledgeAvgs)

    # Delete all ledges beneath the biggest ledge of the entire scan
    while len(ledges[sortedIndices[0]][0]) / biggestLedge_numPoints <= 0.50:    # while biggest layer has 2x more points than bottom layer
        del ledges[sortedIndices[0]]                                            # bottom layer is likely noise. Delete it
        del ledgeAvgs[sortedIndices[0]]
        del ledgeNumPoints[sortedIndices[0]]
        sortedIndices = np.argsort(ledgeAvgs)[::-1] if reverse else np.argsort(ledgeAvgs)
        ledgeNumPoints = [len(ledge[0]) for ledge in ledges]
        descendIndices_numPoints = np.argsort(ledgeNumPoints)[::-1]
        biggestLedge_numPoints = len(ledges[descendIndices_numPoints[0]][0])

    if deleteGround:    # Bottom ledge is ground; skip it                    
        print('Ignoring Ground in Find_HubFace()')
        del ledges[sortedIndices[0]]
        del ledgeAvgs[sortedIndices[0]]
        del ledgeNumPoints[sortedIndices[0]]
        sortedIndices = np.argsort(ledgeAvgs)[::-1] if reverse else np.argsort(ledgeAvgs)
        ledgeNumPoints = [len(ledge[0]) for ledge in ledges]
        descendIndices_numPoints = np.argsort(ledgeNumPoints)[::-1]
        biggestLedge_numPoints = len(ledges[descendIndices_numPoints[0]][0])

    # Delete all ledges beneath the biggest ledge on the hub
    while len(ledges[sortedIndices[0]][0]) / biggestLedge_numPoints <= 0.50:
        del ledges[sortedIndices[0]]
        del ledgeAvgs[sortedIndices[0]]
        del ledgeNumPoints[sortedIndices[0]]
        sortedIndices = np.argsort(ledgeAvgs)[::-1] if reverse else np.argsort(ledgeAvgs)
        ledgeNumPoints = [len(ledge[0]) for ledge in ledges]
        descendIndices_numPoints = np.argsort(ledgeNumPoints)[::-1]
        biggestLedge_numPoints = len(ledges[descendIndices_numPoints[0]][0])

    # Identify if bottom or 2nd to bottom ledge is the hubface
    numPoints0 = len(ledges[sortedIndices[0]][0])
    numPoints1 = len(ledges[sortedIndices[1]][0])
    if numPoints1 / numPoints0 >= 0.15:                 # if 2nd at least 15% the size of the bottom, bottom is likely casting. 2nd is likely hubface           
        print('Checking if hubface spread is OK')       # check if 2nd to bottom has low spread. If not, it's actually vertical wall. 3rd is hubface
        _, _, spread = Calc_Plane(ledges[sortedIndices[1]], numPoints=5000)
        print(spread)
        
        if spread <= 0.1: # mm
            hubFace = ledges[sortedIndices[1]]              
            hubFaceAvg = ledgeAvgs[sortedIndices[1]]
        else:
            hubFace = ledges[sortedIndices[2]]              
            hubFaceAvg = ledgeAvgs[sortedIndices[2]]
    
    else:
        if Check_Not_Casting(ledges[sortedIndices[0]]): # if likely botton, make sure bottom isn't casting
            hubFace = ledges[sortedIndices[0]]          # the bottom is the hubface
            hubFaceAvg = ledgeAvgs[sortedIndices[0]]
        
        else:                                           # the 2nd is likely hubface
            print('Checking if hubface spread is OK')       # check if 2nd to bottom has low spread. If not, it's actually vertical wall. 3rd is hubface
            _, _, spread = Calc_Plane(ledges[sortedIndices[1]], numPoints=5000)
            print(spread)

            if spread <= 0.1: # mm
                hubFace = ledges[sortedIndices[1]]          
                hubFaceAvg = ledgeAvgs[sortedIndices[1]]
            else:
                hubFace = ledges[sortedIndices[2]]              
                hubFaceAvg = ledgeAvgs[sortedIndices[2]]

    return hubFace, hubFaceAvg

def Load_Demo_Data(filename, maxPoints, cutOff):
    # Load data from file
    data = np.loadtxt(filename, delimiter=',')

    # Get dimensions
    numProfiles, numPoints = data.shape
    print('Points per Profile: ', numPoints)
    print('Profiles: ', numProfiles)
    print('Raw Points: ', numPoints * numProfiles)

    # Generate x, y coordinates
    x, y = np.indices((numProfiles, numPoints))
    x = x.ravel()
    y = y.ravel()
    z = data.ravel()

    # Filter out negative z-values
    valid_mask = (z >= cutOff[0]) & (z <= cutOff[1])
    x = x[valid_mask]
    y = y[valid_mask]
    z = z[valid_mask]

    # Combine into a single array
    points = np.array([x, y, z])
    print('Valued Points: ', points.shape[1])

    # Sample points if necessary
    if points.shape[1] > maxPoints:
        sampled_indices = np.linspace(0, points.shape[1] - 1, maxPoints, dtype=int)
        points = points[:, sampled_indices]

    print('Sampled Points: ', points.shape[1])
    return points

def Load_Scan(filename, maxPoints, cutOff, correction=True):
    # Load data from file using memory-efficient method
    data = np.loadtxt(filename, delimiter=',', dtype=np.float64)

    # Get dimensions
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

    # Efficient xScale correction
    if correction:
        print('Applying xScale correction')
        xScaleSlope = ((579 - 640) / numPoints) / (1116 - 936)
        scale_factors = np.where(
            z <= 0,
            0.2,  # Default scale
            0.2 + xScaleSlope * z  # Adjusted scale
        )
        x *= scale_factors
    else:
        x *= 0.2  # Default scale

    # Combine into a single array
    points = np.stack((x, y, z), axis=0)
    print(f'Valued Points: {points.shape[1]}')

    # Downsample points if necessary
    if points.shape[1] > maxPoints:
        sampled_indices = np.linspace(0, points.shape[1] - 1, maxPoints, dtype=int)
        points = points[:, sampled_indices]

    print(f'Sampled Points: {points.shape[1]}\n')
    return points

def close_all_on_esc(event):
    if event.key == 'escape':  # Check if 'Esc' key is pressed
        plt.close('all')  # Close all figures

def Plot_Cloud_PyVista(points):
    cloud = pv.PolyData(points.T)
    print(f'Displaying {points.shape[1]} points')

     # Map z-values to the HSV colormap
    z_values = points[2]  # Assuming the z-values are the third row of 'points'
    cloud.point_data['z'] = z_values
    cloud.point_data.set_array(z_values, 'z')

    plotter = pv.Plotter()
    plotter.set_background('gray')
    plotter.add_mesh(cloud, scalars='z', cmap='coolwarm', point_size=1.0)
    plotter.show()

def Home_On_Hub(cloud, floorOffset=80, radius=100):
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

def Plot_Plane(Plane, Points, plotNum, title=0):
    # Check if the figure already exists
    if plt.fignum_exists(plotNum):
        fig = plt.figure(plotNum)
        ax = fig.gca()  # Get the current axes
    else:
        fig = plt.figure(plotNum)
        ax = fig.add_subplot(111, projection='3d')
    fig.canvas.mpl_connect('key_press_event', close_all_on_esc)

    xRange = abs(np.max(Points[0]) - np.min(Points[0]))
    yRange = abs(np.max(Points[1]) - np.min(Points[1]))
    xx, yy = np.meshgrid(np.linspace(-xRange/2, xRange/2, 10), 
                         np.linspace(-yRange/2, yRange/2, 10))
    zz = (-Plane[0]*xx - Plane[1]*yy - Plane[3]) * (1/Plane[2])

    # Plot the best-fitting plane
    ax.plot_surface(xx, yy, zz, alpha=0.5, color='yellow')
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlim(-plotWidth, plotWidth)
    ax.set_ylim(-plotWidth, plotWidth)
    ax.set_zlim(-plotWidth, plotWidth)
    if title != 0:
        ax.set_title(title)

    plt.tight_layout()

def Plot_Result(ledges, refPlane, hubFaceCloud, hubFacePlane, hubFaceAngleRel, plotNum=999, pointsPerLedge=1000):
    for ledge in ledges:
        if ledge.shape[1] > pointsPerLedge:
            sampled_indices = np.linspace(0, ledge.shape[1] - 1, pointsPerLedge, dtype=int)
            ledge = ledge[:, sampled_indices]
        Plot_Cloud(ledge, plotNum=plotNum)

    Plot_Plane(refPlane, ledges[0], plotNum=plotNum)
    Plot_Plane(hubFacePlane, hubFaceCloud, plotNum=plotNum)
    
    # Check if the figure already exists
    if plt.fignum_exists(plotNum):
        fig = plt.figure(plotNum)
        ax = fig.gca()  # Get the current axes
    else:
        fig = plt.figure(plotNum)
        ax = fig.add_subplot(111, projection='3d')
    fig.canvas.mpl_connect('key_press_event', close_all_on_esc)

    ax.set_title(f"Alignment Results (degrees)\nx: {hubFaceAngleRel[0]:.3f}, y: {hubFaceAngleRel[1]:.3f}, z: {hubFaceAngleRel[2]:.3f}")

    xRange = abs(np.max(hubFaceCloud[0]) - np.min(hubFaceCloud[0]))/2
    yRange = abs(np.max(hubFaceCloud[1]) - np.min(hubFaceCloud[1]))/2
    refHeight = np.min(ledges[0][2])
    resultDim = np.max([xRange, yRange])
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlim(-resultDim, resultDim)
    ax.set_ylim(-resultDim, resultDim)
    ax.set_zlim(refHeight, refHeight + 2*resultDim)
    plt.tight_layout()

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
        ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
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
    
    plt.tight_layout()

def make_orthogonal(matrix):
    """
    Enforces orthogonality of a given matrix using Singular Value Decomposition (SVD).
    
    Parameters:
        matrix (numpy.ndarray): A 3x3 matrix.
    
    Returns:
        numpy.ndarray: An orthogonal 3x3 matrix.
    """
    U, _, Vt = np.linalg.svd(matrix)
    R = np.dot(U, Vt)
    
    # Ensure determinant is 1 (right-handed system)
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = np.dot(U, Vt)
    
    return R

def enforce_rotation_properties(matrix):
    """
    Ensures a matrix is a valid rotation matrix by checking orthogonality and determinant.
    
    Parameters:
        matrix (numpy.ndarray): A 3x3 matrix.
    
    Returns:
        numpy.ndarray: A corrected 3x3 rotation matrix.
    """
    # Check orthogonality
    if not np.allclose(np.dot(matrix.T, matrix), np.eye(3), atol=1e-6):
        matrix = make_orthogonal(matrix)
    
    # Check determinant
    if not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-6):
        U, _, Vt = np.linalg.svd(matrix)
        matrix = np.dot(U, Vt)
    
    return matrix

def Rotation_to_Zaxis(plane):
    """
    Computes the rotation matrix to align a given normal vector with the z-axis.

    Parameters:
        normal (numpy.ndarray): A 3D vector [a, b, c].

    Returns:
        numpy.ndarray: A 3x3 rotation matrix.
    """
    # Normalize the input vector
    normal = np.array(plane[:3])
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

def Level_Ground(cloud, zRange=10, showGround=False):
    ground = cloud[:, cloud[2] <= (np.min(cloud[2]) + zRange)]
    if showGround:
        Plot_Cloud_PyVista(ground)

    groundPlane, groundAngle,_ = Calc_Plane(ground)
    R = Rotation_to_Zaxis(groundPlane)
    ground = np.dot(R, ground)
    groundPlane, groundAngle,_ = Calc_Plane(ground)
    if showGround:
        Plot_Cloud_PyVista(ground)

    cloud = np.dot(R, cloud)
    cloud[2] += groundPlane[3]
    return cloud

def Load_Sim_Scan(filename):
    data = np.loadtxt(filename, skiprows=1)#[::10]   # Take every 10th point. Bottleneck at np.linalg.svd() in Calc_Plane()
    x = data[:,0]
    y = data[:,1]
    z = data[:,2]#/100
    cloud = np.array([x, y, z])

    return cloud


if __name__ == "__main__":
    start = time.time()
    scan1 = Load_Scan(filename=r'Keyence Scans\Week Trial\FlatPlate\8201\8-201_12.csv', maxPoints=12400000, cutOff=[-162, 180], correction=True)
    
    # scan1 = Load_Sim_Scan('simScan0.txt')

    Plot_Cloud_PyVista(scan1)
    scan1_homed = Home_On_Hub(scan1, radius=180)
    # Plot_Cloud_PyVista(scan1_homed)
    scan1_homed = Level_Ground(scan1_homed, zRange=10, showGround=False)
    
    # Filter Noise
    scan1_homed = Filt_Raw_Points(scan1_homed, threshold=10, shortSeg=2)
    Plot_Cloud_PyVista(scan1_homed)

    # Categorize ledges
    myLedges, myLedgeAvgs = Find_Ledges(scan1_homed, ledgeThreshold=2.0, shortLedge=10, closeLedges=4.5) # Defaults: 2.0, 10, 4.5
    mySortedLedges, mySortedLedgeAvgs = Sort_Ledges(myLedges, myLedgeAvgs)
    # Plot_Cloud_PyVista(np.hstack(mySortedLedges))
    Plot_Ledges(mySortedLedges, mySortedLedgeAvgs)
    
    # Calculate relative angles
    print('\nCalculating angles')
        # Ground plane angle
    print('Reference plane')
    groundRef = mySortedLedges[0]
    # Plot_Cloud_PyVista(groundReference)
    refPlane, refAngle,_ = Calc_Plane(groundRef, title='Reference Plane', plotNum=1, numPoints=10000)
        # Hubface angle
    print('Hub Face')
    hubFace, hubFaceAvg = Find_HubFace(mySortedLedges, mySortedLedgeAvgs, reverse=False)
    # Plot_Cloud_PyVista(hubFace)
    hubFacePlane, hubFaceAngle,_ = Calc_Plane(hubFace, title='Hub Face Plane', plotNum=2, numPoints=10000)
        # Relative to reference
    refAngleRel = refAngle - refAngle
    hubFaceAngleRel = hubFaceAngle - refAngle
    
    end = time.time()
    # Display results
    # Plot_Cloud_PyVista(np.hstack((groundRef, hubFace)))
    Plot_Result(mySortedLedges, refPlane, hubFace, hubFacePlane, hubFaceAngleRel)
    
    # # Format the angles as a single string and save the result to results.txt
    # formatted_angles = ' '.join(f'{angle:.3f}' for angle in hubFaceAngleRel)
    # print('\nSaving results to local results.txt')
    # with open(r'Keyence Scans\Week Trial\FlatPlate\8257\results.txt', 'a') as file:
    #     file.write(formatted_angles + '\n')

    print(f'\nTotal Duration: {end-start:.3f}')
    plt.show()
