import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import pycpd


def translate_item(point, x_offset, z_offset):
    """
    Apply a translation to the x and z coordinates of a point, leaving y unchanged.

    Parameters:
    - point: A numpy array or list with three coordinates [x, y, z].
    - x_offset: The translation offset to apply to the x-coordinate.
    - z_offset: The translation offset to apply to the z-coordinate.

    Returns:
    - A numpy array with the translated point.
    """
    point = np.asarray(point)  # Ensure the input is a numpy array
    if point.size != 3:
        raise ValueError(f"Input point must have exactly 3 elements, got {point}")
    point[0] += x_offset  # Apply translation to x
    point[2] += z_offset  # Apply translation to z
    return point



def preprocess_point_cloud(pcd, voxel_size):
    # Downsample the point cloud
    pcd_down = pcd
    
    # Estimate normals
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    
    # Compute FPFH features
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=100)
    )
    
    return pcd_down, fpfh

def icp_translate_only_per_iteration(source, target, threshold, max_iterations=50):
    """
    Perform ICP alignment restricted to translation-only transformations,
    with adjustments applied after each iteration.

    Parameters:
    - source: Source point cloud
    - target: Target point cloud
    - threshold: Maximum correspondence distance
    - max_iterations: Number of ICP iterations to perform

    Returns:
    - ICP result with transformation constrained to translation only
    """
    # Initialize variables
    current_transformation = np.eye(4)
    source_transformed = source

    for i in range(max_iterations):
        # Perform one iteration of ICP
        icp_result = o3d.pipelines.registration.registration_icp(
            source_transformed,
            target,
            threshold,
            current_transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        # Extract the transformation matrix
        transformation = icp_result.transformation

        # Constrain the transformation to translation only
        translation_only = np.eye(4)
        translation_only[:3, 3] = transformation[:3, 3]  # Keep only the translation

        # Update the cumulative transformation
        current_transformation = np.dot(translation_only, current_transformation)

        # Apply the updated transformation to the source
        source_transformed = source_transformed.transform(translation_only)

        print(f"Iteration {i + 1}/{max_iterations}, Transformation:\n{current_transformation}")

        # Stop if the transformation is small (converged)
        if np.allclose(translation_only, np.eye(4), atol=1e-6):
            print("Converged.")
            break

    # Create a result object to return
    result = o3d.pipelines.registration.RegistrationResult()
    result.transformation = current_transformation
    result.fitness = icp_result.fitness
    result.inlier_rmse = icp_result.inlier_rmse

    return result

def ransac(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 3

    # Perform RANSAC registration with mutual_filter=True
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target,
        source_fpfh, target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=50,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
    )

    return result_ransac

def fgr(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 3

    # Create the FastGlobalRegistrationOption object
    option = o3d.pipelines.registration.FastGlobalRegistrationOption(
        division_factor=1.4,
        use_absolute_scale=False,
        decrease_mu=True,
        maximum_correspondence_distance=distance_threshold,
        iteration_number=64,
        tuple_scale=0.95,
        maximum_tuple_count=1000,
        tuple_test=True
    )

    # Perform FGR (Fast Global Registration)
    result_fgr = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source, target,
        source_fpfh, target_fpfh,
        option
    )

    return result_fgr

def fgr_translate_only(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 3

    # Create the FastGlobalRegistrationOption object
    option = o3d.pipelines.registration.FastGlobalRegistrationOption(
        division_factor=1.4,
        use_absolute_scale=False,
        decrease_mu=True,
        maximum_correspondence_distance=distance_threshold,
        iteration_number=64,
        tuple_scale=0.95,
        maximum_tuple_count=1000,
        tuple_test=True
    )

    # Perform FGR (Fast Global Registration)
    result_fgr = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source, target,
        source_fpfh, target_fpfh,
        option
    )

    # Constrain the resulting transformation to translation only
    transformation = result_fgr.transformation
    translation_only = np.eye(4)
    translation_only[:3, 3] = transformation[:3, 3]  # Copy translation components

    # Update the transformation in the result
    result_fgr.transformation = translation_only

    return result_fgr

def icp(source, target, voxel_size, threshold):
    # Perform ICP (Iterative Closest Point) alignment for fine-tuning
    
    icp_result = o3d.pipelines.registration.registration_icp(
        source,  # Source
        target,  # Target
        threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    return icp_result

def display_point_cloud(cloud, title="Point Cloud"):
    arr = np.asarray(cloud.points)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x = arr[:, 0]
    y = arr[:, 1]
    z = arr[:, 2]
    ax.scatter(x, y, z, c='b', marker='o', s=1)
    ax.set_title(title)
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    plt.show()

def multi_scale_alignment(source, target, voxel_sizes, threshold):
    """
    Perform Multi-Scale Alignment (MSA) on the point clouds.
    
    Parameters:
    - source: the source point cloud
    - target: the target point cloud
    - voxel_sizes: list of voxel sizes for multi-scale alignment
    - threshold: the ICP threshold for fine alignment
    
    Returns:
    - final_alignment: the final transformed source point cloud
    """
    current_source = source
    current_target = target
    
    # Perform registration at each scale (starting from coarse to fine)
    for voxel_size in voxel_sizes:
        print(f"Aligning at voxel size: {voxel_size}")
        
        # Preprocess the point clouds (downsample and estimate normals)
        source_down, fpfh1 = preprocess_point_cloud(current_source, voxel_size)
        target_down, fpfh2 = preprocess_point_cloud(current_target, voxel_size)
        
        # Perform ICP alignment
        icp_result = o3d.pipelines.registration.registration_icp(
            source_down, 
            target_down, 
            threshold, 
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        
        # Apply the transformation
        current_source.transform(icp_result.transformation)
        
        # Display the results at this scale (optional)
        # o3d.visualization.draw_geometries([current_source, current_target])
        
    return current_source

def cpd_registration(source_np, target_np):

    # Perform CPD rigid registration
    cpd = pycpd.registration.RigidRegistration(X=source_np, Y=target_np)
    result = cpd.register()

class TransformationEstimationTranslationOnly(o3d.pipelines.registration.TransformationEstimation):
    def compute_transformation(self, source, target, correspondence_set):
        # Estimate the transformation using only translation
        source_points = np.asarray(source.points)
        target_points = np.asarray(target.points)
        
        correspondences = np.asarray(correspondence_set)
        source_corr = source_points[correspondences[:, 0]]
        target_corr = target_points[correspondences[:, 1]]
        
        # Compute the mean translation
        translation = np.mean(target_corr - source_corr, axis=0)
        
        # Create the transformation matrix (translation only)
        transformation = np.eye(4)
        transformation[:3, 3] = translation
        
        return transformation

def ransac_translate_only(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 5

    # Perform RANSAC with a default estimation method
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target,
        source_fpfh, target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=50,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
    )

    # Constrain the resulting transformation to translation only
    transformation = result_ransac.transformation
    translation_only = np.eye(4)
    translation_only[:3, 3] = transformation[:3, 3]  # Copy translation components

    # Update the transformation in the result
    result_ransac.transformation = translation_only

    return result_ransac

def ransac_translate_only_xz(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 3

    # Perform RANSAC with a default estimation method
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target,
        source_fpfh, target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=50,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
    )

    # Constrain the resulting transformation to translation only in x and z axes
    transformation = result_ransac.transformation
    translation_only_xz = np.eye(4)
    translation_only_xz[:3, 3] = transformation[:3, 3]  # Copy translation components
    translation_only_xz[1, 3] = 0  # Zero out translation in the y-axis

    # Update the transformation in the result
    result_ransac.transformation = translation_only_xz

    return result_ransac

def segment_plane(point_cloud, distance_threshold=0.01, ransac_n=3, num_iterations=1000):
    """
    Segment the dominant plane from the point cloud.

    Parameters:
    - point_cloud: The input point cloud.
    - distance_threshold: Maximum distance a point can have to the plane to be considered an inlier.
    - ransac_n: The number of points to sample for plane fitting.
    - num_iterations: Number of RANSAC iterations.

    Returns:
    - plane_model: The coefficients of the plane equation (a, b, c, d) for ax + by + cz + d = 0.
    - inliers: Indices of the points that belong to the plane.
    """
    plane_model, inliers = point_cloud.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations,
    )
    return plane_model, inliers


def filter_points_on_plane(point_cloud, inliers):
    """
    Extract points that belong to the identified plane.

    Parameters:
    - point_cloud: The input point cloud.
    - inliers: Indices of the points that are part of the plane.

    Returns:
    - plane_cloud: The filtered point cloud containing only plane points.
    """
    plane_cloud = point_cloud.select_by_index(inliers)
    return plane_cloud

def ransac_translate_only_same_plane(source, target, source_fpfh, target_fpfh, voxel_size):
    """
    Perform RANSAC alignment, considering only points on the same plane.
    
    Parameters:
    - source: Source point cloud.
    - target: Target point cloud.
    - source_fpfh: FPFH features of the source point cloud.
    - target_fpfh: FPFH features of the target point cloud.
    - voxel_size: Voxel size for downsampling.
    
    Returns:
    - result_ransac: RANSAC alignment result with translation only.
    """
    # Segment the dominant plane in both source and target point clouds
    source_plane_model, source_inliers = segment_plane(source)
    target_plane_model, target_inliers = segment_plane(target)

    # Extract points on the plane
    source_plane = filter_points_on_plane(source, source_inliers)
    target_plane = filter_points_on_plane(target, target_inliers)

    # Perform RANSAC alignment on the filtered plane points
    distance_threshold = voxel_size * 3
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_plane, target_plane,
        source_fpfh, target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=100,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500),
    )

    # Constrain the resulting transformation to translation only
    transformation = result_ransac.transformation
    translation_only = np.eye(4)
    translation_only[:3, 3] = transformation[:3, 3]  # Copy translation components

    # Update the transformation in the result
    result_ransac.transformation = translation_only

    return result_ransac


# this function will take in two three-column np arrays, and merge them. it will return the merged np array
def merge(npArray1, npArray2, voxel_size=40, showPlot=False, writeExternal=False):
    cloud1 = o3d.geometry.PointCloud()
    cloud2 = o3d.geometry.PointCloud()
    translate_vectorized = np.vectorize(translate_item)

    # Create point clouds with translation applied to cloud1
    translated_points = np.apply_along_axis(translate_item, axis=1, arr=npArray1, x_offset=30, z_offset=30)
    cloud1.points = o3d.utility.Vector3dVector(translated_points)


    # cloud1.points = o3d.utility.Vector3dVector(translate_vectorized(npArray1, 30, 30))
    cloud2.points = o3d.utility.Vector3dVector(npArray2)

    display_point_cloud(cloud1 + cloud2, "Initial Clouds")

    # Perform feature matching
    cloud1Down, fpfh1 = preprocess_point_cloud(cloud1, voxel_size)
    cloud2Down, fpfh2 = preprocess_point_cloud(cloud2, voxel_size)

    # Perform RANSAC alignment
    result_ransac = ransac_translate_only_same_plane(cloud1Down, cloud2Down, fpfh1, fpfh2, voxel_size)
    print("RANSAC Transformation:")
    print(result_ransac.transformation)
    cloud1.transform(result_ransac.transformation)
    display_point_cloud(cloud1 + cloud2, "RANSAC Result")

    # cloud1Down, fpfh1 = preprocess_point_cloud(cloud1, voxel_size)
    # cloud2Down, fpfh2 = preprocess_point_cloud(cloud2, voxel_size)
    # result_ransac = ransac_translate_only(cloud1Down, cloud2Down, fpfh1, fpfh2, voxel_size)
    # print("RANSAC Transformation:")
    # print(result_ransac.transformation)
    # cloud1.transform(result_ransac.transformation)
    # display_point_cloud(cloud1 + cloud2, "RANSAC Result 2")

    # # Perform FGR alignment
    # result_fgr = fgr_translate_only(cloud1Down, cloud2Down, fpfh1, fpfh2, voxel_size)
    # print("FGR Transformation:")
    # print(result_fgr.transformation)
    # cloud1.transform(result_fgr.transformation)
    # display_point_cloud(cloud1 + cloud2, "FGR Result")

    # # Perform Multi-Scale Alignment
    # aligned_cloud1 = multi_scale_alignment(cloud1, cloud2, voxel_sizes=[40, 20, 10], threshold=1.0)
    # display_point_cloud(aligned_cloud1 + cloud2, "MSA Result")

    # Perform ICP (Iterative Closest Point) alignment for fine-tuning
    icp_result = icp_translate_only_per_iteration(cloud1, cloud2, voxel_size, voxel_size)
    print("ICP Transformation:")
    print(icp_result.transformation)
    cloud2.transform(icp_result.transformation)
    display_point_cloud(cloud1 + cloud2, "ICP Result")

    # # Merge the aligned point clouds
    # mergedCloud =  cloud1 + cloud2
    # mergedCloud, _ = mergedCloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # if writeExternal:
    #     o3d.io.write_point_cloud("merged_point_cloud.ply", mergedCloud)

    # # Convert back to numpy array for output
    # outputArray = np.asarray(mergedCloud.points)

    # return outputArray

if __name__ == '__main__':
    leftShort = np.load('left_pi_over_12_short.npy')
    rightShort = np.load('right_pi_over_12_short.npy')

    right = np.load('right_side_pi_over_12.npy')
    left = np.load('left_side_pi_over_12.npy')
    top = np.load('top_side_pi_over_12.npy')
    bottom = np.load('bottom_side_pi_over_12.npy')
    merge(leftShort, rightShort, showPlot=True)
