'''MAKE YOUR OWN COPY. DON'T PLAY WITH THIS SPAGHETTI'''
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from scipy.spatial import cKDTree
from concurrent.futures import ProcessPoolExecutor, as_completed
from psutil import Process, HIGH_PRIORITY_CLASS
from os import getpid
import pyvista as pv
'''MAKE YOUR OWN COPY. DON'T PLAY WITH THIS SPAGHETTI'''
# Set current Python process to high priority. THIS WILL BRICK YOUR COMPUTER UNTIL COMPLETED. Disable to avoid
inst = Process(getpid())
inst.nice(HIGH_PRIORITY_CLASS)

''' Capitalized functions are written or adapted from AI
    Lower-case function are copy-paste from AI'''

# Define global frame, origin and axes
    # z towards camera, x to right horizontal, y up
globalFrame = np.array([[0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
plotWidth = 200 # width of plots
'''MAKE YOUR OWN COPY. DON'T PLAY WITH THIS SPAGHETTI'''
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
    if title != 0:
        ax.set_title(title)
    

    ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.tight_layout()

def Plot_Frame(Frame, plotNum, title=0):
    # Check if the figure already exists
    if plt.fignum_exists(plotNum):
        fig = plt.figure(plotNum)
        ax = fig.gca()  # Get the current axes
    else:
        fig = plt.figure(plotNum)
        ax = fig.add_subplot(111, projection='3d')
    fig.canvas.mpl_connect('key_press_event', close_all_on_esc)

    ax.scatter(*Frame[:,0], color='black', label="Origin")
    ax.quiver(*Frame[:,0], *Frame[0, 1:], length=100, color='blue', normalize=True, label=r"$e_x$")
    ax.quiver(*Frame[:,0], *Frame[1, 1:], length=100, color='yellow', normalize=True, label=r"$e_y$")
    ax.quiver(*Frame[:,0], *Frame[2, 1:], length=100, color='green', normalize=True, label=r"$e_z$")

    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlim(-plotWidth, plotWidth)
    ax.set_ylim(-plotWidth, plotWidth)
    ax.set_zlim(-plotWidth, plotWidth)
    if title != 0:
        ax.set_title(title)

    ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.tight_layout()

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
    xx, yy = np.meshgrid(np.linspace(-xRange, xRange, 10), 
                         np.linspace(-yRange, yRange, 10))
    zz = (-Plane[0]*xx - Plane[1]*yy - Plane[3]) * (1/Plane[2])

    # Plot the best-fitting plane
    ax.plot_surface(xx, yy, zz, alpha=0.5, color='yellow', label=f'{Plane[0]:.3f}x + {Plane[1]:.3f}y + {Plane[2]:.3f} = {Plane[3]:.1f}')
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlim(-plotWidth, plotWidth)
    ax.set_ylim(-plotWidth, plotWidth)
    ax.set_zlim(-plotWidth, plotWidth)
    if title != 0:
        ax.set_title(title)

    ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.tight_layout()

def Plot_Ledges(ledges, ledgeAvgs, plotNum, series=0):
    # Check if the figure already exists
    if plt.fignum_exists(plotNum):
        fig = plt.figure(plotNum)
        ax = fig.gca()  # Get the current axes
    else:
        fig = plt.figure(plotNum)
        ax = fig.add_subplot(111, projection='3d')
    fig.canvas.mpl_connect('key_press_event', close_all_on_esc)
    
    numLedges = len(ledges)
    ax = fig.add_subplot(111, projection='3d')
    for i in range(numLedges):
        ax.scatter(ledges[i][0], ledges[i][1], ledges[i][2], s=1, label=f'{ledgeAvgs[i]:.2f}')
        ax.set_title(f'numLedges: {numLedges}')
    
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    ax.set_xlim(-plotWidth, plotWidth)
    ax.set_ylim(-plotWidth, plotWidth)
    ax.set_zlim(-plotWidth, plotWidth)
    
    # ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.tight_layout()

def Plot_Overlap(nodes, plotNum, label=0):
        ovlpCloud = np.array([node.center for node in nodes]).T
        Plot_Cloud(ovlpCloud, plotNum=plotNum, label='Overlap', s=20, c='black')

def close_all_on_esc(event):
    if event.key == 'escape':  # Check if 'Esc' key is pressed
        plt.close('all')  # Close all figures

def Random_Rotation():
    """
    Generates a random 3x3 rotation matrix.
    """
    # Generate a random quaternion
    q = np.random.randn(4)
    q /= np.linalg.norm(q)  # Normalize to make it a unit quaternion
    
    # Extract quaternion components
    w, x, y, z = q

    # Construct the rotation matrix
    R = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                  [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                  [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]])
    
    assert np.allclose(np.dot(R.T, R), np.eye(3)), "Random Rotation matrix is not orthogonal"
    assert np.isclose(np.linalg.det(R), 1.0), "Random Rotation matrix determinant is not 1"

    return R

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

def Rotation_to_Zaxis(normal):
    """
    Computes the rotation matrix to align a given normal vector with the z-axis.

    Parameters:
        normal (numpy.ndarray): A 3D vector [a, b, c].

    Returns:
        numpy.ndarray: A 3x3 rotation matrix.
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

def Bracket_Plane(point_cloud):
    """
    Detect cylindrical features in a 3D point cloud and return the axis of rotation.

    Parameters:
        point_cloud (numpy.ndarray): A 3 x N array representing the point cloud.

    Returns:
        tuple: A tuple containing:
            - axis (numpy.ndarray): A 3D vector representing the cylinder's axis direction.
            - center (numpy.ndarray): A 3D point on the cylinder's axis.
    """
    # Convert to Open3D point cloud
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(point_cloud.T)

    # Estimate normals (required for plane fitting)
    o3d_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # RANSAC to fit a plane
    bracket_plane, inliers = o3d_cloud.segment_plane(distance_threshold=0.1,
                                                         ransac_n=3,
                                                         num_iterations=1000)

    if bracket_plane is None:
        raise ValueError("No planar feature detected in the point cloud.")

    return bracket_plane

def Stitch_Close_Ledges(bigLedges, bigLedgeAvgs, closeBigLedges):
    # print('closeLedgePairs',closeBigLedges)
    stitchedBigLedges = []
    stitchedBigLedgeAvgs = []
    stitched_indices = set()
    
    for i in range(len(bigLedges)):
        involved_pairs = [pair for pair in closeBigLedges if i in pair] # Check if this index is part of any pair
        
        if involved_pairs:
            if i not in stitched_indices: # Combine all ledges connected to this one
                combined_ledge = bigLedges[i]
                combined_avg = bigLedgeAvgs[i]
                stitched_indices.add(i)
        
                for _, j in involved_pairs:
                    if j not in stitched_indices: # Combine the ledges
                        combined_ledge = np.column_stack((combined_ledge, bigLedges[j]))
                        combined_avg = np.mean(combined_ledge[2])
                        stitched_indices.add(j)

                # Add the combined ledge and its average
                stitchedBigLedges.append(combined_ledge)
                stitchedBigLedgeAvgs.append(combined_avg)
                
        else: # If the index isn't part of any pair, add it as-is
            stitchedBigLedges.append(bigLedges[i])
            stitchedBigLedgeAvgs.append(bigLedgeAvgs[i])
    
    # print('\nbigLedgeAvgs: ', bigLedgeAvgs)
    # print('\nstitchedBigLedgeAvgs: ', stitchedBigLedgeAvgs)

    return stitchedBigLedges, stitchedBigLedgeAvgs
    
def Find_Ledges(Points, ledgeThreshold, shortLedge, closeLedges):
    zVal = Points[2][0]
    currentPoint = np.array([[Points[0][0]], [Points[1][0]], [zVal]])
    
    ledges = [currentPoint]
    numLedges = 1
    ledgeAvgs = [zVal]
    # print(f'0\nCurrent Point: {currentPoint}')
    # print('numLedges:', numLedges)
    # print('ledges:', ledges)
    # print('ledgeAvgs:', ledgeAvgs)
    
    for i in range(1, len(Points[0])):
        foundSimilarLedge = False
        zVal = Points[2][i]
        currentPoint = np.array([[Points[0][i]], [Points[1][i]], [zVal]])
        
        for k in range(numLedges):
            avg = ledgeAvgs[k]
            
            if zVal > avg - ledgeThreshold and zVal < avg + ledgeThreshold:
                ledges[k] = np.column_stack((ledges[k], currentPoint))
                ledgeAvgs[k] = avg - (avg - zVal) / (ledges[k].size)
                foundSimilarLedge = True
                break 

        if not foundSimilarLedge:
            ledges.append(currentPoint)
            ledgeAvgs.append(zVal)
            numLedges += 1

        # print(f'\n{i}\nCurrent Point: {currentPoint}')
        # print('numLedges:', numLedges)
        # print('ledges:', ledges)
        # print(f'\n{i}\nledgeAvgs:', ledgeAvgs)
    bigLedges = []
    bigLedgeAvgs = []
    for i in range(len(ledges)):
        if len(ledges[i][0]) > shortLedge:
            bigLedges.append(ledges[i])
            bigLedgeAvgs.append(ledgeAvgs[i])

    '''Check if ledges need stitched to form plane. If do once, do again'''
    closeBigLedges = []
    needStitch = False
    for i in range(len(bigLedges)):
        for j in range(i+1, len(bigLedges)):
            if abs(bigLedgeAvgs[i] - bigLedgeAvgs[j]) <= closeLedges:
                closeBigLedges.append((i,j))
                needStitch = True

    if needStitch:
        bigLedges, bigLedgeAvgs = Stitch_Close_Ledges(bigLedges, bigLedgeAvgs, closeBigLedges)

        closeBigLedges = []
        needStitch = False
        for i in range(len(bigLedges)):
            for j in range(i+1, len(bigLedges)):
                if abs(bigLedgeAvgs[i] - bigLedgeAvgs[j]) <= closeLedges:
                    closeBigLedges.append((i,j))
                    needStitch = True

        bigLedges, bigLedgeAvgs = Stitch_Close_Ledges(bigLedges, bigLedgeAvgs, closeBigLedges)
    
    return bigLedges, bigLedgeAvgs

def Find_Bracket_Ledge(ledges, ledgeAvgs):
    sortedIndices = np.argsort(ledgeAvgs)
    sortedLedges = []
    sortedLedgeAvgs = []
    for i in sortedIndices:
        sortedLedges.append(ledges[i])
        sortedLedgeAvgs.append(ledgeAvgs[i]) 

    # print(" ".join(f'{avg:.1f}' for avg in sortedLedgeAvgs))

    shifts = np.diff(sortedLedgeAvgs)
    sortedShiftIndices = np.argsort(shifts)[::-1]
    # print(" ".join(f'{shifts[i]:.1f}' for i in sortedShiftIndices))
    # print(" ".join(f'{sortedLedgeAvgs[i]:.1f}' for i in sortedShiftIndices))

    numNegativeLedges = np.sum(np.array(sortedLedgeAvgs) < 0)
    # print(numNegativeLedges / len(sortedLedgeAvgs))
    if numNegativeLedges / len(sortedLedgeAvgs) >= 0.4:
        return sortedLedgeAvgs[sortedShiftIndices[1]], True
    else:
        return sortedLedgeAvgs[sortedShiftIndices[0]], False
    
def Check_Scaling(base_cloud, transformed_cloud):
    base_cloud = base_cloud - np.mean(base_cloud, axis=1)[:, np.newaxis]
    transformed_cloud = transformed_cloud - np.mean(transformed_cloud, axis=1)[:, np.newaxis]

    original_norm = np.linalg.norm(base_cloud, axis=0).mean()
    transformed_norm = np.linalg.norm(transformed_cloud, axis=0).mean()
    # print(original_norm)
    # print(transformed_norm)
    if np.isclose(original_norm, transformed_norm):
        # print("No distortion detected")
        pass
    else:
        print("Scaling distortion detected") 

def Locate_Bracket_Plane(filename, series):
    print(f'\n\nSeries {series}')

    # Load first 3D cloud
    data = np.loadtxt(filename, skiprows=1)#[::10]   # Take every 10th point. Bottleneck at np.linalg.svd() in Calc_Plane()
    x = data[:,0]
    y = data[:,1]
    z = data[:,2]#/100
    cloud = np.array([x, y, z])
    cloud0 = np.copy(cloud)  # Create a copy to keep the original unaltered
    cloud0.flags.writeable = False  # Disable writing to cloud0

    Plot_Cloud(cloud, plotNum=0, title='Virgin Cloud', label=f'Series {series}', center=True)
    # cloud = np.dot(Random_Rotation(), cloud)
    # Plot_All(cloud, plotNum=1+series, title='Rotated Cloud')
    
    # Align first cloud with global frame
        # Translate centroid to global origin
    centroid = np.mean(cloud, axis=1)
    # Plot_All(centroid, plotNum=1+series, s=10)
    cloud = cloud - centroid[:, np.newaxis]
    # Plot_Frame(globalFrame, plotNum=2+series)
    # Plot_All(cloud, plotNum=2+series, title='Rotated Cloud Centered', center=True)
        # Rotate hub towards positive z, bracket face in xy plane
    bracketPlane = Bracket_Plane(cloud) # bracket's normal vector in 0prime frame and normal distance from cloud0_prime origin
    bracketNormal = bracketPlane[0:3]
    # Plot_Plane(bracketPlane, cloud, plotNum=2+series)
    
    i = 0
    cloud_xy = np.copy(cloud)
    while True:
        # print(bracketNormal)
        R_to_Z = Rotation_to_Zaxis(bracketNormal)
        bracketNormal = np.dot(R_to_Z, bracketNormal)
        # print(f"Aligned Normal {i}:", bracketNormal0)
        cloud_xy = np.dot(R_to_Z, cloud_xy)
        skew = abs(bracketNormal[0] + bracketNormal[1])
        # print('skew: ', skew)
        # print(i)
        Check_Scaling(cloud0, cloud_xy)
        i += 1
        if skew < 1e-6 or i >= 10:
            break
    
    bracketPlane = Bracket_Plane(cloud_xy)
    if bracketPlane[-1] < 0:
        # print("Initial flip x")
        R_flipX = np.array([[1,  0,  0],
                            [0, -1,  0],
                            [0,  0, -1]])
        cloud_xy = np.dot(R_flipX, cloud_xy)

    cloud_xy[2] += abs(bracketPlane[3])

    myLedges, myLedgeAvgs = Find_Ledges(cloud_xy, ledgeThreshold=2.0, shortLedge=20, closeLedges=5.5)
    # Plot_Ledges(myLedges, myLedgeAvgs, plotNum=500, series=series)
    bracketHeight, needFlip = Find_Bracket_Ledge(myLedges, myLedgeAvgs)

    cloud_xy[2] -= bracketHeight
    if needFlip:
        # print("Second flip x")
        R_flipX = np.array([[1,  0,  0],
                            [0, -1,  0],
                            [0,  0, -1]])
        cloud_xy = np.dot(R_flipX, cloud_xy)

    bracketFace_xy = cloud_xy[:, ((cloud_xy[2] <= 20) & (cloud_xy[2] >= -20))]
    # Plot_All(cloud_xy, plotNum=30+series)
    bracketPlane_xy = Bracket_Plane(bracketFace_xy) # bracket's normal vector in 0prime frame and normal distance from cloud0_prime origin
    bracketNormal_xy = bracketPlane_xy[0:3]
    # Plot_Plane(bracketPlane_xy, cloud_xy, plotNum=30+series, title='Bracket Face before 2nd check')
    
    i = 0
    while True:
        R_to_Z_xy = Rotation_to_Zaxis(bracketNormal_xy)
        bracketNormal_xy = np.dot(R_to_Z_xy, bracketNormal_xy)
        # print(f"Aligned Normal {i}:", bracketNormal0)
        bracketFace_xy = np.dot(R_to_Z_xy, bracketFace_xy)
        cloud_xy = np.dot(R_to_Z_xy, cloud_xy)
        skew = abs(bracketNormal_xy[0] + bracketNormal_xy[1])
        # print('skew: ', skew)
        i += 1

        if skew < 1e-6 or i >= 10:
            break

    bracketPlane_xy = Bracket_Plane(bracketFace_xy)
    cloud_xy[2] += bracketPlane_xy[3]

    # Plot_All(cloud_xy, plotNum=31+series)
    # Plot_Plane(bracketPlane_xy, cloud_xy, plotNum=31+series, title='Bracket Face after 2nd check')


    # Show current result next to original
        # Position original at same height as result
    cloud0_at_origin = cloud0 - np.mean(cloud0, axis=1)[:, np.newaxis]
    myLedges0_at_origin, myLedgeAvgs0_at_origin = Find_Ledges(cloud0_at_origin, ledgeThreshold=2.0, shortLedge=20, closeLedges=5.5)
    bracketHeight0_at_origin, needFlip0_at_origin = Find_Bracket_Ledge(myLedges0_at_origin, myLedgeAvgs0_at_origin)
    cloud0_at_origin[2] -= bracketHeight0_at_origin

    # Plot_Frame(globalFrame, plotNum=999+series)
    # Plot_Cloud(cloud0_at_origin, plotNum=999+series, label='Virgin')
    # Plot_Cloud(cloud_xy, plotNum=999+series, label='Re-Aligned', title='Cloud aligned with xy', center=True)

    return cloud0, cloud_xy

def Alignment_Fitness(baseCloud, adjCloud, radius, plotNum=10, zWeightRamp=0.002, k=1.0):
    # one node centered at each base point, encapsulates any adjustable points within sphere of given radius
        # large radius for rough alignment
        # small radius for fine tuning
    class Node:
        def __init__(self, center, neighbors):
            self.center = center
            self.neighbors = neighbors
            self.has_neighbor = len(neighbors) > 0
            self.error = np.nan
            self.overlapScore = np.nan
            self.errorScore = np.nan
            self.overlapWeight = 1.0
            self.errorWeight = 1.0
            self.weight = 1.0
            self.k = 1.0

        def __repr__(self):
            return f"Node(center = [{" ".join(f'{cord:.2f}' for cord in self.center)}], neighbors = {len(self.neighbors)}, Scores: ovlp={self.overlapScore:.3f}, err={self.errorScore:.3f})"
        
        # node overlap score
            # count how many adjustable points in node
        def calcOverlapScore(self):
            self.overlapScore = len(self.neighbors) * self.overlapWeight

        # node error score
            # calculate vector from node center to each adjustable point
            # score each pair's error by spring force, kx. k is uniform between all nodes
            # 3D sum of error force vectors to get total node error
        def calcError(self):
            if not self.has_neighbor:
                self.error = np.nan
                return

            errorVectors = self.neighbors - self.center
            netError = np.sum(errorVectors, axis=0)
            self.error = self.k * np.linalg.norm(netError)
        
        def calcErrorScore(self):
            self.calcError()
            self.errorScore = self.error * self.errorWeight

        def setWeights(self, weight=1.0, overlapWeight=1.0, errorWeight=1.0, k=1.0):
            self.weight = weight
            self.overlapWeight = overlapWeight
            self.errorWeight = errorWeight
            self.k = k

        def calcScores(self):
            self.calcOverlapScore()
            self.calcErrorScore()

    # populate nodes
    adjTree = cKDTree(adjCloud.T) # row to column vectors
    baseNodes = [None] * baseCloud.shape[1] # list of len(numBasePoints) for each node

    for i, basePoint in enumerate(baseCloud.T):
        indices = adjTree.query_ball_point(basePoint, radius)
        neighbors = adjCloud[:, indices].T
        baseNodes[i] = Node(center=basePoint, neighbors=neighbors)

    ovlpNodes = [node for node in baseNodes if node.has_neighbor]
    # Plot_Overlap(ovlpNodes, plotNum=plotNum)

    # weight nodes according to height
        # nodes on and below bracket face have same base weight
        # linearly less weight with height above bracket
    # weight overlap and error accoring to height
        # weight overlap more / error less in bracket
        # weight overlap less / error more above bracket
    z_cut = 10
    for node in ovlpNodes:
        z = node.center[2]
        if z <= z_cut:
            a = 1.0
            b = 1.0
            c = 0.5
        else:
            a = 1.0 + zWeightRamp*(z_cut - z)
            if a < 0.0: 
                a = 0.0
            b = 1.0 + 3*zWeightRamp*(z_cut - z)
            if b < 0.0:
                b = 0.0
            c = 0.5 - 3*zWeightRamp*(z_cut - z)
            if c > 1.0:
                c = 1.0

        node.setWeights(weight=a, overlapWeight=b, errorWeight=c, k=k)
        node.calcScores()            

    # sum scores
    overlapScore = 0.0
    errorScore = 0.0
    for node in ovlpNodes:
        # print(node)
        overlapScore += node.overlapScore
        errorScore += node.errorScore

    # overlap increases score, error decreases score
    netScore = overlapScore - errorScore

    # return score. net high is good
    return np.array([netScore, overlapScore, errorScore])

def Rotate_Z(baseCloud, adjCloud, startAngle, endAngle, numAngles, plotNum):
    z_angles = np.linspace(startAngle, endAngle, numAngles)
    rotation_scores = []
    R_z = []
    for angle in z_angles:
        R_z.append(np.array([[np.cos(angle), -np.sin(angle), 0],
                             [np.sin(angle),  np.cos(angle), 0],
                             [0            ,              0, 1]]) )
    with ProcessPoolExecutor() as executor:
        for i, R in enumerate(R_z):
            futures = [executor.submit(Alignment_Fitness, baseCloud, np.dot(R, adjCloud), radius=10.0, plotNum=10+i, zWeightRamp=0.002, k=0.1)]

            for future in as_completed(futures):
                scores = future.result()
                rotation_scores.append(scores)    

    rotation_scores = np.array(rotation_scores)
    index = np.where(rotation_scores[:,1] == np.max(rotation_scores[:,1]))[0][0]
   
    Plot_Frame(globalFrame, plotNum=10+plotNum, title=f'{z_angles[index]*180/np.pi:.1f} deg Rotation\n\
               Net:{rotation_scores[index,0]:.1f}, Ovlp:{rotation_scores[index,1]:.1f}, Err:{rotation_scores[index,2]:.1f}')
    Plot_Cloud(baseCloud, plotNum=10+plotNum)
    Plot_Cloud(np.dot(R_z[index], adjCloud), plotNum=10+plotNum)

    plt.figure(plotNum)
    plt.plot(z_angles*180/np.pi, rotation_scores[:,0], label='Net')
    plt.plot(z_angles*180/np.pi, rotation_scores[:,1], label='Overlap')
    plt.plot(z_angles*180/np.pi, rotation_scores[:,2], label='Error')

    plt.legend()

    return z_angles[index], R_z[index], rotation_scores[index,1]

def Step_Into_Sync(baseCloud, adjCloud, Tz, score, numTests=10, rtol=1e-6, maxSteps=10):
    bestTxy = Tz
    bestScore = -1e6
    prevBestScore = bestScore
    steps = 0

    a = 1*np.pi/180 # radians
    b = 0.5 # mm
    improvedEnough = True
    while steps < maxSteps and improvedEnough:
        steps += 1
        print(f'step {steps}')
        transformations = []
        transformationScores = []

        # Apply 10 different random transformations to cloud_2
        for _ in range(numTests):
            randTxy = np.copy(bestTxy)
            # Random rotation about z and translation in xy 
            randRz = np.random.uniform(-a, a, (2, 2))
            rand_txy = np.random.uniform(-b, b, (2,1))

            randTxy[:2, :2] += randRz
            randTxy[[0,1], 3] += rand_txy.flatten()

            # print(randTxy)
            
            # Assess transformation
            transformedAdjCloud = np.dot(randTxy[:, :3], adjCloud) + randTxy[:, 3][:, np.newaxis]
            score = Alignment_Fitness(baseCloud, transformedAdjCloud, radius=1.0, zWeightRamp=0.001, k=2.0)
            
            transformations.append(randTxy)
            transformationScores.append(score)

        # Find the best transformation (the one with the highest overlap score)
        transformationScores = np.array(transformationScores)
        maxScoreIdx = np.argmax(transformationScores[:,1])
        maxScore = transformationScores[maxScoreIdx]

        if maxScore[0] > bestScore:
            print(f'Max Score for step: {maxScore[0]:.1f}')
            print(f'Previous Best Score: {prevBestScore:.1f}')
            bestScore = maxScore[0]
            bestTxy = transformations[maxScoreIdx]
            

            # If the score hasn't changed much, stop the optimization process
            if abs((bestScore - prevBestScore) / bestScore) < rtol:
                print('here2')
                print(abs((bestScore - prevBestScore) / bestScore))
                improvedEnough = False

            prevBestScore = bestScore

    return bestTxy, maxScore

def best_fit_align_clouds(adjCloud, baseCloud, threshold=0.02, max_iteration=50):
    """
    Aligns a source point cloud to a target point cloud using the ICP algorithm.

    Parameters:
        source_cloud (o3d.geometry.PointCloud): The source point cloud to align.
        target_cloud (o3d.geometry.PointCloud): The target point cloud to align to.
        threshold (float): Distance threshold for considering correspondences.
        max_iteration (int): Maximum number of iterations for the ICP algorithm.

    Returns:
        o3d.geometry.PointCloud: The transformed source point cloud.
        np.ndarray: The 4x4 transformation matrix.
        float: Final fitness score (higher is better, ranges from 0 to 1).
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
    
    # Apply the transformation to the source point cloud
    transformed_source = source_cloud.transform(reg_result.transformation)

    # o3d.visualization.draw_geometries([transformed_source.paint_uniform_color([1, 0, 0]), 
    #                                    target_cloud.paint_uniform_color([0, 1, 0])])
    
    return transformed_source, reg_result.transformation, reg_result.fitness

def Align_Z(baseCloud, adjCloud, series):
    # find best pure z rotation for overlap
        # course search full revolution
    print("Course Rotation")
    bestAngle, Rz, ovlpScore = Rotate_Z(baseCloud, adjCloud, startAngle=-np.pi, endAngle=np.pi, numAngles=21, plotNum=900+series)
        # fine search around max
    print("Fine Rotation")
    bestAngle, Rz, ovlpScore = Rotate_Z(baseCloud, adjCloud, startAngle=bestAngle-np.pi/18, endAngle=bestAngle+np.pi/18, numAngles=11, plotNum=901+series)

    # Randomly translate along x and y and rotate around z in tiny steps to maximize overlap
    Tz = np.zeros((3,4))
    Tz[:, :3] = Rz

    print("ICP")
    rotatedCloud = np.dot(Tz[:, :3], adjCloud) + Tz[:, 3][:, np.newaxis]
    transformedAdjCloud, Txy, b = best_fit_align_clouds(rotatedCloud, baseCloud, threshold=5.0, max_iteration=1000)

    Txy = Txy[:-1, :]

    # Txy, finalScores = Step_Into_Sync(cloud1, cloud2, Tz, ovlpScore, numTests=3, maxSteps=100)
    # print(f"Final optimized score: {finalScores}")

    transformedAdjCloud = np.dot(Txy[:, :3], rotatedCloud) + Txy[:, 3][:, np.newaxis]
    Plot_Frame(globalFrame, plotNum=920+series, title=f'Best Transformation')#\n\
            #    Net:{finalScores[0]:.1f}, Ovlp:{finalScores[1]:.1f}, Err:{finalScores[2]:.1f}')
    Plot_Cloud(baseCloud, plotNum=920+series)
    Plot_Cloud(transformedAdjCloud, plotNum=920+series)

    return transformedAdjCloud

def Plot_Cloud_PyVista(points, pointSize=1.0):
    cloud = pv.PolyData(points.T)
    print(f'Displaying {points.shape[1]} points')

     # Map z-values to the HSV colormap
    z_values = points[2]  # Assuming the z-values are the third row of 'points'
    cloud.point_data['z'] = z_values
    cloud.point_data.set_array(z_values, 'z')

    plotter = pv.Plotter()
    plotter.set_background('gray')
    plotter.add_mesh(cloud, scalars='z', cmap='coolwarm', point_size=pointSize)
    plotter.show()


'''MAKE YOUR OWN COPY. DON'T PLAY WITH THIS SPAGHETTI'''
if __name__ == "__main__":
    Plot_Frame(globalFrame, plotNum=0, title='Global Frame')

    # Place bracket face in xy plane with hub in +z
    cloud1_ground, cloud1 = Locate_Bracket_Plane(filename=r'Simulated Scans\simScan0b.txt', series=1000)
    cloud2_ground, cloud2 = Locate_Bracket_Plane(filename=r'Simulated Scans\simScan1b.txt', series=2000)
    cloud3_ground, cloud3 = Locate_Bracket_Plane(filename=r'Simulated Scans\simScan2b.txt', series=3000)
        # Show current result
    # Plot_Frame(globalFrame, plotNum=1, title='Combined Scans')
    # Plot_Cloud(cloud1, plotNum=1)
    # Plot_Cloud(cloud2, plotNum=1)
    # Plot_Cloud(cloud3, plotNum=1)
    Plot_Cloud_PyVista(np.hstack((cloud1_ground, cloud2_ground, cloud3_ground)))
    
    '''MAKE YOUR OWN COPY. DON'T PLAY WITH THIS SPAGHETTI'''
    # # Align second with first using Alignment Scoring function
    # cloud2_aligned = Align_Z(cloud1, cloud2, series=2000)
    # cloud1_2 = np.hstack((cloud1, cloud2_aligned))
    # # Plot_Cloud(cloud1_2, plotNum=2, title='Aligned Scans 1&2')

    # # Align 1&2 with 3 
    # cloud3_aligned = Align_Z(cloud1_2, cloud3, series=3000)
    # cloud1_2_3 = np.hstack((cloud1_2, cloud3_aligned))
    # Plot_Cloud(cloud1_2_3, plotNum=3, title='Aligned Scans 1&2&3', center=True)

    #     # Show final result
    # Plot_Frame(globalFrame, plotNum=99, title='Final Combined Scans')
    # Plot_Cloud(cloud1, plotNum=99, label='Series 1000')
    # Plot_Cloud(cloud2_aligned, plotNum=99, label='Series 2000')
    # Plot_Cloud(cloud3_aligned, plotNum=99, label='Series 3000')

    plt.show()
'''MAKE YOUR OWN COPY. DON'T PLAY WITH THIS SPAGHETTI'''