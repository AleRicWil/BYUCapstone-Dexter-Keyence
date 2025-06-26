import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.sparse.linalg import svds
import scipy.stats
# from mpl_toolkits.mplot3d import Axes3D

def Filt_Raw_Points(Points, threshold, shortSeg):
    z = Points[2]
    # print(Points)
    # print(z)
    # Calculate the difference between consecutive elements
    diff = np.diff(z)
    
    # Find indices where the absolute difference is greater than the threshold
    shift_indices = np.where(np.abs(diff) > threshold)[0]
    # print("Shift indices:", shift_indices)
    
    # Adjust to reflect the original vector's indices
    shift_indices = np.concatenate(([0], shift_indices + 1, [len(z)]))
    
    # Initialize list to store filtered segments
    # filtPoints = np.empty(3)

    # The following block of code is pretty jank
    # it does the first iteration of the following 
    # for-loop manually. refactor later
    segment = Points[:,shift_indices[0]:shift_indices[1]]
    # print('points segment', segment)
    # Apply outlier filtering to the segment (IQR method here)
    Q1 = np.percentile(segment[2], 25)
    Q3 = np.percentile(segment[2], 75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    # Filter out values outside the bounds
    filtered_segment = segment[:,(segment[2] >= lower_bound) & (segment[2] <= upper_bound)]
    # print('filt segment',filtered_segment)
    # Append the filtered segment to the list
    filtPoints = filtered_segment

    # Loop through each segment between shifts
    for i in range(1, len(shift_indices) - 1):
        # Extract the segment of z between current and next shift index
        segment = Points[:,shift_indices[i]:shift_indices[i + 1]]
        # print('points segment', segment)
        # Apply outlier filtering to the segment (IQR method here)
        Q1 = np.percentile(segment[2], 25)
        Q3 = np.percentile(segment[2], 75)
        IQR = Q3 - Q1
        lower_bound = Q1 - 1.5 * IQR
        upper_bound = Q3 + 1.5 * IQR
        
        # Filter out values outside the bounds
        filtered_segment = segment[:,(segment[2] >= lower_bound) & (segment[2] <= upper_bound)]
        # print(len(filtered_segment[2]))
        if len(filtered_segment[2]) >= shortSeg:
            # print('filt segment',filtered_segment[2])
            # Append the filtered segment to the list
            filtPoints = np.concatenate((filtPoints, filtered_segment), axis=1)
        # print(i)
    
    # Return filtered segments
    # print("filtPoints final:", filtPoints)
    return filtPoints

def detect_shifts(z, threshold):
    # Calculate the difference between consecutive elements
    diff = np.diff(z)
    
    # Find indices where the absolute difference is greater than the threshold
    shift_indices = np.where(np.abs(diff) > threshold)[0]
    # print(shift_indices)

    # Return the indices (adjusted to reflect the original vector's indices)
    return shift_indices + 1  # Add 1 because diff reduces the length by 1

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

def Calc_Plane(points, title=0, plotNum=0):
    fig = plt.figure(plotNum)
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[0], points[1], points[2], color='red', s=1)
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    svd = np.linalg.svd(points - np.mean(points, axis=1, keepdims=True)) # Subtract out the centroid and take the SVD
    left = svd[0] # Extract the left singular vectors
    normalVector = left[:, -1] # The normal vector to the plane is the last singular vector
    centroid = np.mean(points, axis=1) # Calculate the centroid of the points
    d = -normalVector.dot(centroid)

    '''Start Filter Plane Outliers'''
    orthoDist = np.abs(normalVector[0]*points[0] +
                       normalVector[1]*points[1] +
                       normalVector[2]*points[2] + d) / np.linalg.norm(normalVector)
    Q1 = np.percentile(orthoDist, 25)
    Q3 = np.percentile(orthoDist, 75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 0.5 * IQR
    upper_bound = Q3 + 0.5 * IQR
    
    filtPoints = points[:,(orthoDist >= lower_bound) & (orthoDist <= upper_bound)]

    svdF = np.linalg.svd(filtPoints - np.mean(filtPoints, axis=1, keepdims=True)) # Subtract out the centroid and take the SVD
    leftF = svdF[0] # Extract the left singular vectors
    normalVectorF = leftF[:, -1] # The normal vector to the plane is the last singular vector
    centroidF = np.mean(filtPoints, axis=1) # Calculate the centroid of the points
    dF = -normalVectorF.dot(centroidF)
    '''Second Filter Plane Outliers'''
    orthoDistF = np.abs(normalVectorF[0]*filtPoints[0] +
                       normalVectorF[1]*filtPoints[1] +
                       normalVectorF[2]*filtPoints[2] + dF) / np.linalg.norm(normalVectorF)
    Q1F = np.percentile(orthoDistF, 25)
    Q3F = np.percentile(orthoDistF, 75)
    IQRF = Q3F - Q1F
    lower_boundF = Q1F - 1.0 * IQRF
    upper_boundF = Q3F + 1.0 * IQRF
    
    filtPoints2 = filtPoints[:,(orthoDistF >= lower_boundF) & (orthoDistF <= upper_boundF)]
    ax.scatter(filtPoints2[0], filtPoints2[1], filtPoints2[2], color='green', s=10)
    
    svdF2 = np.linalg.svd(filtPoints2 - np.mean(filtPoints2, axis=1, keepdims=True)) # Subtract out the centroid and take the SVD
    leftF2 = svdF2[0] # Extract the left singular vectors
    normalVectorF2 = leftF2[:, -1] # The normal vector to the plane is the last singular vector
    centroidF2 = np.mean(filtPoints2, axis=1) # Calculate the centroid of the points
    dF2 = -normalVectorF2.dot(centroidF2)
    orthoDistF2 = np.abs(normalVectorF2[0]*filtPoints2[0] +
                       normalVectorF2[1]*filtPoints2[1] +
                       normalVectorF2[2]*filtPoints2[2] + dF) / np.linalg.norm(normalVectorF2)
    
    CI_95 = scipy.stats.norm.interval(0.95, np.mean(orthoDistF2), np.std(orthoDistF2))

    accuracy = (CI_95[1] - CI_95[0])/2
    print('Accuracy: ', accuracy)
    '''End Filter Plane Outlier'''
    # Calculate corresponding z values on the plane
    # Using the plane equation: ax + by + cz = d
    # Where the normal vector is [a, b, c] and d = normal_vector . centroid
    xxF2, yyF2 = np.meshgrid(np.linspace(np.min(filtPoints2[0]), np.max(filtPoints2[0]), 10),
                        np.linspace(np.min(filtPoints2[1]), np.max(filtPoints2[1]), 10))
    zzF2 = (-normalVectorF2[0] * xxF2 - normalVectorF2[1] * yyF2 - dF2) * (1 / normalVectorF2[2])

    # Calculate angles with x, y, and z axes
    xAngle = np.degrees(np.acos(abs(normalVectorF2[0]) / np.linalg.norm(normalVectorF2)))
    yAngle = np.degrees(np.acos(abs(normalVectorF2[1]) / np.linalg.norm(normalVectorF2)))
    zAngle = np.degrees(np.acos(abs(normalVectorF2[2]) / np.linalg.norm(normalVectorF2)))
    
    # Plot the best-fitting plane
    ax.plot_surface(xxF2, yyF2, zzF2, alpha=0.5, color='yellow', label='Filtered Plane')
    if title != 0:
        ax.set_title(title)
        fig.suptitle(f'x: {90-xAngle:.3f}, y: {90-yAngle:.3f}, z: {zAngle:.3f}')

    planeF2 = [xxF2, yyF2, zzF2]
    angleF2 = [xAngle, yAngle, zAngle]
    plt.figure(1000)
    plt.hist(orthoDistF2, bins=17, density=True)
    plt.title('HubFace Points Spread Distribution - Demo Data')
    plt.xlabel('Orthogonal Distance - Point to Plane (mm)')
    
    return planeF2, angleF2

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
    print('closeLedgePairs',closeBigLedges)
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
    
    print('\nbigLedgeAvgs: ', bigLedgeAvgs)
    print('\nstitchedBigLedgeAvgs: ', stitchedBigLedgeAvgs)

    return stitchedBigLedges, stitchedBigLedgeAvgs
    
def Find_Grid_Ledges(Points, ledgeThreshold, shortLedge, closeLedges):
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

def Plot_Ledges(ledges, ledgeAvgs, hubPlane=0, angle=0, refPlane=0):
    numLedges = len(ledges)
    if hubPlane == 0:
        fig = plt.figure(42)
        ax = fig.add_subplot(111, projection='3d')
        for i in range(numLedges):
            ax.scatter(ledges[i][0], ledges[i][1], ledges[i][2], s=1, label=f'{ledgeAvgs[i]:.2f}')
            ax.set_title(f'numLedges: {numLedges}')
    else:
        fig = plt.figure(43)
        ax = fig.add_subplot(111, projection='3d')
        for i in range(numLedges):
            ax.scatter(ledges[i][0], ledges[i][1], ledges[i][2], s=1, label=f'{ledgeAvgs[i]:.2f}')
            ax.plot_surface(hubPlane[0], hubPlane[1],hubPlane[2], alpha=0.5, label='Filtered Plane')
            if refPlane != 0:
                ax.plot_surface(refPlane[0], refPlane[1],refPlane[2], alpha=0.5, label='Filtered Reference Plane')
            ax.set_title(f'Angles - x:{90-angle[0]:.3f}, y:{90-angle[1]:.3f}, z:{angle[2]:.3f}')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    
    # ax.legend()

def Check_Not_Casting(ledge):
    """
    Determines whether the outer perimeter of the ledge forms a continuous circle or a star-like pattern.

    Parameters:
        ledge (list of numpy arrays): A list containing the x, y, and z coordinates of the ledge points.

    Returns:
        bool: True if the perimeter forms a star-like pattern (with large, consistently spaced gaps),
              False if the perimeter forms a mostly continuous circle.
    """
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

    Plot_All([x,y,z], plotNum=131, title='Filtered Check-Casting Ledge')

    # Approximate the smallest bounding circle's radius as the maximum radius   
    max_radius = np.max(radii)
    min_radius = np.min(radii)

    # Filter points whose radius is less than 90% of the max radius
    valid_indices_outer = radii >= 0.8 * max_radius
    valid_indices_inner = radii <= 1.3 * min_radius
    x_outer, y_outer, z_outer = x[valid_indices_outer], y[valid_indices_outer], z[valid_indices_outer]
    x_inner, y_inner, z_inner = x[valid_indices_inner], y[valid_indices_inner], z[valid_indices_inner]

    Plot_All([x_outer,y_outer,z_outer], plotNum=132, title='outer ring')
    Plot_All([x_inner,y_inner,z_inner], plotNum=133, title='inner ring')

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

def Find_Grid_HubFace(ledges, ledgeAvgs, reverse=False):
    # Count and sort ledges by largest number of points
    ledgeNumPoints = []
    for ledge in ledges:
        ledgeNumPoints.append(len(ledge[0]))
    descendIndices_numPoints = np.argsort(ledgeNumPoints)[::-1]
    print(f'\n\nledgeNumPoints: {ledgeNumPoints}')
    print(f'descendIndices_numPoints: {descendIndices_numPoints}')
    # sort ledges by z-value
    if reverse:
        sortedIndices = np.argsort(ledgeAvgs)[::-1] # Reverses order
    else:
        sortedIndices = np.argsort(ledgeAvgs)

    # delete all ledges beneath the biggest ledge
    numPoints0 = len(ledges[sortedIndices[0]][0])
    numPoints1 = len(ledges[sortedIndices[1]][0])
    biggestLedge_numPoints = len(ledges[descendIndices_numPoints[0]][0])
    while numPoints0/biggestLedge_numPoints <= 0.50:    # while biggest layer has 2x more points than bottom layer
        ledges.pop(sortedIndices[0])                    # bottom layer is likely noise. Delete it
        ledgeAvgs.pop(sortedIndices[0])
        ledgeNumPoints.pop(sortedIndices[0])
        print('Deleted Bottom Ledge')
        if reverse:
            sortedIndices = np.argsort(ledgeAvgs)[::-1] # Reverses order
        else:
            sortedIndices = np.argsort(ledgeAvgs)

        numPoints0 = len(ledges[sortedIndices[0]][0])
        numPoints1 = len(ledges[sortedIndices[1]][0])
        descendIndices_numPoints = np.argsort(ledgeNumPoints)[::-1]
        biggestLedge_numPoints = len(ledges[descendIndices_numPoints[0]][0])

    # Identify if bottom or second to bottom ledge is the hubface
    if numPoints1/numPoints0 >= 0.15:       # if second at least 15% the size of the bottom
        hubFace = ledges[sortedIndices[1]]  # bottom is likely casting. Second is likely hubface
        hubFaceAvg = ledgeAvgs[sortedIndices[1]]
        refPlate = ledges[sortedIndices[0]]
    else:
        if Check_Not_Casting(ledges[sortedIndices[0]]): # if likely botton, make sure it's not the casting
            hubFace = ledges[sortedIndices[0]]          # The bottom is the hubface
            hubFaceAvg = ledgeAvgs[sortedIndices[0]]
            refPlate = None
        else:                                               
            hubFace = ledges[sortedIndices[1]]          # The bottom is the casting
            hubFaceAvg = ledgeAvgs[sortedIndices[1]]
            refPlate = ledges[sortedIndices[0]]

    return hubFace, hubFaceAvg, refPlate

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

def Hub_Only(filename, demo=False):
        if demo:
            myPoints = Load_Demo_Data(filename, maxPoints=10000, cutOff=[0,1000])
        else:
            data = np.loadtxt(filename, skiprows=1)#[::10]   # Take every 10th point. Bottleneck at np.linalg.svd() in Calc_Plane()
            data = data[data[:, 2] >= -100]
            x = data[:,0]
            y = data[:,1]
            z = data[:,2]#/100
            myPoints = np.array([x, y, z])

        myPoints = Filt_nan(myPoints)
        Plot_All(myPoints,plotNum=0, title='Raw Data')
        myPoints = Filt_Raw_Points(myPoints, threshold=5.0, shortSeg=2)

        myLedges, myLedgeAvgs = Find_Grid_Ledges(myPoints, ledgeThreshold=2.0, shortLedge=20, closeLedges=5.5)
        Plot_Ledges(myLedges, myLedgeAvgs)
        myHubFace, myHubFaceAvg, myRefPlate = Find_Grid_HubFace(myLedges, myLedgeAvgs, reverse=False)
        myPlane, myAngle = Calc_Plane(myHubFace, title='Filtered Best Plane', plotNum=20)

        Plot_Ledges(ledges=myLedges, ledgeAvgs=myLedgeAvgs, hubPlane=myPlane, angle=myAngle, refPlane=0)

def Ref_Plane(filename, demo=False):
        if demo:
            myPoints = Load_Demo_Data(filename, maxPoints=10000, cutOff=[0,1000])
            myRefPoints = Load_Demo_Data(filename, maxPoints=10000, cutOff=[-40,0])
        else:
            data = np.loadtxt(filename, skiprows=1)#[::10]   # Take every 10th point. Bottleneck at np.linalg.svd() in Calc_Plane()
            data = data[data[:, 2] >= -100]
            x = data[:,0]
            y = data[:,1]
            z = data[:,2]#/100
            myPoints = np.array([x, y, z])

        Plot_All(myPoints,plotNum=0, title='Raw Data')
        myPoints = Filt_Raw_Points(myPoints, threshold=5.0, shortSeg=2)

        myLedges, myLedgeAvgs = Find_Grid_Ledges(myPoints, ledgeThreshold=2.0, shortLedge=20, closeLedges=5.5)
        Plot_Ledges(myLedges, myLedgeAvgs)
        myHubFace, myHubFaceAvg, myRefPlate = Find_Grid_HubFace(myLedges, myLedgeAvgs, reverse=False)
        myPlane, myAngle = Calc_Plane(myHubFace, title='Filtered Best Plane', plotNum=20)
        myRefPlane, myRefAngle = Calc_Plane(myRefPoints, title='Filtered Best Reference Plane', plotNum=21)

        Plot_Ledges(ledges=myLedges, ledgeAvgs=myLedgeAvgs, hubPlane=myPlane, angle=myAngle, refPlane=myRefPlane)

if __name__ == "__main__":
    start = time.time()
    # Hub_Only(filename='idler_star1.csv', demo=True)
    Hub_Only(filename='simScan_Curve2deg_toe_noise.txt', demo=False)
    end = time.time()

    print('Duration: ', end-start)
    
    plt.show()
