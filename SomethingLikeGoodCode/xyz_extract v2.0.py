import numpy as np
import matplotlib.pyplot as plt
import time
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

def GenData(alpha=0.0):
    # Sample data
    theta = np.linspace(0,np.pi*2, 1000)

    x = np.cos(theta)
    y = np.sin(theta)
    z = np.full(len(theta), np.nan)

    floor = 0.0
    mount = 0.3
    stud = 0.7

    a = 0
    b = 0.2
    c = 0.35
    d = 0.55
    baseOne = 0
    baseTwo = np.pi/2
    baseThree = np.pi
    baseFour = 3*np.pi/2
    for i in range(len(theta)):
        if theta[i] >=a and theta[i] < b:
            z[i] = mount
        elif theta[i] >= b and theta[i] < c:
            z[i] = stud
        elif theta[i] >= c and theta[i] < d:
            z[i] = mount

        elif theta[i] >=baseTwo+a and theta[i] < baseTwo+b:
            z[i] = mount
        elif theta[i] >= baseTwo+b and theta[i] < baseTwo+c:
            z[i] = stud
        elif theta[i] >= baseTwo+c and theta[i] < baseTwo+d:
            z[i] = mount

        elif theta[i] >=baseThree+a and theta[i] < baseThree+b:
            z[i] = mount
        elif theta[i] >= baseThree+b and theta[i] < baseThree+c:
            z[i] = stud
        elif theta[i] >= baseThree+c and theta[i] < baseThree+d:
            z[i] = mount

        elif theta[i] >=baseFour+a and theta[i] < baseFour+b:
            z[i] = mount
        elif theta[i] >= baseFour+b and theta[i] < baseFour+c:
            z[i] = stud
        elif theta[i] >= baseFour+c and theta[i] < baseFour+d:
            z[i] = mount
        
        else:
            z[i] = floor
        z[i] += 0.05*np.sin(theta[i])
        z[i] += np.random.normal(0, alpha)

    return x, y, z, theta

def Extract_HubFace_Points(Points):
    x = Points[0]
    y = Points[1]
    z = Points[2]
    x_avg = np.mean(x) 
    y_avg = np.mean(y)
    z_avg = np.mean(z)
    z_max = z.max()
    z_min = z.min()

    threshold = 15.0  # Define a threshold for detecting significant changes
    shift_indices = detect_shifts(z, threshold)
    segment_averages = calculate_segment_averages(z, shift_indices)

    ledges = np.unique(segment_averages)
    # print(f'ledges: {ledges}')
    x_center = np.full(len(ledges), x_avg)
    y_center = np.full(len(ledges), y_avg)

    threshold = 5.0  # Define a threshold for detecting significant changes
    hubFace = detect_hubFace(ledges, threshold)
    # print(f'hubface: {hubFace}')
    # plt.show()
    x_center2 = np.full(len(hubFace), x_avg)
    y_center2 = np.full(len(hubFace), y_avg)

    threshold = 2.0
    hubFacePoints = np.array([x[(z > np.min(hubFace)-threshold) & (z < np.max(hubFace)+threshold)],
                              y[(z > np.min(hubFace)-threshold) & (z < np.max(hubFace)+threshold)],
                              z[(z > np.min(hubFace)-threshold) & (z < np.max(hubFace)+threshold)]])

    # Create a figure
    fig = plt.figure(42)

    # Add a 3D subplot
    ax = fig.add_subplot(111, projection='3d')

    # Plot the data
    ax.scatter(x, y, z, s=1)
    ax.scatter(x_center, y_center, ledges, color='blue', s=10)
    ax.scatter(x_center2, y_center2, hubFace, color='red', s=50)
    ax.scatter(hubFacePoints[0], hubFacePoints[1], hubFacePoints[2], color='red')

    # Add labels
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    # Subtract out the centroid and take the SVD
    svd = np.linalg.svd(hubFacePoints - np.mean(hubFacePoints, axis=1, keepdims=True))

    # Extract the left singular vectors
    left = svd[0]

    # The normal vector to the plane is the last singular vector
    normal_vector = left[:, -1]

    # Calculate the centroid of the points
    centroid = np.mean(hubFacePoints, axis=1)

    # Generate points on the best-fitting plane
    # Define a grid for the plane
    xx, yy = np.meshgrid(np.linspace(np.min(hubFacePoints[0]), np.max(hubFacePoints[0]), 1000),
                        np.linspace(np.min(hubFacePoints[1]), np.max(hubFacePoints[1]), 1000))

    # Calculate corresponding z values on the plane
    # Using the plane equation: ax + by + cz = d
    # Where the normal vector is [a, b, c] and d = normal_vector . centroid
    d = -normal_vector.dot(centroid)
    zz = (-normal_vector[0] * xx - normal_vector[1] * yy - d) * (1 / normal_vector[2])

    # Plot the best-fitting plane
    ax.plot_surface(xx, yy, zz, alpha=0.5, color='green', label='Best Fit Plane')
    # Plot the normal vector
    print("Normal Vector:", [f"{elem:.5f}" for elem in normal_vector])
    ax.quiver(centroid[0], centroid[1], centroid[2], normal_vector[0], normal_vector[1], normal_vector[2], length=10, color='blue', label='Normal Vector')

    # Calculate angles with x, y, and z axes
    xAngle = np.degrees(np.acos(abs(normal_vector[0]) / np.linalg.norm(normal_vector)))
    yAngle = np.degrees(np.acos(abs(normal_vector[1]) / np.linalg.norm(normal_vector)))
    zAngle = np.degrees(np.acos(abs(normal_vector[2]) / np.linalg.norm(normal_vector)))

    print(f'Degree Angle - x: {90-xAngle:.3f}, y: {90-yAngle:.3f}, z: {zAngle:.3f} degrees')

    return hubFacePoints
    
def Calc_Plane(points, title=0):
    fig = plt.figure(2)
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[0], points[1], points[2], color='red', s=1)
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    
    svd = np.linalg.svd(points - np.mean(points, axis=1, keepdims=True)) # Subtract out the centroid and take the SVD
    print(svd)
    left = svd[0] # Extract the left singular vectors
    normalVector = left[:, -1] # The normal vector to the plane is the last singular vector
    centroid = np.mean(points, axis=1) # Calculate the centroid of the points
    d = -normalVector.dot(centroid)

    # Calculate corresponding z values on the plane
    # Using the plane equation: ax + by + cz = d
    # Where the normal vector is [a, b, c] and d = normal_vector . centroid
    xx, yy = np.meshgrid(np.linspace(np.min(points[0]), np.max(points[0]), 100),
                        np.linspace(np.min(points[1]), np.max(points[1]), 100))
    zz = (-normalVector[0] * xx - normalVector[1] * yy - d) * (1 / normalVector[2])
    
    # Plot the best-fitting plane
    # ax.plot_surface(xx, yy, zz, alpha=0.5, color='blue', label='Best Fit Plane')
    # Plot the normal vector
    # ax.quiver(centroid[0], centroid[1], centroid[2], normalVector[0], normalVector[1], normalVector[2], length=10, color='blue', label='Normal Vector')

    '''Start Filter Plane Outliers'''
    orthoDist = np.abs(normalVector[0]*points[0] +
                       normalVector[1]*points[1] +
                       normalVector[2]*points[2] + d) / np.linalg.norm(normalVector)
    Q1 = np.percentile(orthoDist, 25)
    Q3 = np.percentile(orthoDist, 75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.0 * IQR
    upper_bound = Q3 + 1.0 * IQR
    
    filtPoints = points[:,(orthoDist >= lower_bound) & (orthoDist <= upper_bound)]
    ax.scatter(filtPoints[0], filtPoints[1], filtPoints[2], color='green', s=10)
    
    svdF = np.linalg.svd(filtPoints - np.mean(filtPoints, axis=1, keepdims=True)) # Subtract out the centroid and take the SVD
    leftF = svdF[0] # Extract the left singular vectors
    normalVectorF = leftF[:, -1] # The normal vector to the plane is the last singular vector
    centroidF = np.mean(filtPoints, axis=1) # Calculate the centroid of the points
    dF = -normalVectorF.dot(centroidF)

    # Calculate corresponding z values on the plane
    # Using the plane equation: ax + by + cz = d
    # Where the normal vector is [a, b, c] and d = normal_vector . centroid
    xxF, yyF = np.meshgrid(np.linspace(np.min(filtPoints[0]), np.max(filtPoints[0]), 10),
                        np.linspace(np.min(filtPoints[1]), np.max(filtPoints[1]), 10))
    zzF = (-normalVectorF[0] * xxF - normalVectorF[1] * yyF - dF) * (1 / normalVectorF[2])
    
    # Plot the best-fitting plane
    ax.plot_surface(xxF, yyF, zzF, alpha=0.5, color='yellow', label='Filtered Plane')
    # Plot the normal vector
    ax.quiver(centroidF[0], centroidF[1], centroidF[2], normalVectorF[0], normalVectorF[1], normalVectorF[2], length=10, color='yellow', label='Filtered Normal Vector')
    if title != 0:
        ax.set_title(title)
    '''End Filter Plane Outlier'''
    # Calculate angles with x, y, and z axes
    xAngle = np.degrees(np.acos(abs(normalVectorF[0]) / np.linalg.norm(normalVectorF)))
    yAngle = np.degrees(np.acos(abs(normalVectorF[1]) / np.linalg.norm(normalVectorF)))
    zAngle = np.degrees(np.acos(abs(normalVectorF[2]) / np.linalg.norm(normalVectorF)))

    print(normalVectorF)
    print(f'x: {90-xAngle:.3f}, y: {90-yAngle:.3f}, z: {zAngle:.3f}')

    planeF = [xxF, yyF, zzF]
    angleF = [xAngle, yAngle, zAngle]

    return planeF, angleF

def Plot_All(Points, plotNum, title=0):
    fig = plt.figure(plotNum)
    ax = fig.add_subplot(111, projection='3d')

    # Plot the data
    ax.scatter(Points[0], Points[1], Points[2], s=1)
    # Add labels
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    if title != 0:
        ax.set_title(title)
    # plt.show()

def Filt_nan(Points):
    # print('len(Points): ', len(Points[2]))
    newPoints = Points[:, ~np.isnan(Points[2])]
    # print('len(newPoints): ', len(newPoints[2]))

    return newPoints

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

    closeBigLedges = []
    needStitch = False
    for i in range(len(bigLedges)):
        for j in range(i+1, len(bigLedges)):
            if abs(bigLedgeAvgs[i] - bigLedgeAvgs[j]) <= closeLedges:
                closeBigLedges.append((i,j))
                needStitch = True
    

    if needStitch:
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
    
    else:
        return bigLedges, bigLedgeAvgs

def Plot_Ledges(ledges, ledgeAvgs, hubPlane=0, angle=0):
    numLedges = len(myLedges)
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
            ax.set_title(f'Angles - x:{90-angle[0]:.3f}, y:{90-angle[1]:.3f}, z:{angle[2]:.3f}')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    
    # ax.legend()

def Find_Grid_HubFace(ledges, ledgeAvgs):
    # print(ledgeAvgs)
    sortedIndices = np.argsort(ledgeAvgs)[::-1] # Reverses order
    # print(sortedIndices)

    numPoints0 = len(ledges[sortedIndices[0]][0])
    numPoints1 = len(ledges[sortedIndices[1]][0])
    # print(numPoints0)
    # print(numPoints1)

    if numPoints1/numPoints0 >= 0.1:
        # print('here1')
        hubFace = ledges[sortedIndices[1]]
    else:
        # print('here0')
        hubFace = ledges[sortedIndices[0]]
    
    return hubFace

def Filt_HubFace(hubFace):
    Q1 = np.percentile(hubFace[2], 25)
    Q3 = np.percentile(hubFace[2], 75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    # Filter out values outside the bounds
    filtHubFace = hubFace[:,(hubFace[2] >= lower_bound) & (hubFace[2] <= upper_bound)]
    return filtHubFace

if __name__ == "__main__":
    start = time.time()
    data = np.loadtxt('measuredData_GRID100_BRAKE_SMALL_TILT.txt', skiprows=1) #GenData(0.005)
    
    x = data[:,0]
    y = data[:,1]
    z = data[:,2]/100
    myPoints = np.array([x, y, z])
    
    myPoints = Filt_nan(myPoints)
    Plot_All(myPoints,plotNum=0, title='Raw Data')
    myPoints = Filt_Raw_Points(myPoints, threshold=5.0, shortSeg=3)
    
    myLedges, myLedgeAvgs = Find_Grid_Ledges(myPoints, ledgeThreshold=2.0, shortLedge=15, closeLedges=3.0)

    end = time.time()
    Plot_Ledges(myLedges, myLedgeAvgs)
    myHubFace = Find_Grid_HubFace(myLedges, myLedgeAvgs)
    Plot_All(myHubFace, plotNum=2)
    # myFiltHubFace = Filt_HubFace(myHubFace)
    # Plot_All(myFiltHubFace, plotNum=3)
    myPlane, myAngle = Calc_Plane(myHubFace, title='Filtered Best Plane')

    Plot_Ledges(ledges=myLedges, ledgeAvgs=myLedgeAvgs, hubPlane=myPlane, angle=myAngle)


    # print(end-start)
    
    # Extract_HubFace_Points(myPoints)
    # Calc_Plane([x,y,z])
    
    # np.savetxt('measuredData.txt', hubFacePoints.T, fmt='%.6f', delimiter=' ', header='X Y Z', comments='')
    
    
    plt.show()
