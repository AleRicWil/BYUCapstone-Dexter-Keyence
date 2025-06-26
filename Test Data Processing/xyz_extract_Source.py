import numpy as np
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

def detect_shifts(z, threshold):
    # Calculate the difference between consecutive elements
    diff = np.diff(z)
    
    # Find indices where the absolute difference is greater than the threshold
    shift_indices = np.where(np.abs(diff) > threshold)[0]
    
    # Return the indices (adjusted to reflect the original vector's indices)
    return shift_indices + 1  # Add 1 because diff reduces the length by 1

def detect_hubFace(ledges, threshold):
    diff = np.diff(ledges)   
    shift_indices = np.where(np.abs(diff) > threshold)[0] + 1  # +1 to get the index of the larger value

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

def Extract_HubFace_Points(x, y, z):
    x_avg = np.mean(x) 
    y_avg = np.mean(y)
    z_avg = np.mean(z)
    z_max = z.max()
    z_min = z.min()

    threshold = 1500  # Define a threshold for detecting significant changes
    shift_indices = detect_shifts(z, threshold)
    segment_averages = calculate_segment_averages(z, shift_indices)

    ledges = np.unique(segment_averages)
    # print(f'ledges: {ledges}')
    x_center = np.full(len(ledges), x_avg)
    y_center = np.full(len(ledges), y_avg)

    threshold = 500  # Define a threshold for detecting significant changes
    hubFace = detect_hubFace(ledges, threshold)
    # print(f'hubface: {hubFace}')
    x_center2 = np.full(len(hubFace), x_avg)
    y_center2 = np.full(len(hubFace), y_avg)

    threshold = 200
    hubFacePoints = np.array([x[(z > np.min(hubFace)-threshold) & (z < np.max(hubFace)+threshold)],
                            y[(z > np.min(hubFace)-threshold) & (z < np.max(hubFace)+threshold)],
                            z[(z > np.min(hubFace)-threshold) & (z < np.max(hubFace)+threshold)]])

    # Create a figure
    fig = plt.figure()

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

    return hubFacePoints
    

if __name__ == "__main__":
    data = np.loadtxt('measuredData.txt', skiprows=1) #GenData(0.005)
    print(data)
    x = data[:,0]

    y = data[:,1]
    z = data[:,2]
    hubFacePoints = Extract_HubFace_Points(x, y, z)
    
    # np.savetxt('measuredData.txt', hubFacePoints.T, fmt='%.6f', delimiter=' ', header='X Y Z', comments='')
    
    
    plt.show()
