import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

def square_process(input):
    layers = square_categorize(input)
    for i, layer in enumerate(layers):
        count = np.count_nonzero(~np.isnan(input))
        # print(count)
        # print(i)
        # print(layer)
        # np.savetxt(f"layer_{i}", layer, fmt='%.6f', delimiter='\t')


    for layer_index, layer in enumerate(layers):
        # Create a new figure for each layer
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # print(layer.ndim)
        # print(layer)
        ax.scatter(layer[:,0], layer[:,1], layer[:,2], s=1)
        
        # # Create a meshgrid for x and y coordinates
        # x, y = np.meshgrid(range(layer.shape[1]), range(layer.shape[0]))
        
        # # Plot the layer
        # ax.plot_surface(x, y, layer, cmap='viridis', edgecolor='none')
        
        ax.set_title(f'Layer {layer_index + 1}')
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Height (Z axis)')
        
        # Save the figure as a PNG file, or show the figure
        if False:
            plt.savefig(f'layer_{layer_index + 1}.png')
            plt.close(fig)  # Close the figure to free up memory
        else:
            # plt.show()
            pass

        # print(f"Saved layer_{layer_index + 1}.png")

def square_categorize(input):
    rows, cols = input.shape
    # print(f"rows {rows} cols {cols}")
    num_layers = 0
    threshold = 500 # IDK what this is supposed to be
    empty_val = np.nan # this is what the empty spaces in each layer will be filled with
    layers = [] # python list that has the numpy array for each layer stored in it
    layer_averages = [] # python list that keeps the average heights for each layer, so you don't have to parse the whole array each time you want to refactor the average
    num_layers = 0

    for i in range(rows):
        # for j in range(cols):
            # for every point in our array:
                # if there are any layers at all
                    # if it fits into an existing layer
                        # add the point to that
                        # refactor the average for that layer
                # if we haven't added a point after parsing all existing layers, create a new layer

            zVal = input[i][2] # height value of the point in question
            xVal = input[i][0]
            yVal = input[i][1]

            added_something = False

            if np.isnan(zVal):
                continue

            if len(layers): # if there are already identified layers
                for k in range(len(layers)):
                    ave = layer_averages[k]
                    # print(f"ave: {ave}\n  z: {zVal}")
                    if zVal < ave + threshold and zVal > ave - threshold: # if zVal is in the threshold for a layer
                        # adding the point to the correct layer, preserving x-y location
                        layers[k][i][0] = xVal 
                        layers[k][i][1] = yVal
                        layers[k][i][2] = zVal
                        layer_averages[k] = ave - (ave - zVal) / (layers[k].size+1) # refactoring average value including weight of new point
                        added_something = True
                        break

            if not added_something: # we make a new layer for this point
                num_layers += 1
                layers.append(np.empty((rows, cols))) # add a new layer with same dimensions as input array to the end of the layers list
                # insert the new value
                layers[-1][i][0] = xVal
                layers[-1][i][1] = yVal
                layers[-1][i][2] = zVal 
                layer_averages.append(zVal) # average value will just be itself since size=1
    # print("layer_averages: ")
    # print(layer_averages)        
    # print(f"num layers: {num_layers}")
    return layers

start = time.time()
with open('measuredData_GRID100.txt', 'r') as file:
    lines = file.readlines()[1:]  # Skip the header line

# Parse the data
data = []
for line in lines:
    values = line.split()
    x, y = map(float, values[:2])
    z = float(values[2]) if values[2] != 'nan' else np.nan
    data.append([x, y, z])

# Convert to NumPy array
data_array = np.array(data)

square_process(data_array)

end = time.time()
print(end - start)