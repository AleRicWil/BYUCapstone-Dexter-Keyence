import cv2
import numpy as np
import matplotlib.pyplot as plt
import sys

def png_convert(input_file, output_file):
    # Load the image as grayscale (ensure it reads 16-bit if needed)
    image_path = input_file
    heightmap = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    
    # Check if the image is loaded properly
    if heightmap is None:
        raise ValueError("Failed to load image. Check file path.")
    
    # Print image details
    print(f"Image shape: {heightmap.shape}")
    print(f"Data type: {heightmap.dtype}")
    
    # Normalize the data for visualization (if needed)
    normalized = cv2.normalize(heightmap, None, 0, 65535, cv2.NORM_MINMAX)
    normalized = np.uint8(normalized)
    
    # Display the heightmap
    plt.imshow(normalized, cmap='gray')
    plt.colorbar()
    
    #heightmap = 0.0023999863818105023*heightmap + -39.321368083269455
    heightmap = 0.0024*heightmap - 39.32136785540743 
    heightmap[heightmap==-39.32136785540743] = -99999.9999
    
    # If you need raw height values, they are stored in 'heightmap' as a NumPy array.
    np.savetxt(output_file, heightmap, delimiter=",", fmt="%.4f")  # 6 decimal places

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python png_covert.py <input_file.png> <output-file.csv>")
    else:
        png_convert(sys.argv[1], sys.argv[2])
