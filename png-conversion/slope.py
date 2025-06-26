import numpy as np
import matplotlib.pyplot as plt
import sys

def plot_best_fit(x, y):
    x = np.array(x)
    y = np.array(y)
    
    # Create a mask to filter out zero values
    mask = (x != 0) & (y != 0)
    
    # Apply the mask to filter out the zero values
    x_filtered = x[mask]
    y_filtered = y[mask]
    
    # Compute the best-fit line using linear regression on filtered data
    m, b = np.polyfit(x_filtered, y_filtered, 1)
    best_fit_line = m * x_filtered + b
    
    # Plot the points and the best-fit line
    plt.scatter(x_filtered, y_filtered, color='blue', label='Data Points')
    plt.plot(x_filtered, best_fit_line, color='red', label=f'Best-Fit Line: y={m:.2f}x+{b:.2f}')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Best-Fit Line for Given Points (Excluding Zero Values)')
    print(f"Best Fit Line: y = {m}*x + {b}")
    #plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python slope.py <file1.csv> <file2.csv>")
    else:
        # Load data from a CSV file into a NumPy array
        # Assuming the CSV has a column 'X' for x values and 'Y' for y values
        csv_file_path = sys.argv[1]  # Replace with the path to your CSV file
        csv_data = np.genfromtxt(csv_file_path, delimiter=',', invalid_raise=False, filling_values=0)
        y = csv_data.flatten()
        
        csv_file_path = sys.argv[2]  # Replace with the path to your CSV file
        csv_data = np.genfromtxt(csv_file_path, delimiter=',', invalid_raise=False, filling_values=0)
        x = csv_data.flatten()
        
        plot_best_fit(x, y)
        
        #slope = y = 0.0023999863818105023*x + -39.321368083269455
