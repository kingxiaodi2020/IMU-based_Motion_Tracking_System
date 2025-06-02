import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate, signal
import os

def read_sto_file(file_path):
    """Read an OpenSim .sto file and return a pandas DataFrame."""
    with open(file_path, 'r') as f:
        lines = f.readlines()
        header_end = 0
        for i, line in enumerate(lines):
            if 'endheader' in line.lower():
                header_end = i
                break
        
        column_names = lines[header_end + 1].strip().split('\t')
        data_lines = [line.strip().split('\t') for line in lines[header_end + 2:]]
        
        data = {col: [] for col in column_names}
        for line in data_lines:
            if len(line) == len(column_names):
                for i, col in enumerate(column_names):
                    try:
                        data[col].append(float(line[i]))
                    except ValueError:
                        print(f"Warning: Could not convert {line[i]} to float")
                        data[col].append(float('nan'))
        
        return pd.DataFrame(data)

# plot the data against the first column
def plot_data(df, file_name):
    """Plot the data in the DataFrame and show the figure."""
    
    # Shift the time column to start with 0
    df[df.columns[0]] = df[df.columns[0]] - df[df.columns[0]].min()
    
    plt.figure(figsize=(10, 6))
    for column in df.columns[1:]:
        plt.plot(df[df.columns[0]], df[column], label=column)
    
    plt.xlabel('Time (s)', fontsize=20)
    plt.ylim([-40,80])
    plt.ylabel('Angle (degrees)',fontsize=20)
    plt.title('IMU-Based Motion Reconstruction in OpenSim',fontsize=24)
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

# example usage
if __name__ == "__main__":
    # Define the path to the .sto file
    sto_file_path = r"IMU_opensim.sto"
    
    # Read the .sto file
    data = read_sto_file(sto_file_path)
    
    # Plot the data
    plot_data(data, os.path.basename(sto_file_path))


