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

def align_signals(signal1, signal2):
    """Align two signals using cross-correlation."""
    correlation = signal.correlate(signal1, signal2, mode='full')
    lags = signal.correlation_lags(len(signal1), len(signal2), mode='full')
    lag = lags[np.argmax(correlation)]
    return lag

def process_and_plot(imu_data, marker_data, column_name):
    """Process and plot comparison for specified column."""
    time_column = imu_data.columns[0]
    
    # Extract data
    imu_time = imu_data[time_column]
    imu_data_col = imu_data[column_name]
    marker_time = marker_data[time_column]
    marker_data_col = marker_data[column_name]

    # Normalize time
    imu_time_normalized = imu_time - imu_time.min()
    marker_time_normalized = marker_time - marker_time.min()

    # Create interpolation functions
    f_imu = interpolate.interp1d(imu_time_normalized, imu_data_col, 
                                bounds_error=False, fill_value="extrapolate")
    f_marker = interpolate.interp1d(marker_time_normalized, marker_data_col, 
                                   bounds_error=False, fill_value="extrapolate")

    # Create common time grid
    common_duration = min(imu_time_normalized.max(), marker_time_normalized.max())
    common_time = np.linspace(0, common_duration, 1000)

    # Interpolate signals
    imu_interp = f_imu(common_time)
    marker_interp = f_marker(common_time)

    # Align signals
    time_shift = align_signals(imu_interp, marker_interp) * (common_time[1] - common_time[0])
    aligned_common_time = common_time + time_shift
    marker_aligned = f_marker(aligned_common_time)

    # Plot results
    fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(15, 10))
    
    # Original data
    ax1.plot(imu_time_normalized, imu_data_col, 'b-', label='IMU')
    ax1.plot(marker_time_normalized, marker_data_col, 'r-', label='Marker')
    ax1.set_title(f'Original {column_name} Data')
    ax1.set_ylabel(column_name)
    ax1.legend()
    ax1.grid(True)

    # Interpolated data
    ax2.plot(common_time, imu_interp, 'b-', label='IMU')
    ax2.plot(common_time, marker_interp, 'r-', label='Marker')
    ax2.set_title(f'Interpolated {column_name} Data')
    ax2.set_ylabel(column_name)
    ax2.legend()
    ax2.grid(True)

    # Aligned data
    ax3.plot(common_time, imu_interp, 'b-', label='IMU')
    ax3.plot(common_time, marker_aligned, 'r-', label='Marker (Aligned)')
    ax3.set_title(f'Aligned {column_name} Data')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel(column_name)
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout()
    plt.savefig(f'{column_name}_comparison.png', dpi=300)
    plt.show()

    # Calculate and plot error metrics
    valid_indices = ~np.isnan(marker_aligned) & ~np.isnan(imu_interp)
    error = imu_interp[valid_indices] - marker_aligned[valid_indices]
    rmse = np.sqrt(np.mean(error**2))
    mean_abs_error = np.mean(np.abs(error))

    print(f"\nMetrics for {column_name}:")
    print(f"RMSE: {rmse:.4f}")
    # print(f"Correlation: {correlation:.4f}")
    print(f"Mean absolute error: {mean_abs_error:.4f}")

    # Error plot
    fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6))
    
    ax1.plot(common_time, imu_interp, 'b-', label='IMU')
    ax1.plot(common_time, marker_aligned, 'r-', label='Marker')
    ax1.set_title(f'{column_name} Data', fontsize=24)
    ax1.set_ylabel('Angle (degrees)', fontsize=20)
    ax1.legend()
    ax1.grid(True)

    ax2.plot(common_time, imu_interp - marker_aligned, 'g-')
    ax2.set_title(f'Difference in {column_name}', fontsize=24)
    ax2.set_xlabel('Time (s)', fontsize=20)
    ax2.set_ylabel('Angle (degrees)', fontsize=20)
    ax2.axhline(y=0, color='k', linestyle='--')
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig(f'{column_name}_error.png', dpi=300)
    plt.show()

if __name__ == "__main__":
    # File paths
    imu_file_path = "IMU_euler_angles.sto"
    marker_file_path = "Marker_euler_angles.sto"

    # Read data
    imu_data = read_sto_file(imu_file_path)
    marker_data = read_sto_file(marker_file_path)

    # Print available columns
    print("Available columns:", imu_data.columns.tolist())

    # Specify column to analyze
    column_name = 'scapula_pitch'  # Change this to analyze different measures
    
    # Process and plot
    process_and_plot(imu_data, marker_data, column_name)