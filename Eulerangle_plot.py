import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from math import acos, atan2, asin, pi, degrees

def read_sto_file(file_path):
    """Read .sto file and return header lines and data"""
    with open(file_path, 'r') as f:
        file_content = f.read()
    
    lines = file_content.strip().split('\n')
    
    # Extract header information
    header_lines = []
    data_start_idx = 0
    for i, line in enumerate(lines):
        header_lines.append(line)
        if line.startswith('endheader'):
            data_start_idx = i + 1
            break
    
    # Get column headers
    column_names = lines[data_start_idx].split()
    
    # Initialize data
    data = {
        'time': []
    }
    
    # Initialize lists for each quaternion column
    for col in column_names[1:]:  # Skip time column
        data[col] = []
    
    # Parse data rows
    for i in range(data_start_idx + 1, len(lines)):
        values = lines[i].split()
        if not values or len(values) < len(column_names):  # Skip incomplete lines
            continue
            
        # Extract time
        data['time'].append(float(values[0]))
        
        # Extract quaternion strings and convert to components
        for j, col in enumerate(column_names[1:], 1):
            quat_str = values[j]
            try:
                # Split the quaternion string by commas and convert to float
                quat_components = [float(x) for x in quat_str.split(',')]
                data[col].append(quat_components)
            except (ValueError, IndexError):
                print(f"Error parsing quaternion at line {i+1}, column {j}")
                data[col].append([1.0, 0.0, 0.0, 0.0])  # Default to identity quaternion
    
    return header_lines, data, column_names[1:]  # Return quaternion column names

def quaternion_dot_product(q1, q2):
    """Calculate the dot product of two quaternions.
    
    Args:
        q1, q2: Quaternions in [w, x, y, z] format
    
    Returns:
        Dot product (scalar)
    """
    return q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]

def quaternion_angle_difference(q1, q2):
    """Calculate the angle difference between two quaternions using the method in the screenshot.
    
    Args:
        q1, q2: Quaternions in [w, x, y, z] format
    
    Returns:
        Angle in degrees
    """
    # Calculate dot product and take its absolute value
    dot_product = abs(quaternion_dot_product(q1, q2))
    
    # Ensure dot product is in valid range for arccos [-1, 1]
    dot_product = np.clip(dot_product, -1.0, 1.0)
    
    # Calculate angle using the formula θ = 2 * arccos(|q1 · q2|)
    angle = 2 * acos(dot_product) * 180 / np.pi
    
    return angle

def quaternion_to_euler(w, x, y, z):
    """Convert quaternion to Euler angles (in degrees)
    
    Returns:
        tuple: (roll, pitch, yaw) representing rotations around X, Y, Z axes
    """
    # Roll (rotation around X axis)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (rotation around Y axis)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = pi/2 if sinp > 0 else -pi/2  # Use 90° if out of range
    else:
        pitch = asin(sinp)
    
    # Yaw (rotation around Z axis)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    
    # Convert from radians to degrees
    return (degrees(roll), degrees(pitch), degrees(yaw))

# Uncomment the following function if you want to visulalize the quaternion angles as Euler angles.
def plot_quaternion_angles(data, quat_columns):
    """Plot rotation angles from parsed quaternion data"""
    # Create figure with 4 subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 16), sharex=True)
        
    color_map = {
        'scapula_imu': 'blue',
        'humerus_imu': 'orange',
        'thorax_imu': 'green'
    }

    # For each quaternion column, calculate and plot angle differences from first frame
    for col in quat_columns:
        if len(data[col]) == 0:
            continue
            
        # Get reference quaternion (first frame)
        ref_quaternion = data[col][0]
        
        # Calculate angle difference for each frame compared to first frame
        angle_differences = []
        
        # For Euler angles
        roll_angles = []
        pitch_angles = []
        yaw_angles = []
        
        for quat in data[col]:
            if len(quat) == 4:  # Ensure we have all 4 components
                # Calculate angle difference from first frame
                angle_diff = quaternion_angle_difference(ref_quaternion, quat)
                angle_differences.append(angle_diff)
                
                # Calculate Euler angles
                w, x, y, z = quat
                roll, pitch, yaw = quaternion_to_euler(w, x, y, z)
                roll_angles.append(roll)
                pitch_angles.append(pitch)
                yaw_angles.append(yaw)
        
        if angle_differences:  # Only plot if we have angles
            time_values = data['time'][:len(angle_differences)]
             
            # Plot the three Euler angles in separate subplots
            axes[0].plot(time_values, roll_angles, label=f'{col}', color=color_map[col])
            axes[1].plot(time_values, pitch_angles, label=f'{col}', color=color_map[col])
            axes[2].plot(time_values, yaw_angles, label=f'{col}', color=color_map[col])
    
            # # Plot angle differences from first frame
            # axes[3].plot(time_values, angle_differences, label=f'{col}')

    axes[0].set_title('Roll (X-Axis)', fontsize=20)
    axes[0].set_ylabel('Angle (degrees)', fontsize=20)
    axes[0].set_ylim([-110, 20]) 
    axes[0].grid(True)
    axes[0].legend()
    
    axes[1].set_title('Pitch (Y-Axis)', fontsize=20)
    axes[1].set_ylabel('Angle (degrees)', fontsize=20)
    axes[1].set_ylim([-50, 40]) 
    axes[1].grid(True)
    axes[1].legend()
    
    axes[2].set_title('Yaw (Z-Axis)', fontsize=20)
    axes[2].set_xlabel('Time (s)', fontsize=20, labelpad=10)
    axes[2].set_ylabel('Angle (degrees)', fontsize=20)
    axes[2].set_ylim([-40, 40]) 
    axes[2].grid(True)
    axes[2].legend()

    # Add titles and labels
    # axes[3].set_title('Quaternion Angle Difference from First Frame')
    # axes[3].set_ylabel('Angle (degrees)')
    # axes[3].grid(True)
    # axes[3].legend()
    
    fig.suptitle('Marker Euler Angles ("XYZ")', fontsize=24, y=0.95, x=0.5, ha='center')
    plt.tight_layout(rect=[0, 0.02, 1, 0.95]) 
    return fig, axes

def process_sto_file(file_path):
    """Process .sto file and plot quaternion data"""
    header_lines, data, quat_columns = read_sto_file(file_path)
    
    if not quat_columns:
        print("No quaternion columns found in the file")
        return None, None
    
    fig, axes = plot_quaternion_angles(data, quat_columns)
    plt.show()
    return fig, axes

if __name__ == "__main__":
    file_path = r"C:\EPFL\Semester Project\MARKER EXPERIMENT\exp2_r\Y90_mid_vis_inv.sto" # This should be the path to your .sto file
    process_sto_file(file_path)



# #   Euler angles conversion and saving to .sto file
# def save_euler_angles_to_sto(imu_data, output_file="euler_angles.sto"):
#     """Save Euler angles (roll, pitch, yaw) to a .sto file"""
#     segments = ['scapula_imu', 'humerus_imu', 'thorax_imu']
#     euler_data = {
#         'time': imu_data['time'],
#         'scapula_roll': [],
#         'scapula_pitch': [],
#         'scapula_yaw': [],
#         'humerus_roll': [],
#         'humerus_pitch': [],
#         'humerus_yaw': [],
#         'thorax_roll': [],
#         'thorax_pitch': [],
#         'thorax_yaw': []
#     }
    
#     # Calculate Euler angles for each segment
#     for segment in segments:
#         for quat in imu_data[segment]:
#             w, x, y, z = quat
#             roll, pitch, yaw = quaternion_to_euler(w, x, y, z)
            
#             # Store angles in the corresponding columns
#             segment_name = segment.split('_')[0]  # Remove '_imu' suffix
#             euler_data[f'{segment_name}_roll'].append(roll)
#             euler_data[f'{segment_name}_pitch'].append(pitch)
#             euler_data[f'{segment_name}_yaw'].append(yaw)
    
#     # Write .sto file
#     with open(output_file, 'w') as f:
#         # Write header
#         f.write("DataRate=100.000000\n")
#         f.write("DataType=EulerAngles\n")
#         f.write("version=3\n")
#         f.write("OpenSimVersion=4.1\n")
#         f.write("endheader\n")
        
#         # Write column headers
#         f.write("time\tscapula_roll\tscapula_pitch\tscapula_yaw\t")
#         f.write("humerus_roll\thumerus_pitch\thumerus_yaw\t")
#         f.write("thorax_roll\tthorax_pitch\tthorax_yaw\n")
        
#         # Write data
#         for i in range(len(euler_data['time'])):
#             line = f"{euler_data['time'][i]}\t"
#             line += f"{euler_data['scapula_roll'][i]}\t{euler_data['scapula_pitch'][i]}\t{euler_data['scapula_yaw'][i]}\t"
#             line += f"{euler_data['humerus_roll'][i]}\t{euler_data['humerus_pitch'][i]}\t{euler_data['humerus_yaw'][i]}\t"
#             line += f"{euler_data['thorax_roll'][i]}\t{euler_data['thorax_pitch'][i]}\t{euler_data['thorax_yaw'][i]}\n"
#             f.write(line)
    
#     print(f"Saved Euler angles to {output_file}")

# if __name__ == "__main__":
#     # File paths
#     imu_file = "Marker_data.sto"
#     output_file = "Marker_euler_angles.sto"
    
#     # Read IMU data
#     header_lines, imu_data, quat_columns = read_sto_file(imu_file)
    
#     # Save Euler angles
#     save_euler_angles_to_sto(imu_data, output_file)




#   # Uncomment the following function if you want to save quaternion angle differences to a .sto file.
# def save_angle_differences_to_sto(imu_data, output_file="angle_differences.sto"):
#     """Save quaternion angle differences to a .sto file"""
#     segments = ['scapula_imu', 'humerus_imu', 'thorax_imu']
#     angles_data = {
#         'time': imu_data['time'],
#         'scapula': [],
#         'humerus': [],
#         'thorax': []
#     }
    
#     # Calculate angles for each segment
#     for segment in segments:
#         ref_quaternion = imu_data[segment][0]  # Reference quaternion (first frame)
#         angles = []
        
#         for quat in imu_data[segment]:
#             angle = quaternion_angle_difference(ref_quaternion, quat)
#             angles.append(angle)
            
#         # Store angles in the corresponding column
#         segment_name = segment.split('_')[0]  # Remove '_imu' suffix
#         angles_data[segment_name] = angles
    
#     # Write .sto file
#     with open(output_file, 'w') as f:
#         # Write header
#         f.write("DataRate=100.000000\n")
#         f.write("DataType=Angle\n")
#         f.write("version=3\n")
#         f.write("OpenSimVersion=4.1\n")
#         f.write("endheader\n")
        
#         # Write column headers
#         f.write("time\tscapula\thumerus\tthorax\n")
        
#         # Write data
#         for i in range(len(angles_data['time'])):
#             line = f"{angles_data['time'][i]}\t"
#             line += f"{angles_data['scapula'][i]}\t"
#             line += f"{angles_data['humerus'][i]}\t"
#             line += f"{angles_data['thorax'][i]}\n"
#             f.write(line)
    
#     print(f"Saved angle differences to {output_file}")

# if __name__ == "__main__":
#     # File paths
#     imu_file = "IMU_data.sto"
#     output_file = "IMU_angles.sto"
    
#     # Read IMU data
#     header_lines, imu_data, quat_columns = read_sto_file(imu_file)
    
#     # Save angle differences
#     save_angle_differences_to_sto(imu_data, output_file)
