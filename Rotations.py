import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

def read_sto(file_path):
    """Read .sto file with metadata header and tab-delimited data"""
    with open(file_path, 'r') as f:
        # Read header until endheader
        header = []
        while True:
            line = f.readline().strip()
            header.append(line)
            if line.startswith("endheader"):
                break
        
        # Read data with tab delimiter
        data = pd.read_csv(f, delimiter='\t')
    return header, data

def write_sto(file_path, header, data):
    """Write .sto file maintaining original format"""
    with open(file_path, 'w') as f:
        # Write header
        for line in header:
            f.write(line + '\n')
        
        # Write column names and data with tab delimiter
        data.to_csv(f, sep='\t', index=False)

def rotate_quaternion(quat_str, axis='x', angle=0):
    """
    Rotate quaternion by specified angle around given axis
    Input: comma-separated string "w,x,y,z"
    Output: comma-separated string "w,x,y,z"
    """
    # Parse quaternion string to array
    w, x, y, z = map(float, quat_str.split(','))
    
    # Convert to scipy rotation
    rot = R.from_quat([x, y, z, w])
    
    # Create additional rotation
    rotation_vec = np.zeros(3)
    axis_idx = {'x': 0, 'y': 1, 'z': 2}
    rotation_vec[axis_idx[axis.lower()]] = np.radians(angle)
    additional_rot = R.from_rotvec(rotation_vec)
    
    # Combine rotations
    new_rot = additional_rot * rot
    
    # Convert back to w,x,y,z string format
    result = new_rot.as_quat()
    return f"{result[3]},{result[0]},{result[1]},{result[2]}"

def manipulate_quaternions(input_file, output_file, rotations):
    """
    Apply rotations to specified IMU columns
    rotations: dict mapping column names to list of (axis, angle) tuples
    Example: {'scapula_imu': [('y', 90)]}
    """
    header, data = read_sto(input_file)
    
    # Apply rotations to specified columns
    for column, rot_list in rotations.items():
        if column in data.columns:
            # Apply each rotation in sequence
            for axis, angle in rot_list:
                data[column] = data[column].apply(
                    lambda q: rotate_quaternion(q, axis, angle)
                )
    
    write_sto(output_file, header, data)
    print(f"Modified data saved to {output_file}")


def get_reset_rotation(quat_str):
    """
    Get reset rotation from initial quaternion
    Input: quaternion string "w,x,y,z"
    Output: scipy Rotation object for reset
    """
    # Parse initial quaternion
    w, x, y, z = map(float, quat_str.split(','))
    
    # Convert to rotation object
    rot = R.from_quat([x, y, z, w])
    
    # Get inverse rotation (will reset to identity)
    return rot.inv()

def reset_quaternions(input_file, output_file, column_to_reset):
    """Reset quaternions based on first quaternion orientation"""
    header, data = read_sto(input_file)
    
    # Get first quaternion and calculate reset rotation
    first_quat = data.iloc[0][column_to_reset]
    reset_rot = get_reset_rotation(first_quat)
    
    # Apply reset rotation to all quaternions in the column
    data[column_to_reset] = data[column_to_reset].apply(
        lambda q_str: apply_reset_rotation(q_str, reset_rot)
    )
    
    write_sto(output_file, header, data)
    print(f"Reset data saved to {output_file}")

def apply_reset_rotation(quat_str, reset_rot):
    """Apply reset rotation to a quaternion"""
    # Parse quaternion
    w, x, y, z = map(float, quat_str.split(','))
    rot = R.from_quat([x, y, z, w])
    
    # Apply reset rotation
    new_rot = reset_rot * rot
    
    # Convert back to w,x,y,z string
    result = new_rot.as_quat()
    return f"{result[3]},{result[0]},{result[1]},{result[2]}"

def quaternion_inverse(quat_str):
    """
    Calculate inverse of quaternion from string format
    Input: quaternion string "w,x,y,z"
    Output: inverse quaternion as scipy Rotation object
    """
    # Parse quaternion string
    w, x, y, z = map(float, quat_str.split(','))
    
    # Convert to scipy rotation object
    rot = R.from_quat([x, y, z, w])
    
    # Return inverse rotation
    return rot.inv()

def apply_inverse_rotation(quat_str, inv_rot):
    """
    Apply inverse rotation to a quaternion
    Input: 
        quat_str: quaternion string "w,x,y,z"
        inv_rot: inverse rotation as scipy Rotation object
    Output: resulting quaternion as "w,x,y,z" string
    """
    try:
        # Parse input quaternion
        w, x, y, z = map(float, quat_str.split(','))
        
        # Validate and normalize
        w, x, y, z = validate_quaternion(w, x, y, z)
        
        # Convert to rotation object
        rot = R.from_quat([x, y, z, w])
        
        # Apply inverse rotation (multiplication)
        result_rot =  rot * inv_rot   # Changed multiplication order
        
        # Convert back to w,x,y,z string format
        result = result_rot.as_quat()
        return f"{result[3]},{result[0]},{result[1]},{result[2]}"
    except ValueError as e:
        print(f"Warning: Invalid quaternion {quat_str}: {e}")
        return quat_str  # Return original if invalid
    
def validate_quaternion(w, x, y, z):
    """
    Validate and normalize quaternion
    """
    # Check for zero quaternion
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-10:  # Small threshold for numerical stability
        raise ValueError("Invalid zero quaternion")
    
    # Normalize
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    return w, x, y, z

if __name__ == "__main__":
    input_file = r"C:\EPFL\Semester Project\MARKER EXPERIMENT\exp2_r\Y90_mid_vis_inv.sto"
    output_file = r"C:\EPFL\Semester Project\MARKER EXPERIMENT\exp2_r\Y90_mid_vis_inv.sto"
    
    # List of IMUs to process
    imus = ['humerus_imu', 'scapula_imu', 'thorax_imu']
    
    # Read data
    header, data = read_sto(input_file)
    
    # Process each IMU
    for imu in imus:
        # Get first frame quaternion
        first_quat = data.iloc[0][imu]
        
        # Calculate inverse of first frame quaternion
        inv_rot = quaternion_inverse(first_quat)
        
        # Apply inverse rotation to all frames
        data[imu] = data[imu].apply(
            lambda q: apply_inverse_rotation(q, inv_rot)
        )
        
        print(f"Applied inverse rotation to {imu}")
    
    # Save results
    write_sto(output_file, header, data)
    print(f"Saved results to {output_file}")


# # Example usage
# if __name__ == "__main__":
#     input_file = r"C:\EPFL\Semester Project\MARKER EXPERIMENT\exp2_r\mid_vis_inv.sto"
#     output_file = r"C:\EPFL\Semester Project\MARKER EXPERIMENT\exp2_r\Y90_mid_vis_inv.sto"
    
#     rotations = {
#         'humerus_imu': [
#             ('y', 90)   # Rotate around Z axis by 60 degrees
#         ],
#         'scapula_imu': [
#             ('y', 90)   # Rotate around Z axis by 60 degrees
#         ],
#         'thorax_imu': [
#             ('y', 90)   # Rotate around Z axis by 60 degrees
#         ]
#     }
    
    
#     manipulate_quaternions(input_file, output_file, rotations)
