# """
# Get OpenSim Coordinate System
# """
# import opensim as osim

# # 加载模型
# model_path = "C:/EPFL/Semester Project/Scap Joint/1.Model-latest/ScapulothorachicJoint_Shoulder.osim"
# model = osim.Model(model_path)
# state = model.initSystem()

# # 获取 scapula 的 Transform
# scapula = model.getBodySet().get('humerus')
# transform = scapula.getTransformInGround(state)
# rotation = transform.R()

# # 提取旋转矩阵和位移向量
# rotation = transform.R()  # 旋转矩阵
# translation = transform.p()  # 平移向量

# # 打印结果
# # print("Rotation:")
# # print(rotation)

# # print("Translation Vector (T):")
# # print(translation)

# print("Rotation Matrix (R):")
# for i in range(3):  # 遍历行
#     row = [rotation.get(i, j) for j in range(3)]
#     print(f"[{row[0]:.3f}, {row[1]:.3f}, {row[2]:.3f}]")

# Scapula Rotation Matrix (R) w.r.t Ground
# [0.950, -0.026, 0.313]
# [0.094, 0.974, -0.207]
# [-0.299, 0.226, 0.927]

# Thoracic Matrix (R) w.r.t Ground
# [1.000, 0.000, 0.000]
# [0.000, 1.000, 0.000]
# [0.000, 0.000, 1.000]

# Humerus Matrix (R) w.r.t Ground
# [0.953, -0.092, 0.289]
# [0.108, 0.993, -0.038]
# [-0.284, 0.067, 0.957]

# """
# Data Transformation for OpenSim Quaternion Data
# """
import numpy as np
import pandas as pd

input_file = r"exp3_vis.sto"  # Replace with your input .sto file path
output_file = r"transformed_exp3_vis"  # Replace with your desired output .sto file path
IMU_INSTALL_ROTATION = False    # IMU SETUP Set to True 

# # Rotation matrices for each body part
R_scapula = np.array([
    [0.950, -0.026,  0.313],
    [0.094,  0.974, -0.207],
    [-0.299,  0.226,  0.927]
])

R_humerus = np.array([
    [0.953, -0.092,  0.289],
    [0.108,  0.993, -0.038],
    [-0.284,  0.067,  0.957]
])

R_thorax = np.array([
    [1.000,  0.000,  0.000],
    [0.000,  1.000,  0.000],
    [0.000,  0.000,  1.000]
])

# # Transformation matrix T (Markers to OpenSim global frame)
T = np.array([
    [  1,  0,  0],
    [  0,  1,  0],
    [  0, 0,  1]
])

# Convert quaternion to rotation matrix
def quaternion_to_rotation_matrix(q):
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])

# Convert rotation matrix to quaternion
def rotation_matrix_to_quaternion(R):
    qw = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    qx = (R[2, 1] - R[1, 2]) / (4 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4 * qw)
    return np.array([qw, qx, qy, qz])

def transform_quaternion(q_BNO085, R_install, R_local):
    # Step 1: 将IMU的四元数转换为旋转矩阵
    R_BNO085 = quaternion_to_rotation_matrix(q_BNO085)

    # Step 2: 从IMU的世界坐标系转换到OpenSim的世界坐标系
    if IMU_INSTALL_ROTATION:
        R_OpenSim_Global = R_install @ R_BNO085
    else:
        R_OpenSim_Global = T @ R_BNO085
    
    # Step 3: 从OpenSim的世界坐标系转换到OpenSim的局部坐标系
    R_OpenSim_Local = R_local @ R_OpenSim_Global

    # Step 4: 将最终的旋转矩阵转换回四元数
    q_OpenSim_Local = rotation_matrix_to_quaternion(R_OpenSim_Local)
    return q_OpenSim_Local

# Read .sto file
def read_sto(file_path):
    with open(file_path, 'r') as f:
        # Read the header lines
        header = []
        while True:
            line = f.readline().strip()
            header.append(line)
            if line.startswith("endheader"):
                break
        
        # Read the data into a DataFrame
        data = pd.read_csv(f, delim_whitespace=True)
    
    return header, data

# Write .sto file
def write_sto(file_path, header, data):
    with open(file_path, 'w') as f:
        # Write the header lines
        for line in header:
            f.write(line + '\n')
        
        # Write the data
        f.write('\t'.join(data.columns) + '\n')  # Write column names
        for _, row in data.iterrows():
            row_string = '\t'.join(map(str, row.values))
            f.write(row_string + '\n')

R_install_scapula = np.array([
    [ 0,  -1,  0],
    [  0,  0,  1],
    [  -1 , 0,  0]
])

R_install_humerus = np.array([
    [ 0,  -1,  0],
    [  0,  0,  1],
    [  -1 , 0,  0]
])

theta_humerus = np.radians(-25)  # Convert 10 degrees to radians
R_z_humerus = np.array([
    [np.cos(theta_humerus), -np.sin(theta_humerus),0], 
    [np.sin(theta_humerus), np.cos(theta_humerus), 0],
    [0, 0, 1] 
])
# Update the humerus installation rotation matrix to include the additional rotation    
R_install_humerus = R_install_humerus @ R_z_humerus


R_install_thorax = np.array([
    [ 0,  -1,  0],
    [  0,  0,  1],
    [  -1 , 0,  0]
])

# Process the .sto file and apply transformations
def process_sto(input_file, output_file):
    # Read the .sto file
    header, data = read_sto(input_file)
    
    # 对肩胛骨IMU数据进行转换
    scapula_data = data['scapula_imu']
    transformed_scapula_data = []
    for quaternion_str in scapula_data:
        q_BNO085 = np.array([float(x) for x in quaternion_str.split(',')])
        q_transformed = transform_quaternion(q_BNO085, R_install_scapula, R_scapula)
        transformed_scapula_data.append(','.join([f"{x:.6f}" for x in q_transformed]))
    data['scapula_imu'] = transformed_scapula_data

    # 对肱骨IMU数据进行转换
    humerus_data = data['humerus_imu']
    transformed_humerus_data = []
    for quaternion_str in humerus_data:
        q_BNO085 = np.array([float(x) for x in quaternion_str.split(',')])
        q_transformed = transform_quaternion(q_BNO085, R_install_humerus, R_humerus)
        transformed_humerus_data.append(','.join([f"{x:.6f}" for x in q_transformed]))
    data['humerus_imu'] = transformed_humerus_data

    # 对胸腔IMU数据进行转换
    thorax_data = data['thorax_imu']
    transformed_thorax_data = []
    for quaternion_str in thorax_data:
        q_BNO085 = np.array([float(x) for x in quaternion_str.split(',')])
        q_transformed = transform_quaternion(q_BNO085, R_install_thorax, R_thorax)
        transformed_thorax_data.append(','.join([f"{x:.6f}" for x in q_transformed]))
    data['thorax_imu'] = transformed_thorax_data
    
    # Write the updated .sto file
    write_sto(output_file, header, data)
    print(f"Transformed data saved to {output_file}")

process_sto(input_file, output_file)


