import numpy as np

def mirror_quaternion(q):
    # q = [w, x, y, z]
    # 镜像后 q_mirror = [w, x, -y, -z]
    return np.array([q[0], q[1], -q[2], -q[3]])

def process_sto_file(input_file, output_file):
    with open(input_file, 'r') as f:
        lines = f.readlines()
    
    # 找到数据开始的行
    header_end = 0
    for i, line in enumerate(lines):
        if line.startswith('endheader'):
            header_end = i
            break
    
    # 写入新文件
    with open(output_file, 'w') as f:
        # 写入头部
        for i in range(header_end + 1):
            f.write(lines[i])
            
        # 处理数据行
        header = lines[header_end + 1].strip().split('\t')
        f.write('\t'.join(header) + '\n')
        
        for line in lines[header_end + 2:]:
            if not line.strip():
                continue
                
            data = line.strip().split('\t')
            time = data[0]
            quaternions = [np.array([float(x) for x in q.split(',')]) for q in data[1:]]
            
            # 镜像每个四元数
            mirrored_quaternions = [mirror_quaternion(q) for q in quaternions]
            
            # 转换回字符串格式
            quaternion_strings = [','.join(map(str, q)) for q in mirrored_quaternions]
            
            # 写入新行
            f.write(f"{time}\t" + "\t".join(quaternion_strings) + "\n")
        print(f"处理完成，结果已保存到 {output_file}")

# 使用示例
input_file = output_file = r"C:\EPFL\Semester Project\MARKER EXPERIMENT\exp2_r\transformed_mid"   # 输入文件
output_file = r"C:\EPFL\Semester Project\MARKER EXPERIMENT\exp2_r\transformed_mid_mirrored"   # 输出文件

process_sto_file(input_file, output_file)