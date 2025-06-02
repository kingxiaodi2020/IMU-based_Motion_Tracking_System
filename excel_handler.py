import pandas as pd

# 读取 CSV 文件
file_path = r"15May2025_1105-2.csv"  # 确保路径正确
# 生成 .sto 文件内容
sto_file_path = r"exp3-2.sto"
# 读取 CSV 文件
df = pd.read_csv(file_path)

# 确保所有数据是数值类型（OpenSim 需要数值格式）
df = df.apply(pd.to_numeric, errors='coerce')

# 删除包含 NaN 的行
df = df.dropna()

# 获取列名列表（第一列是 time，其余是四元数）
columns = df.columns[1:]

# 以 4 列为一组合并四元数
merged_columns = []
sto_data = df.iloc[:, [0]].copy()  # 先保留时间列

for i in range(0, len(columns), 4):
    if i + 3 < len(columns):  # 确保不超出列数
        col_name = f"{columns[i]}_{columns[i+1]}_{columns[i+2]}_{columns[i+3]}"  # 生成新列名
        sto_data[col_name] = df.iloc[:, i+1].astype(str) + "," + \
                             df.iloc[:, i+2].astype(str) + "," + \
                             df.iloc[:, i+3].astype(str) + "," + \
                             df.iloc[:, i+4].astype(str)
        merged_columns.append(col_name)

# 计算行数和列数（时间列 + 处理后的四元数列）
nRows = sto_data.shape[0]
nColumns = sto_data.shape[1]

with open(sto_file_path, "w") as f:
    f.write("DataRate=100.000000\n")
    f.write("DataType=Quaternion\n")
    f.write("version=3\n")
    f.write("OpenSimVersion=4.1\n")
    f.write("endheader\n")
    f.write("time\t" + "\t".join(merged_columns) + "\n")

    # 写入数据
    for index, row in sto_data.iterrows():
        f.write("\t".join(row.astype(str)) + "\n")

print(f"转换完成，.sto 文件已保存至: {sto_file_path}")
