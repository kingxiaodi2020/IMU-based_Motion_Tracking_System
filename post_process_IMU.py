import pandas as pd
import io

df = pd.read_csv('15MAY2025_test3.csv')
saved_path = 'test3.csv'

# Initialize empty lists for each quaternion component
time = []
q1_w, q1_x, q1_y, q1_z = [], [], [], []
q2_w, q2_x, q2_y, q2_z = [], [], [], []
q3_w, q3_x, q3_y, q3_z = [], [], [], []

# Process each row
for _, row in df.iterrows():
    # Extract time from the Time column
    time_val = row['Time (s)']
    time.append(time_val)
    
    # Split the Data column by commas and remove the first empty element
    values = row['Data'].split(',')
    if values[0] == '':
        values = values[1:]
    
    # Extract quaternions and calibration values
    q1 = values[0:4]
    cal1 = values[4]
    q2 = values[5:9]
    cal2 = values[9]
    q3 = values[10:14]
    cal3 = values[14]
    
    # Append to respective lists
    q1_w.append(float(q1[0]))
    q1_x.append(float(q1[1]))
    q1_y.append(float(q1[2]))
    q1_z.append(float(q1[3]))
    
    q2_w.append(float(q2[0]))
    q2_x.append(float(q2[1]))
    q2_y.append(float(q2[2]))
    q2_z.append(float(q2[3]))
    
    q3_w.append(float(q3[0]))
    q3_x.append(float(q3[1]))
    q3_y.append(float(q3[2]))
    q3_z.append(float(q3[3]))

# Create a new DataFrame with organized columns
processed_df = pd.DataFrame({
    'Time (s)': time,
    'Sensor1_w': q1_w,
    'Sensor1_x': q1_x,
    'Sensor1_y': q1_y,
    'Sensor1_z': q1_z,
    'Sensor2_w': q2_w,
    'Sensor2_x': q2_x,
    'Sensor2_y': q2_y,
    'Sensor2_z': q2_z,
    'Sensor3_w': q3_w,
    'Sensor3_x': q3_x,
    'Sensor3_y': q3_y,
    'Sensor3_z': q3_z
})

# Save to a new CSV file
processed_df.to_csv(saved_path, index=False)

# Display the first few rows of the processed DataFrame
print(processed_df.head())