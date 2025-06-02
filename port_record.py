import serial
import serial.tools.list_ports
import time
import os
import csv

# List available ports
print("Available ports:")
ports = list(serial.tools.list_ports.comports())
for p in ports:
    print(f"  {p}")

# configure serial port
port = 'COM5'  # Change this if needed
baud_rate = 2000000

csv_file = 'C:/EPFL/Semester Project/15MAY2025_test3.csv'
os.makedirs(os.path.dirname(csv_file), exist_ok=True)
try:
    ser = serial.Serial(port, baud_rate)
    time.sleep(2)  
    
    # record start time
    start_time = time.time()

    file_exists = os.path.isfile(csv_file)

    # Open the CSV file
    with open(csv_file, 'a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(['Time (s)', 'Data'])

        print(f"Start recording... (from {port} port)")
        try:
            while True:
                if ser.in_waiting: 
                    data = ser.readline().decode().strip()  
                    if data:
                        # 计算从开始到现在的时间（4位小数）
                        current_time = "{:.3f}".format(time.time() - start_time)
                        writer.writerow([current_time, data])  
                        file.flush()  # 确保数据被写入文件
                        print(f"{current_time} -> {data}")
        
        except KeyboardInterrupt:
            print("\nStop recording data.")
        finally:
            ser.close()  

except serial.SerialException as e:
    print(f"Error: Unable to access {port}")