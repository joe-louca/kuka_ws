import serial

 

ser = serial.Serial('COM4',9800,timeout=1)

data = ser.readline()

data_list = data.split(", ")

 

EMG = float(data_list[0])

HR = int(data_list[1])

 

# Create timestamped msgs

# Publish to topics / save as txt files
