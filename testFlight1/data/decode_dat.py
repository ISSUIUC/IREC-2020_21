import numpy as np
import struct
import csv
import os

# Helper function to write a line to a csv file
def write_to_file(writer,line):
	writer.writerow(line)


# Get the names of all unconverted DAT files in folder
files = os.listdir()
dats_to_convert = []
for file in files:
	if file.endswith(".DAT"):
		csv_version = file.split('.')[0] + '.CSV'
		if csv_version not in files:
			dats_to_convert.append(file)


# Loop through every dat that hasn't been converted yet
for dat_file in dats_to_convert:
	# Open the dat, read its contents, and close the dat
	with open(dat_file,'rb') as file:
		data = file.read()
		file.close()

	# Open a csv file with the name of the dat file
	with open(dat_file.split('.')[0] + '.CSV','w',newline='') as csvfile:
		# Create an object that can write to the csv
		csvwriter = csv.writer(csvfile,delimiter=',')

		# Split dat data on \r\n so the data heading can be separated from the data
		data_split = data.split(b'\r\n')
		# Write data headings to the csv as the first row
		write_to_file(csvwriter,data_split[0].decode('ascii').split(','))

		# Define the length of one packet of data (check sensorDataStruct_t in dataLog.h and add up all the bytes in it)
		packet_length = 160 # RECALCULATE IF sensorDataStruct_t CHANGES
		data_rejoined = b'\r\n'.join(data_split[1:]) # sometimes \r\n shows up in the data, so stitch the data back together without the heading

		# Split data into chunks of size packet_length
		sensor_datapackets = [data_rejoined[i:i+packet_length] for i in range(0,len(data_rejoined),packet_length)]

		# Loop through each data packet
		for datapacket in sensor_datapackets:

			if test2:
				print(len(datapacket))
				for hexi in datapacket:
					print(hexi)
				test2 = False

			data_list = list(struct.unpack('f'*9,datapacket[:36]))# low g data
			data_list += list(struct.unpack('i',datapacket[36:40]))# low g timestamp

			data_list += list(struct.unpack('f'*3,datapacket[40:52]))# high g data
			data_list += list(struct.unpack('i',datapacket[52:56]))# high g timestamp

			data_list += list(struct.unpack('f'*3,datapacket[56:68]))# gps data
			gps_lock_int = struct.unpack('i',datapacket[68:72])[0] # gps posLock

			if gps_lock_int == 1:
				data_list.append(True)
			elif gps_lock_int == 0:
				data_list.append(False)
			else:
				print('Failed to interpret gps lock data.',gps_lock_int)
			data_list += list(struct.unpack('i',datapacket[72:76]))# gps timestamp

			data_list += list(struct.unpack('f'*18,datapacket[76:148]))# state data
			data_list += list(struct.unpack('i',datapacket[148:152]))# state timestamp

			rocket_state_int = struct.unpack('i',datapacket[152:156])[0] # rocket state
			data_list.append(rocket_state_int)

			try:
				timestamp = struct.unpack('i',datapacket[156:])[0] # rest of the packet should be rocket state timestamp
				data_list.append(timestamp)
			except:
				data_list.append('error')

			# Write all data to a line in the csv
			write_to_file(csvwriter,data_list)

		# Close the csv when done writing all data to it
		csvfile.close()




