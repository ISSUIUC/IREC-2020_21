import numpy as np
import struct
import csv
import os

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



for dat_file in dats_to_convert:
	with open(dat_file,'rb') as file:
		data = file.read()
		file.close()

	with open(dat_file.split('.')[0] + '.CSV','w',newline='') as csvfile:
		csvwriter = csv.writer(csvfile,delimiter=',')



		data_split = data.split(b'\r\n')
		write_to_file(csvwriter,data_split[0].decode('ascii').split(','))

		packet_length = 156
		data_rejoined = b'\r\n'.join(data_split[1:])
		sensor_datapackets = [data_rejoined[i:i+packet_length] for i in range(0,len(data_rejoined),packet_length)]
		# print(sensor_datapackets)
		for datapacket in sensor_datapackets:
			data_list = list(struct.unpack('f'*17,datapacket[:68]))

			gps_lock_int = struct.unpack('h',datapacket[68:70])[0]
			if gps_lock_int == 1:
				data_list.append(True)
			elif gps_lock_int == 0:
				data_list.append(False)
			else:
				print('Failed to interpret gps lock data.',gps_lock_int)

			data_list += list(struct.unpack('f'*20,datapacket[70:150]))

			rocket_state_int = struct.unpack('h',datapacket[150:152])[0]
			# print(rocket_state_int)
			data_list.append(rocket_state_int)

			# print(len(datapacket[64:]))
			try:
				timestamp = struct.unpack('i',datapacket[152:])[0]
				data_list.append(timestamp)
			except:
				data_list.append('error')


			write_to_file(csvwriter,data_list)


		csvfile.close()




