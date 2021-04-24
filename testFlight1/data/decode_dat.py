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

<<<<<<< HEAD
		packet_length = 156
=======
		packet_length = 160
>>>>>>> main_cleanup
		data_rejoined = b'\r\n'.join(data_split[1:])
		sensor_datapackets = [data_rejoined[i:i+packet_length] for i in range(0,len(data_rejoined),packet_length)]
		# print(sensor_datapackets)
		test2 = True
		for datapacket in sensor_datapackets:
<<<<<<< HEAD
			data_list = list(struct.unpack('f'*17,datapacket[:68]))

			gps_lock_int = struct.unpack('h',datapacket[68:70])[0]
=======
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
			gps_lock_int = struct.unpack('i',datapacket[68:72])[0]
>>>>>>> main_cleanup
			if gps_lock_int == 1:
				data_list.append(True)
			elif gps_lock_int == 0:
				data_list.append(False)
			else:
				test = 1
				# print('Failed to interpret gps lock data.',gps_lock_int)
			data_list += list(struct.unpack('i',datapacket[72:76]))# gps timestamp

			data_list += list(struct.unpack('f'*18,datapacket[76:148]))# state data
			data_list += list(struct.unpack('i',datapacket[148:152]))# state timestamp

<<<<<<< HEAD
			data_list += list(struct.unpack('f'*20,datapacket[70:150]))

			rocket_state_int = struct.unpack('h',datapacket[150:152])[0]
=======
			rocket_state_int = struct.unpack('i',datapacket[152:156])[0]
>>>>>>> main_cleanup
			# print(rocket_state_int)
			data_list.append(rocket_state_int)

			# print(len(datapacket[64:]))
			try:
<<<<<<< HEAD
				timestamp = struct.unpack('i',datapacket[152:])[0]
=======
				timestamp = struct.unpack('i',datapacket[156:])[0]
>>>>>>> main_cleanup
				data_list.append(timestamp)
			except:
				data_list.append('error')


			write_to_file(csvwriter,data_list)


		csvfile.close()




