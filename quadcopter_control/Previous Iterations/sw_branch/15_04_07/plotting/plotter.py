import matplotlib.pyplot as plt
import csv

def show():
	collect = {
		'lx': [],
		'ly': [],
		'lz': [],
		'ax': [],
		'ay': [],
		'az': []
	}

	files = ['lx', 'ly', 'lz', 'ax', 'ay', 'az']
	for f in files:
		with open('../txt/'+f+'.txt', 'rb') as csvfile:
			csvreader = csv.reader(csvfile, delimiter=',')
			for row in csvreader:
				for item in row:
					if len(item) > 0:
						collect[f].append(float(item))

	f, axarr = plt.subplots(2, sharex=True)

	axarr[0].plot(collect['lx'], color='r', label='lx')
	axarr[0].plot(collect['ly'], color='g', label='ly')
	axarr[0].plot(collect['lz'], color='b', label='lz')
	axarr[0].set_title('Linear velocity')
	axarr[0].legend()
	axarr[1].plot(collect['ax'], color='r', label='ax')
	axarr[1].plot(collect['ay'], color='g', label='ay')
	axarr[1].plot(collect['az'], color='b', label='az')
	axarr[1].set_title('Angular velocity')
	axarr[1].legend()
	plt.show()