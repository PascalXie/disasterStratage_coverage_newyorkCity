import numpy as np
import csv
import string
import math

import seaborn as sns
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


def plot(typeName, ID):
	# read
	f = open('alice_step1_part2_populationDensityGenerator/build/data_step1_part2_'+typeName+'_time_'+str(ID)+'.txt')
	lines = f.readlines()
	f.close()

	xs = []
	ys = []
	values = []
	values_log = []
	for line in lines:
		line = line.strip().split()
		xs.append(round(float(line[2])/1000.,2))
		ys.append(round(float(line[3])/1000.,2))

		value = float(line[4])
		values.append(value)

		if value==0:
			value = 1e-4;
		value_log = math.log10(value)
		values_log.append(value_log)


	df = pd.DataFrame({'y':ys,'x': xs,'Content':values, 'logValue':values_log})
	pt = df.pivot_table(index='y', columns='x', values='Content', aggfunc=np.sum)
	pt_log = df.pivot_table(index='y', columns='x', values='logValue', aggfunc=np.sum)

	f, ax1 = plt.subplots(figsize = (8,7),nrows=1)
	cmap = sns.diverging_palette(200,20,sep=20,as_cmap=True)
	sns_plot = sns.heatmap(pt_log, linewidths = 0.00, ax = ax1, cmap="RdPu", xticklabels=12, yticklabels=12)
	sns_plot.tick_params(labelsize=10)

	ax1.invert_yaxis()
	ax1.set_title('Population Density Distribution of Districts in New York City', fontsize=14)
	ax1.set_xlabel('X / km',fontsize=12)
	ax1.set_ylabel('Y / km',fontsize=12)

	plt.savefig('figure-step1-'+typeName+'-time-'+str(ID)+'.jpeg',dpi=300, bbox_inches='tight')
	#plt.show()
	return

def plot_force(typeName, ID):
	# read
	f = open('alice_step1_part2_populationDensityGenerator/build/data_step1_part2_'+typeName+'_time_'+str(ID)+'.txt')
	lines = f.readlines()
	f.close()

	xs = []
	ys = []
	values = []
	for line in lines:
		line = line.strip().split()
		xs.append(round(float(line[2])/1000.,2))
		ys.append(round(float(line[3])/1000.,2))

		value = float(line[4])
		values.append(value)

	df = pd.DataFrame({'y':ys,'x': xs,'Content':values})
	pt = df.pivot_table(index='y', columns='x', values='Content', aggfunc=np.sum)

	f, ax1 = plt.subplots(figsize = (8,7),nrows=1)
	cmap = sns.diverging_palette(200,20,sep=20,as_cmap=True)
	sns_plot = sns.heatmap(pt, linewidths = 0.00, ax = ax1, cmap="RdPu", xticklabels=12, yticklabels=12)
	sns_plot.tick_params(labelsize=10)

	ax1.invert_yaxis()
	ax1.set_title('Population Density Distribution of Districts in New York City', fontsize=14)
	ax1.set_xlabel('X / km',fontsize=12)
	ax1.set_ylabel('Y / km',fontsize=12)

	plt.savefig('figure-step1-'+typeName+'-time-'+str(ID)+'.jpeg',dpi=300, bbox_inches='tight')
	#plt.show()
	return

if __name__ == '__main__':
	print("hello")

	for i in range(40):
		print("Time Step ID : {}".format(i))
		plot('population',i)
		plot_force('forcex',i)
		plot_force('forcey',i)
