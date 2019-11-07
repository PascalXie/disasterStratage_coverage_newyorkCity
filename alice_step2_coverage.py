import numpy as np
import csv
import string
import math

import seaborn as sns
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

def drawAPlot(dataFileName, mode):
	print("Reading : {}".format(dataFileName))

	data = open(dataFileName)
	lines = data.readlines()
	data.close()

	xs = []
	ys = []
	values = []
	values_log = []
	for line in lines:
		line = line.strip().split()
		xs.append(round(float(line[0])/1000.,1))
		ys.append(round(float(line[1])/1000.,1))

		value = float(line[2])
		values.append(value)

	#
	df = pd.DataFrame({'y':ys,'x': xs,'CostFunctionValues':values})
	pt = df.pivot_table(index='y', columns='x', values='CostFunctionValues', aggfunc=np.sum)

	f, ax1 = plt.subplots(figsize = (6,5),nrows=1)
	cmap = sns.diverging_palette(200,20,sep=20,as_cmap=True)

	sns_plot = sns.heatmap(pt, linewidths = 0.00, ax = ax1, cmap="viridis", xticklabels=12, yticklabels=12)
	sns_plot.tick_params(labelsize=10)

	ax1.invert_yaxis()
	ax1.set_title('Evenly Deploying for '+mode, fontsize=12)
	ax1.set_xlabel('X / km',fontsize=10)
	ax1.set_ylabel('Y / km',fontsize=10)

	plt.savefig('figure-DistributionOfDeployment_CoverageStep1_'+mode+'.jpeg',dpi=300, bbox_inches='tight')
	#plt.show()

	return

if __name__ == '__main__':
	print("hello")

	#filename = "alice_step2_UAVsCoverage_part1_ForMaximumCoverage/build/data_distributionCoverage_coverStep1.txt"
	#drawAPlot(filename, "MaximumCoverage")

	#filename = "alice_step2_UAVsCoverage_part2_ForLocalization/build/data_distributionCoverage_coverStep1.txt"
	#drawAPlot(filename, "ForLocalization")

	filename = "alice_step2_UAVsCoverage_part3_ForMaximumCoverage_withGeneticAl/build/data_distributionCoverage_coverStep1.txt"
	drawAPlot(filename, "MaximumCoverage")

	plt.show()
