# ===================================================================
# =========== Authors: David Abel and D Ellis Hershkowits ===========
# == Summary: Plotting tool for Approximate State Abstraction work ==
# ===================================================================

# Python imports.
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import pylab
import sys
import os
from os import listdir
from os.path import isdir, join

_markers = ['v','^','s']

def loadFile(taskName):

	resultsFile = taskName + "/" + taskName + ".results"
	
	if not os.path.isfile(resultsFile):
		print "Error: file '" + resultsFile + "' not found."
		quit()

	return resultsFile

def parseResultsFile(resultsFile):
	epsilons = []
	numStates = []
	groundStateVals = []
	abstractStateVals = []
	rand = []
	numGroundStates = []

	for line in file(resultsFile,"r").readlines():
		results = line.split("\t") # Tab separated result file.

		if "RAND" in results:
			# Add rand if it's in there.
			rand.append(float(results[1]))
			continue
		if "NUMGROUNDSTATES" in results:
			numGroundStates.append(int(results[1]))
			continue

		epsilons.append(float(results[0]))
		numStates.append(int(results[1]))
		groundStateVals.append(float(results[2]))
		abstractStateVals.append(float(results[3]))

	return numGroundStates, rand, epsilons, numStates, groundStateVals, abstractStateVals

def plot(xData, yData, yAxisLabel, taskName, scatter=False, xAxisLabel="Epsilon", epsilons=[], yDataNames=[]):
	
	if scatter:
		# Scatter plot.
		for i in xrange(len(xData)):
			plt.scatter(xData[i], yData[i], s=(epsilons[i]*200)/max(epsilons), alpha=0.5)
	elif type(yData[0]) == list:

		for i, yDatum in enumerate(yData):
			if len(yDataNames) >= i:
				legendInfo = yDataNames[i]
			else:
				legendInfo = ""
			plt.plot(xData, yDatum, "-", marker=_markers[i], markersize=10, label=legendInfo)

		plt.xlim([min(0,min(xData)), max(xData)])
		plt.ylim([0, max([max(y) for y in yData]) *1.2])

	else: 
		# Line plot.
		plt.plot(xData, yData, marker="o", color="black", markersize=10)
		plt.xlim([min(0,min(xData)), max(xData)])
		plt.ylim([0, max(yData) + 5.0])


	font = {'family' : 'normal',
	        'size'   : 15}

	matplotlib.rc('font', **font)
	plt.xlabel(xAxisLabel)
	plt.ylabel(yAxisLabel)
	plt.title(taskName[0].upper() + taskName[1:] + ': ' + xAxisLabel + ' vs. ' + yAxisLabel)
	plt.grid(True)
	plt.legend()
	plt.savefig(taskName.lower() + '/' + xAxisLabel.lower() + '_vs_' + yAxisLabel.lower().replace(" ", "_") + '.pdf', format="pdf")
	plt.clf()
	# plt.show()
	

def makeAllPlots():
	allLocalDirs = [f for f in listdir(".") if isdir(join(".", f))]

	for taskName in allLocalDirs:
		makePlot(taskName)


def makePlot(taskName):
	resultsFile = loadFile(taskName)

	# Get results.
	numGroundStates, randVal, epsilons, numStates, groundStateVals, abstractStateVals = parseResultsFile(resultsFile)

	# randDiffs = [abs(perf - randVal[0]) for perf in groundStateVals]
	# performances = [abs(groundStateVals[i] - abstractStateVals[i]) for i in xrange(len(groundStateVals))]

	# Get the random policy value and the number of ground states.
	randYData = randVal*len(abstractStateVals)
	numGroundStatesYData = numGroundStates*len(numStates)

	# Plot.
	plot(epsilons, [numStates, numGroundStatesYData], "Num Abstract States", taskName, yDataNames=["Num. Abstract States", "Num. Ground States"])
	plot(epsilons, [abstractStateVals, groundStateVals, randYData], "Value of Abstract Policy Init Ground State", taskName, yDataNames=["Val. Abstract Policy", "Val. Optimal Policy", "Val. Random Policy"])
	plot(abstractStateVals, numStates, "Num States", taskName, scatter=True, xAxisLabel="Abstract Policy Value", epsilons=epsilons)

def main():

	# Get task name and results file.
	if len(sys.argv) < 2:
		print "Usage: python plotter.py <task_name> (e.g taxi, upworld)"
		quit()
	taskName = sys.argv[1].replace("/","")

	if taskName == "all":
		# Make all plots.
		makeAllPlots()
	else:
		# Make plots for just this task.
		makePlot(taskName)
	

if __name__ == "__main__":
	# Puts the legend into the best location in the plot.
	pylab.rcParams['legend.loc'] = 'best'
	main()
