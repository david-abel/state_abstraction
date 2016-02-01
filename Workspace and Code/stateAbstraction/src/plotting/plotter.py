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
import numpy as np
import scipy as sp
import scipy.stats as stats
import math

_markers = ['v','^','s']

def loadFile(taskName):

	resultsFile = taskName + "/" + taskName + ".results"
	
	if not os.path.isfile(resultsFile):
		print "Error: file '" + resultsFile + "' not found."
		quit()

	return resultsFile

def parseTrialData(trialData):
	epsilons = []
	numStates = []
	groundStateVals = []
	abstractStateVals = []
	rand = []
	numGroundStates = []
	for line in trialData.split("\n")[:-1]:
		results = line.split("\t")
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
	return [numGroundStates, rand, epsilons, numStates, groundStateVals, abstractStateVals]

def parseResultsFile(resultsFile):
	entireFile = ""
	for line in file(resultsFile,"r").readlines():
		entireFile += line
	resultsSplitByTrial = entireFile.split("~~~\n")[:-1]

	trialIndexToData = {}
	for currTrialData, currTrialIndex in zip(resultsSplitByTrial, range(0, len(resultsSplitByTrial))):
		trialIndexToData[currTrialIndex] = parseTrialData(currTrialData)

	return trialIndexToData

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
	
def	plotWithConfidenceIntervals(xData, allYData, yAxisLabel, taskName, confidence=.95, xAxisLabel="Epsilon", yDataNames=[]):
	
	maxY = -99999

	for yDataIndex, yData in enumerate(allYData):
		means = []
		errors = []
		#Setup legend names
		legendInfo = ""
		if len(yDataNames) > yDataIndex:
			legendInfo = yDataNames[yDataIndex]

		#Plot CIs if there is data for multiple trials
		if (type(yData[0]) == list):
			for yDataAcrossTrials in yData:
				mean, plusminus = mean_confidence_interval(yDataAcrossTrials, confidence)
				if math.isnan(plusminus):
					errors.append(0)
				else:
					errors.append(plusminus)
				means.append(mean)
			maxY = max(maxY, max(means))
			plt.errorbar(xData, means, yerr=errors, label=legendInfo)
		#Otherwise just plot it without
		else:
			maxY = max(maxY, max(yData))
			plt.plot(xData, yData, label=legendInfo, marker="o", markersize=10)

	#Set x axis limits	
	plt.xlim([min(0,min(xData)), max(xData)])

	#Set y axis limits
	plt.ylim([0,maxY *1.2])

	#Set font
	font = {'family' : 'normal',
	        'size'   : 15}

	matplotlib.rc('font', **font)
	plt.xlabel(xAxisLabel)
	plt.ylabel(yAxisLabel)
	plt.title(taskName[0].upper() + taskName[1:] + ': ' + xAxisLabel + ' vs. ' + yAxisLabel)

	plt.grid(True)
	plt.legend()
	plt.savefig(taskName.lower() + '/' + xAxisLabel.lower() + '_vs_' + yAxisLabel.lower().replace(" ", "_") + '.png')
	plt.clf()
	# plt.show()


def mean_confidence_interval(data, confidence):
	mean, sigma = np.mean(data), np.std(data)
	lowerMean, upperMean = stats.norm.interval(confidence, loc=mean, scale=sigma/math.sqrt(len(data)))

	return mean, mean-lowerMean

def makeAllPlots():
	allLocalDirs = [f for f in listdir(".") if isdir(join(".", f))]

	for taskName in allLocalDirs:
		makePlot(taskName)

def combineTrialData(trialToDataHM, indexOfData):
	toReturn = []

	for index, trialData in trialToDataHM.iteritems():
		currData = trialData[indexOfData]
		if len(toReturn) == 0:
			for dataPoint in currData:
				toReturn.append([dataPoint])
		else:
			for index, dataPoint in enumerate(currData):
				toReturn[index].append(dataPoint)
		
	return toReturn

def makePlot(taskName):
	resultsFile = loadFile(taskName)

	# Get results.
	trialIndexToData = parseResultsFile(resultsFile)
	epsilons = trialIndexToData[0][2]
	numGroundStates = trialIndexToData[0][0]
	randVal = trialIndexToData[0][1]
	groundStateVals = trialIndexToData[0][4]

	# numGroundStates, randVal, epsilons, numStates, groundStateVals, abstractStateVals = parseResultsFile(resultsFile)[0]

	numAbstractStates = combineTrialData(trialIndexToData, 3)
	abstractStateVals = combineTrialData(trialIndexToData, 5)

	# randDiffs = [abs(perf - randVal[0]) for perf in groundStateVals]
	# performances = [abs(groundStateVals[i] - abstractStateVals[i]) for i in xrange(len(groundStateVals))]

	# Get the random policy value and the number of ground states.
	randYData = randVal*len(abstractStateVals)
	numGroundStatesYData = numGroundStates*len(numAbstractStates)

	# Plot with CIs.
	plotWithConfidenceIntervals(epsilons, [numAbstractStates, numGroundStatesYData],"Num Abstract States", taskName, yDataNames=["Num. Abstract States", "Num. Ground States"])
	plotWithConfidenceIntervals(epsilons, [abstractStateVals, groundStateVals, randYData], "Value of Abstract Policy Init Ground State", taskName, yDataNames=["Val. Abstract Policy", "Val. Optimal Policy", "Val. Random Policy"])

	#Old plotting without CIs
	# plot(epsilons, [numStates, numGroundStatesYData], "Num Abstract States", taskName, yDataNames=["Num. Abstract States", "Num. Ground States"])
	# plot(epsilons, [abstractStateVals, groundStateVals, randYData], "Value of Abstract Policy Init Ground State", taskName, yDataNames=["Val. Abstract Policy", "Val. Optimal Policy", "Val. Random Policy"])
	# plot(abstractStateVals, numStates, "Num States", taskName, scatter=True, xAxisLabel="Abstract Policy Value", epsilons=epsilons)

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
