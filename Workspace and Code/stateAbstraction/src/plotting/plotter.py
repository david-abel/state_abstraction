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
from collections import defaultdict
import numpy as np
import scipy as sp
import scipy.stats as stats
import math
from math import log10, floor

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

		#Grab legend name
		legendInfo = ""
		if len(yDataNames) > yDataIndex:
			legendInfo = yDataNames[yDataIndex]

		#If Y data is a constant, convert it to a list
		if (not type(yData) == list):
			maxY = max(maxY, yData)
			plt.plot(xData, [yData]*len(xData), label=legendInfo, marker="o", markersize=10)
		#If a list of constants plot as a line
		elif(not type(yData[0]) == list):
			maxY = max(maxY, max(yData))
			plt.plot(xData, yData, label=legendInfo, marker="o", markersize=10)
		#If a list of lists, plot CIs if there is data for multiple trials
		elif (type(yData[0]) == list):
			means = []
			errors = []
			numTrials = []
			for yDataAcrossTrials in yData:
				mean, plusminus, numTrial = mean_confidence_interval(yDataAcrossTrials, confidence)
				errors.append(plusminus)
				means.append(mean)
				numTrials.append(numTrial)
			maxY = max(maxY, max(means))
			plt.errorbar(xData, means, yerr=errors, label=legendInfo)
			#Add annotation of # trials
			for (i, x), y in zip(enumerate(xData), means):
				matplotlib.rcParams.update({'font.size': 8})
				plt.annotate(numTrials[i], xy=(x, y))

	#Set x axis limits	
	plt.xlim([min(0,min(xData)), max(xData)*1.01])

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
	plt.savefig(taskName.lower() + '/' + xAxisLabel.lower() + '_vs_' + yAxisLabel.lower().replace(" ", "_") + '.pdf', format="pdf")
	plt.clf()
	# plt.show()


def mean_confidence_interval(data, confidence):
	mean, sigma = np.mean(data), np.std(data)
	#If all same element return 0 for interval
	if len(set(data)) <= 1:
		return mean, 0, len(data)

	#Otherwise calculate bounds
	lowerMean, upperMean = stats.norm.interval(confidence, loc=mean, scale=sigma/math.sqrt(len(data)))

	return mean, mean-lowerMean, len(data)

def makeAllPlots():
	allLocalDirs = [f for f in listdir(".") if isdir(join(".", f))]

	for taskName in allLocalDirs:
		makePlot(taskName)

def round_sig(x, sig=5):
	if (x == 0):
		return 0
	return round(x, sig-int(floor(log10(x)))-1)


def combineTrialData(trialToDataHM, yDataIndex, xDataIndex):
	xToListOfYData = defaultdict(list)
	#Create HM from x values to list of y values across trials
	for trialIndex, trialData in trialToDataHM.iteritems():
		xData = trialToDataHM[trialIndex][xDataIndex]
		for xIndex, x in enumerate(xData):
			roundedX = round_sig(x)
			xToListOfYData[roundedX].append(trialToDataHM[trialIndex][yDataIndex][xIndex])

	#Convert HM to list
	values = [(x , y) for x,y in xToListOfYData.iteritems()]
	values = sorted(values, key=lambda x: x[0])

	xValues = [x for (x,y) in values]
	yValues = [y for (x,y) in values]

	return xValues, yValues
		

def makePlot(taskName):
	resultsFile = loadFile(taskName)

	# Get results.
	trialIndexToData = parseResultsFile(resultsFile)
	numGroundStates = trialIndexToData[0][0][0]
	randPolVal = trialIndexToData[0][1][0]
	groundStateVals = trialIndexToData[0][4][0]

	# numGroundStates, randVal, epsilons, numStates, groundStateVals, abstractStateVals = parseResultsFile(resultsFile)[0]

	numAbstractStatesEpsilons, numAbstractStates = combineTrialData(trialIndexToData, 3, 2)
	abstratStateValEpsilons, abstractStateVals = combineTrialData(trialIndexToData, 5, 2)

	# Plot with CIs.
	plotWithConfidenceIntervals(numAbstractStatesEpsilons, [numAbstractStates, numGroundStates],"Num Abstract States", taskName, yDataNames=["Num. Abstract States", "Num. Ground States"])
	plotWithConfidenceIntervals(abstratStateValEpsilons, [abstractStateVals, groundStateVals, randPolVal], "Value of Abstract Policy Init Ground State", taskName, yDataNames=["Val. Abstract Policy", "Val. Optimal Policy", "Val. Random Policy"])

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
