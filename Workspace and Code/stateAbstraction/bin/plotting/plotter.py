# ===================================================================
# =========== Authors: David Abel and D Ellis Hershkowits ===========
# == Summary: Plotting tool for Approximate State Abstraction work ==
# ===================================================================

# Python imports.
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import sys
import os
from os import listdir
from os.path import isdir, join

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

	for line in file(resultsFile,"r").readlines():
		results = line.split("\t") # Tab separated result file.
		epsilons.append(float(results[0]))
		numStates.append(int(results[1]))
		groundStateVals.append(float(results[2]))
		abstractStateVals.append(float(results[3]))

	return epsilons, numStates, groundStateVals, abstractStateVals

def plot(xData, yData, yAxisLabel, taskName, scatter=False, xAxisLabel="Epsilon", epsilons=[]):
	
	if scatter:
		# Scatter plot.
		

		for i in xrange(len(xData)):
			plt.scatter(xData[i], yData[i], s=(epsilons[i]**2+1)*20, alpha=0.5)

		
	else: 
		# Line plot.
		plt.plot(xData, yData, marker="o", color="black")

	font = {'family' : 'normal',
	        'size'   : 15}

	matplotlib.rc('font', **font)
	plt.xlim([min(0,min(xData)), max(xData)])
	plt.ylim([0, max(yData) + 5.0])
	plt.xlabel(xAxisLabel)
	plt.ylabel(yAxisLabel)
	plt.title(taskName[0].upper() + taskName[1:] + ': ' + xAxisLabel + ' vs. ' + yAxisLabel)
	plt.grid(True)
	plt.savefig(taskName.lower() + '/' + xAxisLabel.lower() + '_vs_' + yAxisLabel.lower().replace(" ", "_") + '.png')
	plt.clf()
	# plt.show()
	

def makeAllPlots():
	allLocalDirs = [f for f in listdir(".") if isdir(join(".", f))]

	for taskName in allLocalDirs:
		makePlot(taskName)


def makePlot(taskName):
	resultsFile = loadFile(taskName)

	# Get results.
	epsilons, numStates, groundStateVals, abstractStateVals = parseResultsFile(resultsFile)

	performances = [abs(groundStateVals[i] - abstractStateVals[i]) for i in xrange(len(groundStateVals))]

	# Plot.
	plot(epsilons, numStates, "Num States", taskName)
	plot(epsilons, performances, "Abstract vs. Ground Init State Value", taskName)
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
	main()
