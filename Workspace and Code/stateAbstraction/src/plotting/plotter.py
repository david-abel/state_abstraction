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


def loadFile(taskName):

	resultsFile = taskName + "/" + taskName + ".results"
	
	if not os.path.isfile(resultsFile):
		print "Error: file '" + resultsFile + "' not found."
		quit()

	return resultsFile

def parseResultsFile(resultsFile):
	epsilons = []
	numStates = []
	performances = []

	for line in file(resultsFile,"r").readlines():
		results = line.split("\t") # Tab separated result file.
		epsilons.append(float(results[0]))
		numStates.append(int(results[1]))
		performances.append(float(results[2]))

	return epsilons, numStates, performances

def plot(xData, yData, yAxisLabel, taskName):
	# Plot.
	plt.plot(xData, yData, marker="o", color="black")

	font = {'family' : 'normal',
	        'size'   : 15}

	matplotlib.rc('font', **font)
	plt.xlim([min(xData), max(xData)])
	plt.ylim([0, max(yData) + 5.0])
	plt.xlabel('Epsilon')
	plt.ylabel(yAxisLabel)
	plt.title(taskName[0].upper() + taskName[1:] + ': Epsilon vs. ' + yAxisLabel)
	plt.grid(True)
	plt.savefig(taskName.lower() + '/epsilon_vs_' + yAxisLabel.lower().replace(" ", "_") + '.png')
	plt.clf()
	# plt.show()
	

def main():

	# Get task name and results file.
	if len(sys.argv) < 2:
		print "Usage: python plotter.py <task_name> (e.g taxi, upworld)"
		quit()
	taskName = sys.argv[1]
	resultsFile = loadFile(taskName)

	# Get results.
	epsilons, numStates, performances = parseResultsFile(resultsFile)

	# Plot.
	plot(epsilons, numStates, "Num States", taskName)
	plot(epsilons, performances, "Abstract vs. Ground Init State Value", taskName)

if __name__ == "__main__":
	main()
