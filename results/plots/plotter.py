# ===================================================================
# =========== Authors: David Abel and D Ellis Hershkowits ===========
# == Summary: Plotting tool for Approximate State Abstraction work ==
# ===================================================================

# Python imports.
import matplotlib.pyplot as plt
import matplotlib
import numpy as np

yValueTypes = ["Num States", "Compressed State Value Diff."]
yValueType = yValueType[0] # CHANGE THIS TO 1 IF OTHER PLOT TYPE.

# Epsilon values.
maxEps = 20
increment = 2
minEps = 0
epsilons = [x / 10.0 for x in range(minEps, maxEps + increment, increment)]

# Note: Insert results into this list.
yData = [7, 13, 12, 4, 6, 7, 4]

# Plot.
plt.plot(xData, yData, marker="o", color="black")

font = {'family' : 'normal',
        'size'   : 15}

matplotlib.rc('font', **font)
plt.xlim([minEps, maxEps])
plt.ylim([0, max(yData) + increment*2])
plt.xlabel('Epsilon')
plt.ylabel(yValueType)
plt.title('Epsilon vs. ' + yValueType)
plt.grid(True)
plt.savefig('epsilon_vs_' + yValueType.lower() + '.png')
plt.show()