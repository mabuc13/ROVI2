# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import numpy.polynomial.polynomial as poly
from math import acos, asin, sqrt, sin, cos, pi, atan
import argparse
import signal
import sys
from scipy.stats import norm


def signal_handler(signal, frame):
    #print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

parser = argparse.ArgumentParser()
parser.add_argument('--n', help='the name/path to the file to analyse')
args = parser.parse_args()



f = open (args.n, "r")

#Variables for plotting
showPlot = True

timeData = []
ampData = []
zData = []
zeroVec = []

for line in f:
    line=line[:-1]
    csv = line.split(' ')
    time = float(csv[0])
    z_error = float(csv[1])
    z_value = float(csv[2])

    zeroVec.append(0)   
    ampData.append(z_error)
    timeData.append(time)
    zData.append(z_value)


plt.plot(timeData, ampData, label = "Z error", linestyle="-",marker=".")
plt.plot(timeData, zData, label = "Current Z", linestyle="-",marker=".")
plt.plot(timeData, zeroVec, label = "Current Z", linestyle="-",marker=".")

plt.show()

