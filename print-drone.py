#!/bin/python

import matplotlib.pyplot as plt
import sys


data =[[float(x.strip()) for x in open(file)] for file in sys.argv[1:]]

for d in data:
    plt.plot(d)

plt.show()