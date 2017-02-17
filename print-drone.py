#!/bin/python

import matplotlib.pyplot as plt
import sys
import pandas as pd

df = pd.read_csv(open("data"), sep=" ")


plt.figure(figsize=(13,10))
for arg in sys.argv[1:]:
    plt.plot(df["t"], df[arg], label=arg)
    plt.xlabel("t")

plt.legend(loc=4)
plt.grid(True)

plt.show()
