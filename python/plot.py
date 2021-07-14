import csv
import glob
import matplotlib.pyplot as plt
import numpy as np
import os
import time

# Set up
print("Hello guys, calling from the Python side. Waiting for the logging to be finished...")
print("...waited long enough, let's plot!")

# Plotting
list_of_files = glob.glob(os.getenv("HOME") + "/.ros/fg_filtering*.csv")
print(list_of_files)
latestCSVFile = max(list_of_files, key=os.path.getctime)
print("Latest CSV file is: " + latestCSVFile)

with open(latestCSVFile, newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in reader:
        rowNumpy = np.asarray(row)
        rowName = rowNumpy[0]
        print("Plotting data for field " + "'" + rowName + "'")

        fig, ax = plt.subplots()
        ax.plot([float(i) for i in rowNumpy[1:-1]])

        ax.set(xlabel='Steps', ylabel=rowName, title=rowName)
        ax.grid()

        fig.savefig(os.getenv("HOME") + "/.ros/plots/" + rowName + ".png")