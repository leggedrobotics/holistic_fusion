import csv
import glob
import matplotlib.pyplot as plt
import numpy as np
import os
import time
from datetime import datetime

# Set up
print("Hello guys, calling from the Python side. Waiting for the logging to be finished...")
print("...waited long enough, let's plot!")

# Plotting
list_of_files = glob.glob(os.getenv("HOME") + "/.ros/fg_filtering*.csv")
print(list_of_files)
latestCSVFile = max(list_of_files, key=os.path.getctime)
print("Latest CSV file is: " + latestCSVFile)

now = datetime.now()
# dd/mm/YY H:M:S
dtString = now.strftime("%d-%m-%Y_%H:%M:%S")

with open(latestCSVFile, newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in reader:
        rowNumpy = np.asarray(row)
        rowName = rowNumpy[0]
        rowNumpy = rowNumpy[1:]
        print("Plotting data for field " + "'" + rowName + "'")

        fig, ax = plt.subplots()
        ax.plot([float(i) for i in rowNumpy[(rowNumpy != "") & (rowNumpy != " ")]])

        ax.set(xlabel='Steps', ylabel=rowName, title=rowName)
        ax.grid()

        directoryName = os.getenv("HOME") + "/.ros/plots/" + dtString + "/"
        if not os.path.exists(directoryName):
            os.makedirs(directoryName)

        fig.savefig(directoryName + rowName.split("/")[-1] + ".pdf")