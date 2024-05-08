import numpy as np
import matplotlib.pyplot as plt
import csv

colors = [
    'b',
    'r',
    'g',
    'c',
    'm',
    'y',
    'k'
]


# Plots an arbitrary trajectory from CSVs
def plot_things(ax, filenames, scale=1.0):
    for c_idx, filename in enumerate(filenames):
        color = colors[c_idx % len(colors)] # wraparound

        x = []
        y = []
        theta = []

        with open(filename, newline='') as f:
            reader = csv.reader(f)
            next(reader, None)  # skip the headers
            for row in reader:
                x.append(float(row[1]))
                y.append(float(row[2]))
                theta.append(float(row[3]))

        for i in range(len(x)):
            label = f"{filename} trajectory" if i==0 else None
            ax.arrow(x[i], y[i], scale*np.cos(theta[i]), scale*np.sin(theta[i]),
                     width=scale*0.05, color=color,
                     label=label
                     )

    ax.legend()
    ax.grid()
    ax.set_aspect('equal')
    ax.set_title('Trajectory Comparison')