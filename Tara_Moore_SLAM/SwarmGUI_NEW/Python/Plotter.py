# -*- coding: utf-8 -*-
"""
Created on Thu May 28 16:27:51 2020

@author: lukef
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# ----------------------------------------------------------
# 1. Load CSV
# ----------------------------------------------------------
data = pd.read_csv(
    r"C:\Users\taram\source\repos\SwarmGUI_NEW\AirSim\EntropyRewrite\data-files\entropySave.csv",
    sep=r"\s+",
    engine="python"
)

# ----------------------------------------------------------
# 2. Helper: find “teleport” index based on biggest step
# ----------------------------------------------------------
def find_cut_index(x, y, teleport_thresh=30.0):
    """
    x, y: 1D numpy arrays of positions.
    teleport_thresh: distance in meters between consecutive samples
                     that we consider a 'teleport' instead of normal motion.

    Returns: index to keep data up to (i.e., slice x[:cut_idx]).
             If no big jump, returns len(x).
    """
    if len(x) < 2:
        return len(x)

    dx = np.diff(x)
    dy = np.diff(y)
    dist = np.sqrt(dx**2 + dy**2)

    # Index of biggest step
    j = np.argmax(dist)
    max_step = dist[j]

    print(f"[trim] max consecutive step = {max_step:.2f} m")

    if max_step > teleport_thresh:
        cut_idx = j + 1  # keep up to the point *before* the jump
        print(f"[trim] Trimming at index {cut_idx} due to teleport.")
        return cut_idx
    else:
        print("[trim] No large teleport; using full path.")
        return len(x)

# ----------------------------------------------------------
# 3. Convert columns to numpy, detect cut, and trim ALL arrays
# ----------------------------------------------------------
x1Full = data['X1'].to_numpy(dtype=float)
y1Full = data['Y1'].to_numpy(dtype=float)
x2Full = data['X2'].to_numpy(dtype=float)
y2Full = data['Y2'].to_numpy(dtype=float)
x3Full = data['X3'].to_numpy(dtype=float)
y3Full = data['Y3'].to_numpy(dtype=float)
e1Full = data['EntropyDrone1'].to_numpy(dtype=float)
e2Full = data['EntropyDrone2'].to_numpy(dtype=float)
e3Full = data['EntropyDrone3'].to_numpy(dtype=float)
timeFull = data['TimeDifference'].to_numpy(dtype=float)

# Find a cut index for EACH UAV, then use the earliest one
cut1 = find_cut_index(x1Full, y1Full, teleport_thresh=30.0)
cut2 = find_cut_index(x2Full, y2Full, teleport_thresh=30.0)
cut3 = find_cut_index(x3Full, y3Full, teleport_thresh=30.0)
cut_idx = min(cut1, cut2, cut3)

print(f"[trim] Using cut_idx = {cut_idx} (out of {len(x1Full)})")

x1Full = x1Full[:cut_idx]
y1Full = y1Full[:cut_idx]
x2Full = x2Full[:cut_idx]
y2Full = y2Full[:cut_idx]
x3Full = x3Full[:cut_idx]
y3Full = y3Full[:cut_idx]
e1Full = e1Full[:cut_idx]
e2Full = e2Full[:cut_idx]
e3Full = e3Full[:cut_idx]
timeFull = timeFull[:cut_idx]

num_frames = len(x1Full)

# ----------------------------------------------------------
# 4. Set up animation variables and plotting
# ----------------------------------------------------------
x1, y1, x2, y2, x3, y3 = [], [], [], [], [], []
x1b, y1b, x2b, y2b, x3b, y3b = [], [], [], [], [], []

e1, e2, e3, time = [], [], [], []

plt.style.use('fivethirtyeight')
fig = plt.figure(figsize=(15, 10))
ax1 = fig.add_subplot(2, 1, 1)
ax2 = fig.add_subplot(2, 1, 2)

ax1.set(xlim=(-110, 160), ylim=(20, 200))
ax2.set(xlim=(0, timeFull[-1] if len(timeFull) > 0 else 1), ylim=(min(e1Full.min(), e2Full.min(), e3Full.min()) - 0.1,
                                                                   max(e1Full.max(), e2Full.max(), e3Full.max()) + 0.1))

behindby = 1

def animate(frame_idx):
    if frame_idx >= num_frames:
        return

    # accumulate new point
    x1.append(x1Full[frame_idx]); y1.append(y1Full[frame_idx])
    x2.append(x2Full[frame_idx]); y2.append(y2Full[frame_idx])
    x3.append(x3Full[frame_idx]); y3.append(y3Full[frame_idx])

    if frame_idx > behindby:
        x1b.append(x1Full[frame_idx-behindby]); y1b.append(y1Full[frame_idx-behindby])
        x2b.append(x2Full[frame_idx-behindby]); y2b.append(y2Full[frame_idx-behindby])
        x3b.append(x3Full[frame_idx-behindby]); y3b.append(y3Full[frame_idx-behindby])

    e1.append(e1Full[frame_idx])
    e2.append(e2Full[frame_idx])
    e3.append(e3Full[frame_idx])
    time.append(timeFull[frame_idx])

    ax1.cla()
    ax1.plot(x1b, y1b, linestyle='-', color='c')
    ax1.plot(x2b, y2b, linestyle='-', color='r')
    ax1.plot(x3b, y3b, linestyle='-', color='#fcbe03')
    ax1.set_xlim(-110, 160)
    ax1.set_ylim(20, 200)
    ax1.legend(["UAV 1", "UAV 2", "UAV 3"])
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title("Positions")

    ax2.cla()
    ax2.plot(time, e1, linestyle='-', color='c')
    ax2.plot(time, e2, linestyle='--', color='r')
    ax2.plot(time, e3, linestyle='--', color='#fcbe03')
    ax2.legend(["Entropy 1", "Entropy 2", "Entropy 3"])
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Entropy")
    ax2.set_title("Entropy for each UAV")

    plt.tight_layout()

ani = FuncAnimation(fig, animate, frames=num_frames, interval=50, repeat=False)

writerFull = PillowWriter(fps=30)
ani.save(filename='plotting.gif', writer=writerFull)
plt.show()

print("num_frames =", num_frames)
print("len(x1Full) =", len(x1Full))
