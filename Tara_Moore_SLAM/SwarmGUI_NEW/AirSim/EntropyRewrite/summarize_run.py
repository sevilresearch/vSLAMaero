import pandas as pd
import numpy as np
from pathlib import Path

#csv_path = Path("C:\Users\taram\source\repos\SwarmGUI_NEW\AirSim\EntropyRewrite\data-files\entropySave.csv")  # you already create this
#df = pd.read_csv(csv_path, sep=r"\s+", engine="python")  # your file is space-delimited
df = pd.read_csv( r"C:\Users\taram\source\repos\SwarmGUI_NEW\AirSim\EntropyRewrite\data-files\entropySave.csv",
    sep=r"\s+",
    engine="python")

print("Num rows:", len(df))
# Infer number of drones from the header: ["X1","Y1","EntropyDrone1", ...]
cols = df.columns.tolist()
N = sum(1 for c in cols if c.startswith("X"))

dist_per_drone = []
for i in range(1, N+1):
    x = df[f"X{i}"].to_numpy()
    y = df[f"Y{i}"].to_numpy()
    # drop NaNs if any
    m = ~(np.isnan(x) | np.isnan(y))
    x, y = x[m], y[m]
    # pairwise step distances
    steps = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    dist_per_drone.append(steps.sum())

avg_distance = float(np.mean(dist_per_drone))

def path_length(x, y):
    x = x.to_numpy(float)
    y = y.to_numpy(float)
    steps = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    return steps.sum()

print("\nManual check for Drone1 path length:")
print(path_length(df["X1"], df["Y1"]))

print("Per-drone distances (m):", [round(d,2) for d in dist_per_drone])
print("Average distance (m):", round(avg_distance, 2))