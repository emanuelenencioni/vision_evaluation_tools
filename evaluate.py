import numpy as np 
import json
from tqdm import tqdm
from scipy.spatial.distance import euclidean
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import utils
print(sys.argv)
os2 = []
with open(sys.argv[1], "r") as f:
    os2 = json.load(f)

os3 = []
with open(sys.argv[2], "r") as f:
    os3 = json.load(f)

print("length os2: "+str(len(os2))+" length os3: "+str(len(os3)))
traj, _ = utils.compare_trajectories(os2, os3, "timestamp", "nsec", 1000)
print(traj[0])
os3_traj = []
os2_traj = []

for i in range(len(traj)):
    os3_traj = traj[i][1]
    os2_traj = traj[i][0]

distances = []
for i in tqdm(range(len(traj))):
    distance = euclidean(traj[i][0], traj[i][1])
    distances.append(distance)

print(sum(distances)/len(distances))
print("correlation coeff: "+str(np.corrcoef(os2_traj, os3_traj)[0, 1]))
points1 = os2
points2 = os3

# Extract the x, y, and z coordinates of the points
x1 = [point["position"][0] for point in points1]
y1 = [point["position"][1] for point in points1]
z1 = [point["position"][2] for point in points1]
x2 = [point["position"][0] for point in points2]
y2 = [point["position"][1] for point in points2]
z2 = [point["position"][2] for point in points2]

# Create a figure and an Axes3D object
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the two trajectories as 3D lines
ax.plot(x1, y1, z1, "r-", label="Orb slam 2 trajectory")
ax.plot(x2, y2, z2, "b-", label="Orb slam 3 trajectory")

# Add a legend
ax.legend()

# Show the plot
plt.show()