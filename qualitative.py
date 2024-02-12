import numpy as np 
import json
from tqdm import tqdm
from scipy.spatial.distance import euclidean
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


os2 = []
with open("/run/media/emanuele/Volume/Dati/Università/tesi triennale/vision_evaluation_tools/data/traj_os2.json", "r") as f:
    os2 = json.load(f)

os3 = []
with open("/run/media/emanuele/Volume/Dati/Università/tesi triennale/vision_evaluation_tools/data/traj_os3.json", "r") as f:
    os3 = json.load(f)

def compare_trajectories(traj1: list, traj2: list, attribute: str, attribute2:str, tolerance: float) -> list:
    corresponding_points = []
    corresponding_orientation = []
    corresponding_points.append((traj1[0]["position"], traj2[0]["position"]))
    corresponding_orientation.append((traj1[0]["orientation"], traj2[0]["orientation"]))
    del traj1[0]
    del traj2[0]
    for point1 in traj1:
        for point2 in traj2:
            if point1[attribute] == point2[attribute] and abs(point1[attribute2] - point2[attribute2]) <= tolerance: 
                corresponding_points.append((point1["position"], point2["position"]))
                corresponding_orientation.append((point1["orientation"], point2["orientation"]))
    return corresponding_points, corresponding_orientation

print("length os2: "+str(len(os2))+" length os3: "+str(len(os3)))
traj, _ = compare_trajectories(os2, os3, "timestamp", "nsec", 100)
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
print(len(x1))
# Plot the two trajectories as 3D lines
ax.plot(x1, y1, z1, "r-", label="Orb slam 2 trajectory")
ax.plot(x2, y2, z2, "b-", label="Orb slam 3 trajectory")

# Add a legend
ax.legend()

# Show the plot
plt.show()

