import numpy as np

def compare_trajectories(traj1: list, traj2: list, attribute: str, attribute2:str, tolerance: float) -> list:
    corresponding_points = []
    corresponding_orientation = []
    time_diff =  traj2[0][attribute] - traj1[0][attribute]
    nsec_diff = traj2[0][attribute2] - traj1[0][attribute2]
    for point1 in traj1:
        for point2 in traj2:
            if point1[attribute] == (point2[attribute]- time_diff): #and abs(point1[attribute2] - point2[attribute2] - nsec_diff) <= tolerance: 
                corresponding_points.append((point1["position"], point2["position"]))
                corresponding_orientation.append((point1["orientation"], point2["orientation"]))
                break

    return corresponding_points, corresponding_orientation

def get_values(traj, num):
    final_traj = []
    final_traj.append(traj.pop(0))
    
    for i in range(num-1):
        gfg = np.random.uniform(0, len(traj)-1)
        final_traj.append(traj.pop(round(gfg)))

    return sorted(final_traj)

