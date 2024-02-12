import numpy as np
import json
from numpy.lib.function_base import append
from tqdm import tqdm
from scipy.spatial.distance import euclidean
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import utils
import open3d as o3d
import copy
##Based on Arun et al., 1987

############### ICP params ##################
threshold = 0.02
trans_init = np.asarray([[1.0, 0.0, 0.0, 1.0],
                         [-0.0, 1.0, -0.0, 0.0],
                         [0.0, 0.0, 1.0, 0.0], 
                         [0.0, 0.0, 0.0, 1.0]])


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size, source, target):
    print(":: Load two point clouds and disturb initial pose.")
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result

print(sys.argv)
os2 = []
#with open("data/traj_os2.json", "r") as f:
with open(sys.argv[1], "r") as f:
    os2 = json.load(f)

os3 = []
#with open("data/traj_os3.json", "r") as f:
with open(sys.argv[2], "r") as f:
    os3 = json.load(f)
traj, _ = utils.compare_trajectories(os2, os3,"timestamp", "nsec", 9999)
os3_traj = []
os2_traj = []
for i in range(len(os2)):
    os2_traj.append(os2[i]["position"])

for i in range(len(os3)):
    os3_traj.append(os3[i]["position"])

os3_traj = utils.get_values(os3_traj, len(os2_traj))
#print(os3_traj)

#print(str(len(os3_traj)) + " "+str(len(os2_traj)) + " demo: "+str(os3_traj[0]) )
#sys.exit()
#Writing points with rows as the coordinates
# p1_t = np.array([p1["position"] for p1 in traj[:,0]])
# p2_t = np.array([p1["position"] for p1 in traj[:,1]]) #Approx transformation is 90 degree rot over x-axis and +1 in Z axis

p1_t = np.array(os2_traj)
p2_t = np.array(os3_traj)
"""print(p1_t[1])
point_set_os2 = o3d.geometry.PointCloud()
point_set_os2.points = o3d.utility.Vector3dVector(p1_t)

point_set_os3 = o3d.geometry.PointCloud()
point_set_os3.points = o3d.utility.Vector3dVector(p2_t)
print(np.asarray(point_set_os3.points))
#print(point_set_os2)
draw_registration_result(point_set_os2, point_set_os3, np.identity(4))
voxel_size = 0.001
source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, point_set_os2, point_set_os3)

result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
print(result_ransac)
draw_registration_result(source_down, target_down, result_ransac.transformation)"""

#result_icp = refine_registration(source, target, source_fpfh, target_fpfh, voxel_size)
#print(result_icp)
#draw_registration_result(source, target, result_icp.transformation)

# syntax for 3-D projection
#Take transpose as columns should be the points
#sys.exit("exit on purpose")
p1 = p1_t.transpose()
print(p1)
p2 = p2_t.transpose()
fig = plt.figure()
ax = plt.axes(projection ='3d')
plt.xlim(-1, 20)
plt.ylim(-1, 5)
plt.gca().set_aspect('equal', adjustable='box')

ax.scatter(p1[0], p1[2], p1[1], 'red')
ax.scatter(p2[0], p2[2], p2[1], 'blue') 
plt.show()
#Calculate centroids
p1_c = np.mean(p1, axis = 1).reshape((-1,1)) #If you don't put reshape then the outcome is 1D with no rows/colums and is interpeted as rowvector in next minus operation, while it should be a column vector
p2_c = np.mean(p2, axis = 1).reshape((-1,1))

#Subtract centroids
q1 = p1-p1_c
q2 = p2-p2_c

#Calculate covariance matrix
H=np.matmul(q1,q2.transpose())

#Calculate singular value decomposition (SVD)
U, X, V_t = np.linalg.svd(H) #the SVD of linalg gives you Vt

#Calculate rotation matrix
R = np.matmul(V_t.transpose(),U.transpose())

assert np.allclose(np.linalg.det(R), 1.0), "Rotation matrix of N-point registration not 1, see paper Arun et al."

#Calculate translation matrix
T = p2_c - np.matmul(R,p1_c)

#Check result
result =  np.matmul(R,p1)
print(T)
fig = plt.figure()
ax = plt.axes(projection ='3d')
ax.scatter(result[0], result[1], result[2], 'red')
ax.scatter(p2[0], p2[1], result[2], 'blue') 
plt.show()
print(R)
if np.allclose(result,p2):
    print("transformation is correct!")
else:
    print("transformation is wrong...")
    
    
