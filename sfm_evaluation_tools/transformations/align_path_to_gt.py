# -*- coding: utf-8 -*-
""" Align Poses to a Groud Truth

This scripts allows to align the poses of a sensor or slam in CSV format to a GT extracted from AliceVision (so not a real GT, but much more accurate than the other).

In the same folder as the script will be created the GT file and the Poses file in the KITTI format. 
These ".txt" files can be used in the KITTI test suite modified by me .
"""
import numpy as np
import json
import sys
import math
import matplotlib.pyplot as plt
import sophuspy as sp
from collections import Counter
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from matplotlib.patches import Ellipse
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from helpers import DEBUG

def subsample(signal: np.ndarray, new_length: int):
    if new_length >= len(signal):
        return signal
    indices = np.round(np.linspace(0, len(signal) - 1, new_length)).astype(int)
    if DEBUG>=1: print(f"indices: {indices}")
    return signal[indices]


def horn_method(p1: np.ndarray, p2: np.ndarray):
    """
    Applies Horn's method (quaternion-based) to find the optimal rigid
    transformation between two sets of corresponding 3D points.

    Args:
        p1 (np.ndarray): The first set of points, shape (N, 3).
        p2 (np.ndarray): The second set of corresponding points, shape (N, 3).

    Returns:
        np.ndarray: The 3x3 rotation matrix.
        np.ndarray: The 3x1 translation vector.
    """
    # 1. Calculate the centroids of both point clouds
    centroid1 = np.mean(p1, axis=0)
    centroid2 = np.mean(p2, axis=0)

    # 2. Center the points by subtracting the centroids
    p1_centered = p1 - centroid1
    p2_centered = p2 - centroid2

    # 3. Calculate the covariance matrix H
    H = np.dot(p1_centered.T, p2_centered)
    
    # Unpack the elements of H for the symmetric matrix N
    Sxx, Sxy, Sxz = H[0,0], H[0,1], H[0,2]
    Syx, Syy, Syz = H[1,0], H[1,1], H[1,2]
    Szx, Szy, Szz = H[2,0], H[2,1], H[2,2]

    # 4. Form the 4x4 symmetric matrix N
    N = np.array([
        [Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx],
        [Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz],
        [Szx - Sxz, Sxy + Syx, Syy - Sxx - Szz, Syz + Szy],
        [Sxy - Syx, Szx + Sxz, Syz + Szy, Szz - Sxx - Syy]
    ])

    # 5. Find the eigenvector corresponding to the largest eigenvalue of N
    eigenvalues, eigenvectors = np.linalg.eig(N)
    max_eigenvalue_index = np.argmax(eigenvalues)
    q = eigenvectors[:, max_eigenvalue_index] # This is our optimal quaternion [q0, q1, q2, q3]

    # 6. Convert the quaternion to a rotation matrix
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
    
    R = np.array([
        [q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2]
    ])

    # 7. Calculate the translation vector
    t = centroid2.T - np.dot(R, centroid1.T)

    return R, t


def register_point_clouds(source_points: np.ndarray, target_points: np.ndarray):
    """
    A complete registration pipeline for two point clouds of potentially
    different lengths.

    Args:
        source_points (np.ndarray): The point cloud to be aligned.
        target_points (np.ndarray): The reference point cloud.

    Returns:
        np.ndarray: The 3x3 rotation matrix.
        np.ndarray: The 3x1 translation vector.
    """
    # Step 1: Find corresponding points using nearest neighbors
    p1, p2 = find_nearest_neighbors(source_points, target_points)
    
    # Step 2: Apply Horn's method to the corresponding points
    R, t = horn_method(p1, p2)
    
    return R, t

def align(model,data):
    """Align two trajectories using the method of Horn (closed-form).
    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)
    
    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)
    """
    np.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)
    
    W = np.zeros( (3,3) )
    for column in range(data.shape[1]):
        W += np.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    
    U,d,Vh = np.linalg.linalg.svd(W.transpose())
    S = np.matrix(np.identity( 3 ))
    if(np.linalg.det(U) * np.linalg.det(Vh)<0): S[2,2] = -1
    rot = U*S*Vh

    rotmodel = rot*model_zerocentered 
    dots = 0.0
    norms = 0.0

    for column in range(data_zerocentered.shape[1]):
        dots += np.dot(data_zerocentered[:,column].transpose(),rotmodel[:,column])
        normi = np.linalg.norm(model_zerocentered[:,column])
        norms += normi*normi

    s = dots.item()/norms.item()    
    
    transGT = data.mean(1) - s*rot * model.mean(1)
    trans = data.mean(1) - rot * model.mean(1)

    model_alignedGT = s*rot * model + transGT
    model_aligned = rot * model + trans

    alignment_errorGT = model_alignedGT - data
    alignment_error = model_aligned - data

    trans_errorGT = np.sqrt(np.sum(np.multiply(alignment_errorGT,alignment_errorGT),0)).A[0]
    trans_error = np.sqrt(np.sum(np.multiply(alignment_error,alignment_error),0)).A[0]
        
    return rot,transGT,trans_errorGT,trans,trans_error, s

def extract_path(sfm):
    path = sfm["views"][0]["path"]
    x = path.split("/")
    return path[0 : len(path) - len(x[len(x)-1])]

def plot_traj(ax,traj,style,color,label):
    """ Plot a trajectory using matplotlib. 
    ax -- the plot
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend
    """
    ax.scatter(traj[0],traj[2],traj[1], style,color=color,label=label)

def save_sfm_like_kitti(poses, filename):
    with open (filename, "w") as f:
        for i in range(len(poses)):
            line = f"{poses[i].A[0].astype(list)[0]} {poses[i].A[0].astype(list)[1]} {poses[i].A[0].astype(list)[2]} {poses[i].A[0].astype(list)[3]} {poses[i].A[1].astype(list)[0]} {poses[i].A[1].astype(list)[1]} {poses[i].A[1].astype(list)[2]} {poses[i].A[1].astype(list)[3]} {poses[i].A[2].astype(list)[0]} {poses[i].A[2].astype(list)[1]} {poses[i].A[2].astype(list)[2]} {poses[i].A[2].astype(list)[3]} "
            f.write(line + "\n")

def save_sfm_like_kitti_p(poses, filename):
    with open (filename, "w") as f:
        for p in poses:
            rot = p.rotationMatrix()
            t = p.translation()
            f.write( f"{rot[0][0]} {rot[0][1]}  {rot[0][2]} {t[0]} {rot[1][0]} {rot[1][1]}  {rot[1][2]} {t[1]} {rot[2][0]} {rot[2][1]}  {rot[2][2]} {t[2]} \n")

def plot_slam_poses(poses_kitti, ax, style='-', color='black', label='SLAM Poses'):
    # Extract translations from each 4x4 pose matrix
    if isinstance(poses_kitti[0], sp.SE3):
        # If poses are in Sophus SE3 format
        xs = [pose.translation()[0] for pose in poses_kitti]
        ys = [pose.translation()[1] for pose in poses_kitti]
        zs = [pose.translation()[2] for pose in poses_kitti]
    else:
        xs = [pose[0,3] for pose  in poses_kitti]
        ys = [pose[1,3] for pose  in poses_kitti]
        zs = [pose[2,3] for pose  in poses_kitti]

    zmin, zmax = ax.get_zlim()
    xmin, xmax = ax.get_xlim()
    ymin, ymax = ax.get_ylim()
    ax.set_ylim(ymin - 0.5, ymax + 0.5)
    ax.set_zlim(zmin - 0.5, zmax + 0.5)
    ax.set_xlim(xmin - 0.5, xmax + 0.5)
        
    # Note: using x, z, y ordering (if that’s your convention)
    ax.plot(xs, zs, ys, style, color=color, label=label)

# Usage example:

if __name__ == "__main__":
    assert len(sys.argv) in [3, 4], "usage: align_slam_to_gt.py path_to_csv path_to_sfm_file [True/False suppress Graphs]"

    suppress_graph = False
    if len(sys.argv) == 4: 
        if sys.argv[3] in ['false', 'n', '0', 'False', 'no', 'No', 'NO']: suppress_graph = False
        elif sys.argv[3] in ['true', 'y', '1', 'True', 'yes', 'Yes', 'YES']: suppress_graph = True

    print(suppress_graph)
    # Extracting SLAM poses
    slam_positions = []
    slam_poses = []
    number_of_poses = 0
    with open(sys.argv[1], "r") as f:
        for line in f:
            if "timestamp" not in line:
                number_of_poses += 1
                vec = line.split(",")
                slam_positions.append([float(vec[1]), float(vec[2]), float(vec[3])])
                slam_poses.append([[float(vec[4]), float(vec[5]), float(vec[6]),float(vec[1])],
                                [float(vec[7]), float(vec[8]), float(vec[9]),float(vec[2])],
                                [float(vec[10]), float(vec[11]), float(vec[12]),float(vec[3])],
                                [0.0, 0.0, 0.0, 1.0]])
            
    # Extracting GT (SFM) Poses
    with open(sys.argv[2], "r") as f:
        x = json.loads(f.read())
        assert x is not None, "GT file not accessible or not found"
        
    poses = x["poses"]
    images = x["views"]
    poses_id = []
    sfm_file_path = extract_path(x)

    gt_positions = []
    counter = 0
    gt_rotations = []

    number_of_poses = 604-269 #int(sys.argv[3])
    assert isinstance(number_of_poses, int) == True, "Please, insert correct arguments"
    first_image_numb = 269
    for i in range(first_image_numb, first_image_numb+number_of_poses):
        for im in images:
            if im["path"] == sfm_file_path + "frame%04i.jpg" % i:
                poseId = im["poseId"]
                poses_id.append(poseId)
                for j in range(len(poses)):
                    if poseId == poses[j]["poseId"]:
                        counter+=1
                        gt_positions.append([float(poses[j]["pose"]["transform"]["center"][0]), float(poses[j]["pose"]["transform"]["center"][1]), float(poses[j]["pose"]["transform"]["center"][2])])
                        gt_rotations.append([[float(poses[j]["pose"]["transform"]["rotation"][0]), float(poses[j]["pose"]["transform"]["rotation"][3]), float(poses[j]["pose"]["transform"]["rotation"][6])],
                                                [float(poses[j]["pose"]["transform"]["rotation"][1]), float(poses[j]["pose"]["transform"]["rotation"][4]), float(poses[j]["pose"]["transform"]["rotation"][7])],
                                                [float(poses[j]["pose"]["transform"]["rotation"][2]), float(poses[j]["pose"]["transform"]["rotation"][5]), float(poses[j]["pose"]["transform"]["rotation"][8])]])
                        break

    assert len(gt_positions) > 0, "Ground Truth data is empty"

    print(len(gt_positions))
    print("poses:" + str(len(poses)))
    print("views:" + str(len(images)))

    # using numpy matrices and transpose.
    gt_positions_t = np.matrix(gt_positions).transpose()
    gt_subsampled_pos = np.matrix(subsample(np.array(gt_positions), len(slam_positions))).transpose()
    slam_positions_t = np.matrix(slam_positions).transpose()

    rot,transGT,trans_errorGT,trans,trans_error, scale = align(gt_subsampled_pos,slam_positions_t)

    gt_positions_scaled = scale * gt_subsampled_pos 

    
    if not suppress_graph:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection ='3d')
        plot_traj(ax,slam_positions_t.A,'-',"black","SLAM System")
        plot_traj(ax,gt_positions_scaled.A,'-',"blue","Ground Truth")
        ax.legend()
        plt.show()
    print("alignment tranformations: ")
    print(f"translation vector: {trans}")
    print(f" rotation matrix: {rot}")
    print(f"scale: {scale} \n")
    print("compared_pose_pairs %d pairs"%(len(trans_error)))
    print("absolute_translational_error.rmse %f m"%np.sqrt(np.dot(trans_error,trans_error) / len(trans_error)))
    print("absolute_translational_error.mean %f m"%np.mean(trans_error))
    print("absolute_translational_error.median %f m"%np.median(trans_error))
    print("absolute_translational_error.std %f m"%np.std(trans_error))
    print( "absolute_translational_error.min %f m"%np.min(trans_error))
    print( "absolute_translational_error.max %f m"%np.max(trans_error))
    print( "max idx: %i" %np.argmax(trans_error))

    rot,transGT,trans_errorGT,trans,trans_error, scale =  align(slam_positions_t,gt_positions_scaled)
    print()
    print("final alignment after scaling")
    print("------------------------------------------")
    print("compared_pose_pairs %d pairs"%(len(trans_error)))
    print("absolute_translational_error.rmse %f m"%np.sqrt(np.dot(trans_error,trans_error) / len(trans_error)))
    print("absolute_translational_error.mean %f m"%np.mean(trans_error))
    print("absolute_translational_error.median %f m"%np.median(trans_error))
    print("absolute_translational_error.std %f m"%np.std(trans_error))
    print( "absolute_translational_error.min %f m"%np.min(trans_error))
    print( "absolute_translational_error.max %f m"%np.max(trans_error))
    print( "max idx: %i" %np.argmax(trans_error))

    transformation_matrix = [[rot.A[0][0], rot.A[0][1], rot.A[0][2],trans.A[0][0]],
                        [rot.A[1][0],rot.A[1][1], rot.A[1][2],trans.A[1][0]],
                        [rot.A[2][0], rot.A[2][1], rot.A[2][2], trans.A[2][0]],
                        [0.0, 0.0, 0.0, 1.0]]
    

    # Alignment of the poses to a reference system centered in the first pose of the sequence
    # GT aligment
    final_gt = [sp.SE3(np.matrix(r).transpose(), t) for r,t in zip (gt_rotations, gt_positions_scaled.transpose().A)]
    final_gt_kitti = [final_gt[0].inverse() * s for s in final_gt]
    
    # SLAM poses Aligment
    aligned_slam_poses = [np.matmul(np.matrix(transformation_matrix), np.matrix(pose)) for pose in slam_poses]
    inv_first_pose = np.linalg.inv(aligned_slam_poses[0])
    slam_poses_kitti = [np.matmul(inv_first_pose, s) for s in aligned_slam_poses]

    save_sfm_like_kitti_p(final_gt_kitti,"gt_poses.txt")
    save_sfm_like_kitti(slam_poses_kitti, "slam_poses.txt")


    #More plots
    if not suppress_graph:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection ='3d')
        final_plot_slam_positions = rot * slam_positions_t + trans
        plot_traj(ax,final_plot_slam_positions.A,'-',"black","SLAM System")

        plot_traj(ax,gt_positions_scaled.A,'-',"blue","Ground Truth")
        ax.set_xlim([-1,1])
        ax.set_ylim([-1,1])
        zmin, zmax = ax.get_zlim()
        ax.set_zlim(zmin - 1, zmax + 1)

        plt.show()
        # Ensure x and y axes share the same ran
        
        label="difference"
        # for (x1,y1,z1),(x2,y2,z2) in zip(slam_positions_t.transpose().A,gt_positions_scaled.transpose().A):
        #     ax.plot([x1,x2],[y1,y2],'-',color="red",label=label)
        #     label=""

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        plot_slam_poses(slam_poses_kitti, ax, color='black', label='GNSS poses')
        plot_slam_poses(final_gt_kitti, ax, color='red', label='SfM poses')
        ax.legend()
        plt.show()



