# -*- coding: utf-8 -*-
""" Align Poses to a Groud Truth

This scripts allows to align the poses of a SLAM System to a GT extracted from AliceVision (so not a real GT, but much more accurate than a SLAM system).

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
    for column in range(model.shape[1]):
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

    s = float(dots/norms)    
    
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

if __name__ == "__main__":
    assert len(sys.argv) in [4, 5], "usage: align_slam_to_gt.py path_to_slam_file path_to_sfm_file number_of_poses [True/False suppress Graphs]"

    suppress_graph = False
    if len(sys.argv) == 5:
        suppress_graph = bool(sys.argv[4])
        assert isinstance(suppress_graph, bool) == True, "Please, insert correct arguments"

    # Extracting SLAM poses
    slam_positions = []
    slam_poses = []
    with open(sys.argv[1], "r") as f:
        for line in f:
            vec = line.split(" ")
            slam_positions.append([float(vec[3]), float(vec[7]), float(vec[11])])
            slam_poses.append([[float(vec[0]), float(vec[1]), float(vec[2]),float(vec[3])],
                            [float(vec[4]), float(vec[5]), float(vec[6]),float(vec[7])],
                            [float(vec[8]), float(vec[9]), float(vec[10]),float(vec[11])],
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

    number_of_poses = int(sys.argv[3])
    assert isinstance(number_of_poses, int) == True, "Please, insert correct arguments"

    for i in range(0, number_of_poses):
        for im in images:
            if im["path"] == sfm_file_path + "image_"+str(i+1)+".jpg":
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

    print(gt_positions)
    assert len(gt_positions) > 0, "Ground Truth data is empty"

    print(len(gt_positions))
    print("poses:" + str(len(poses)))
    print("views:" + str(len(images)))

    # using numpy matrices and transpose.
    gt_positions_t = np.matrix(gt_positions).transpose()
    slam_positions_t = np.matrix(slam_positions).transpose()

    rot,transGT,trans_errorGT,trans,trans_error, scale = align(gt_positions_t,slam_positions_t)

    gt_positions_scaled = scale * gt_positions_t 

    
    if not suppress_graph:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection ='3d')
        plot_traj(ax,slam_positions_t.A,'-',"black","SLAM System")
        plot_traj(ax,gt_positions_scaled.A,'-',"blue","Ground Truth")
        ax.legend()
        plt.show()

    print(trans)
    print(rot)
    print("compared_pose_pairs %d pairs"%(len(trans_error)))
    print("absolute_translational_error.rmse %f m"%np.sqrt(np.dot(trans_error,trans_error) / len(trans_error)))
    print("absolute_translational_error.mean %f m"%np.mean(trans_error))
    print("absolute_translational_error.median %f m"%np.median(trans_error))
    print("absolute_translational_error.std %f m"%np.std(trans_error))
    print( "absolute_translational_error.min %f m"%np.min(trans_error))
    print( "absolute_translational_error.max %f m"%np.max(trans_error))
    print( "max idx: %i" %np.argmax(trans_error))
    print(f"scale: {scale}")


    rot,transGT,trans_errorGT,trans,trans_error, scale =  align(slam_positions_t,gt_positions_scaled)
    print()
    print("final alignment")
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
        plt.show()

        label="difference"
        for (x1,y1,z1),(x2,y2,z2) in zip(slam_positions_t.transpose().A,gt_positions_scaled.transpose().A):
            ax.plot([x1,x2],[y1,y2],'-',color="red",label=label)
            label=""
