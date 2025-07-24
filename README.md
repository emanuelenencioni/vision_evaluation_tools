# vision_evaluation_tools
A toolkit for extrapolating and evaluating data of SLAM Systems. 

# Kitti Tools Dependeces
This repositories need `gnuplot`, `pdfcrop` installing respectively with:
```sudo apt-get install gnuplot```
```sudo apt-get install texlive-extra-utils```.

To build the kitti tool for evaluation, run `build.sh`.

# Python dependeces
For all the scripts, there is the `requirements.txt` file.
 
# Guide
### Extracting images
For extract images from any bag, see the corresponding folder. There is separated script for stereo and RGBD cameras.
#### StereoDownload odometry data set (velodyne laser data, 80 GB)
For stereo camera I have also included a `YAML` configuration file . This is useful to pass all the various parameters:
- **camera_model**
- **focal_length**
- **is_for_meshroom**: boolean, if it's true,  the information of camera model and focal length will be embedded in the EXIF data, and the output images will be jpeg. This results in a loss of quality, but unfortunatly, Meshrooms needs these EXIF details.
- **frame_interval**: an integer value indicating how many frame to skip. This is done to reduce the actual number of frame saved from the bag file.
- **left_camera_topic**: the ROS topic in the bag corresponding to the left camera
- **right_camera_topic**: same as above.
- **save_folder_name**
The yaml file need to specify all these parameters otherswise the script would not work.
Run the script, it will tell you what it need to work. 
#### RGBD
I didn't have time to test and I don't remember. I believe the script for saving images may not be functioning correctly when extrapolating depth images.

### SFM evaluation
1. So u need to play with meshroom (or alicevision if u like to play with the terminal). Extract the structure from motion. (should create a *.sfm that's basically a json file) 
2. Put .sfm file into `sfm_evaluation_tools/transformation` folder. 
3. get your slam trajectory in format like `sesto_lista_os3.txt` (as an example) and put it always inside `transformation` folder.
4. Run the  `align_slam_to_gt.py` script. The new file **gt_poses.txt** is the ground truth extracted from SFM in kitti format. It also generate a slam_poses.txt that should be the aligned version of your slam trajectory. It shouldn't be necessary. Both these trajectory are at the end aligned to the first pose (Accordingly to kitti dataset). 
5. copy your ground truth file into `cpp/odometry/poses` folder . Remember to rename the file differently than **gt_poses.txt**. 
6. your slam data '.txt' should be placed into `cpp/odometry/results/name_of_test/data/` folder. This file need to have the same name as the ground truth file.
7. go in `sfm_evaluation_tools/cpp`, use **build.sh** to build them.  U can then run **evaluate_odometry**, it needs:
    - **result_sha**, the actual path of the test, just the name of the folder `name_of_test`.
    - **sequence_file_name**, name of the ground truth file without extension.
8. look into the `cpp/odometry/results/name_of_test/` folder to see the results.
# TOD0's
- [ ] Add node for publish kitti's lidar data
- [ ] Clean the code
- [ ] Automatize most of the SFM points