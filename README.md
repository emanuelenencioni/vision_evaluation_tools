# vision_evaluation_tools
A toolkit for extrapolating and evaluating data of SLAM Systems. 

# Kitti Tools Dependeces
This repositories need `gnuplot`, `pdfcrop` installing respectively with:
```sudo apt-get install gnuplot```
```sudo apt-get install texlive-extra-utils```.

To build the kitti tool for evaluation, run `buils.sh`.

# Python dependeces
For all the scripts, there is the `requirements.txt` file.
 
# Guide
### Extracting images
For extract images from any bag, see the corresponding folder. There is separated script for stereo and RGBD cameras.
#### StereoDownload odometry data set (velodyne laser data, 80 GB)
For stereo camera I have also included a `YAML` configuration file . This file is useful to pass all the various parameters:
- **camera_model**
- **focal_length**
- **is_for_meshroom**: boolean, if it's true,  the information of camera model and focal length will be embedded in the EXIF data, and the output images will be jpeg. This results in a loss of quality, but unfortunatly, Meshrooms needs these EXIF details.
- **frame_interval**: an integer value indicating how many frame to skip. This is done to reduce the actual number of frame saved from the bag file.
- **left_camera_topic**: the ROS topic in the bag corresponding to the left camera
- **right_camera_topic**: same as above.
- **save_folder_name**
#### RGBD
I didn't have time to test and I don't remember. I believe the script for saving images may not be functioning correctly when extrapolating depth images.
### Using Meshroom
TODO
### SFM evaluation
TODO


# TOD0's
- [ ] Finish write README
- [ ] Add node for publish kitti's lidar data
- [ ] Clean the code
