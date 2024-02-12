# -*- coding: utf-8 -*-
""" Save Stereo

This script permits to extract stereo images from a rosbag file. It can easily be adapted to different type of cameras.

This script needs a configuration file in *.yaml format. 
Use 'save_stereo_config.yaml' as a template.

In the same folder of the script will be create an images folder, containing the resulting images.
"""

import rosbag
import cv2
import cv_bridge
import numpy as np
import sys
import piexif
import os
import yaml
import progressbar

def save_image(topic, msg):
    """
    Function that transform a ros msg (image msg) to an opencv image object.
    """
    encoding = msg.encoding
    if encoding == 'bgra8':
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgra8")
        #cv2.imwrite("/home/slam/test/color1/image_" + str(counter) + ".png", cv_image
        image_buffer[topic].append((msg.header.stamp, cv_image))
    else:
        print("Image encoding not supported - "+str(encoding))




if len(sys.argv) != 3:
    sys.exit("use save_stereo.py rosbag_file config_file")

with open("save_stereo_config.yaml") as config_file:
    config = yaml.safe_load(config_file)
    if config == None:
        sys.exit("Error opening configuration file")

# for progress bar
widgets = [' [',
         progressbar.Timer(format= 'elapsed time: %(elapsed)s'),
         '] ',
           progressbar.Bar('█'),' (',
           progressbar.ETA(), ') ',
          ]




##### Configuration from yaml #####
frame_interval = config['frame_interval'] # take image every 'frame_interval' images.
camera_model = config['camera_model']
focal_length = config['focal_length']
is_for_meshroom = config['is_for_meshroom']
left_camera_topic = config['left_camera_topic']
right_camera_topic = config['right_camera_topic']
save_folder_name = config['save_folder_name']
left_camera = 0
right_camera = 1

path = os.path.dirname(os.path.realpath(__file__))

left_image_path = path + "/"+ save_folder_name + "/left"
right_image_path = path + "/"+ save_folder_name + "/right"

if not os.path.exists(left_image_path):
    os.makedirs(left_image_path)

if not os.path.exists(right_image_path):
    os.makedirs(right_image_path)

if is_for_meshroom: # Adding encoding for meshroom to recognize the camera.
    exif_bytes_left = piexif.dump({"0th": {piexif.ImageIFD.Make: u"StereoLab", piexif.ImageIFD.Model: u"Zed2i"}, 
     "Exif": {piexif.ExifIFD.FocalLength: (int(focal_length * 1000),1000), piexif.ExifIFD.BodySerialNumber: "8007232"}})
    exif_bytes_right = piexif.dump({"0th": {piexif.ImageIFD.Make: u"StereoLab", piexif.ImageIFD.Model: u"Zed2i"}, 
     "Exif": {piexif.ExifIFD.FocalLength: (int(focal_length * 1000),1000), piexif.ExifIFD.BodySerialNumber: "8007233"}})


bag = rosbag.Bag(sys.argv[1])
bridge = cv_bridge.CvBridge()

topic_list = [left_camera_topic , right_camera_topic]
image_buffer = {topic:[] for topic in topic_list}




max_len = bag.get_message_count(topic_filters=left_camera_topic) +  bag.get_message_count(topic_filters=right_camera_topic)
print("extracting pics...")
bar = progressbar.ProgressBar(max_value=max_len, widgets=widgets).start()
counter = 0
for topic, msg, t in  bag.read_messages(topics=topic_list):
    save_image(topic, msg)
    bar.update(counter)
    counter +=1

max_len = max([len(image_buffer[topic_list[0]]), len(image_buffer[topic_list[1]])])

counter = 0
print("\nchecking for synchronicity")
bar = progressbar.ProgressBar(max_value=max_len, widgets=widgets).start()

while len(image_buffer[topic_list[1]])>0 and len(image_buffer[topic_list[0]])>0:
    """ 
    checking synchronicity
    """
    right_timestamp, right = image_buffer[topic_list[right_camera]][0]
    left_timestamp, left = image_buffer[topic_list[left_camera]][0]
    if right_timestamp < left_timestamp:
        image_buffer[topic_list[right_camera]].pop(0)
    elif left_timestamp <= right_timestamp:
        if counter%frame_interval == 0:
            if is_for_meshroom:
                cv2.imwrite(left_image_path + "/image_" + str(counter) + ".jpg", left)
                piexif.insert(exif_bytes_left, left_image_path + "/image_" + str(counter) + ".jpg")
                cv2.imwrite(right_image_path + "/image_" + str(counter) + ".jpg", right)
                piexif.insert(exif_bytes_right, right_image_path + "/image_" + str(counter) + ".jpg")
            else:
                cv2.imwrite(left_image_path + "/image_" + str(counter) + ".png", left)
                cv2.imwrite(right_image_path + "/image_" + str(counter) + ".png", right)

        image_buffer[topic_list[left_camera]].pop(0)
        bar.update(counter)
        counter += 1
