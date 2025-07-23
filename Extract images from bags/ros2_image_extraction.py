from rosbags.highlevel import AnyReader
from pathlib import Path
import cv2
from rosbags.image import message_to_cvimage
import yaml
import sys
import os
import piexif
from tqdm import tqdm
count = 0


if __name__ == "__main__":
    if len(sys.argv) != 3: sys.exit("use ros2_save_image.py rosbag_file config_file")

    with open(sys.argv[2]) as config_file:
        config = yaml.safe_load(config_file)
    
    camera_model = config['camera_model']
    focal_length = config['focal_length']
    stride = int(config['frame_interval'])

    exif_bytes_left = piexif.dump({"0th": {piexif.ImageIFD.Make: u"StereoLab", piexif.ImageIFD.Model: u"Zed2i"}, 
        "Exif": {piexif.ExifIFD.FocalLength: (int(focal_length * 1000),1000), piexif.ExifIFD.BodySerialNumber: "8007232"}})
    dir_path = os.path.dirname(os.path.realpath(__file__))
    dir_path = dir_path + "/" if dir_path[-1] != "/" else dir_path
    save_path = dir_path + config['save_folder_name'] +"/"
    if not os.path.isdir(save_path): os.makedirs(save_path)

    print(sys.argv[1])
    with AnyReader([Path(sys.argv[1])]) as reader:
        # topic and msgtype information is available on .connections list
        for connection in reader.connections:
            print(connection.topic, connection.msgtype)
        i = 0
        for connection, timestamp, rawdata in tqdm(reader.messages(), total=reader.message_count, desc="Extracting images"):
            if connection.topic == '/zed/zed_node/left_raw/image_raw_color': # topic Name of images
                if i % stride != 0: 
                    i+= 1
                    continue
                else:
                    img_path =save_path + "frame%04i.jpg" % count
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    img = message_to_cvimage(msg, 'bgr8') # change encoding type if needed
                    cv2.imwrite(img_path, img)
                    piexif.insert(exif_bytes_left, img_path)
                    count += 1
                    i+= 1
