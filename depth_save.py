import rosbag
import cv2
import cv_bridge
import numpy as np

frame_interval = int(sys.argv[2])
focal_length = 2.12 # in mm
path = os.path.dirname(os.path.realpath(__file__))
if not os.path.exists(path + "/test/left"):
    os.makedirs(path + "/test/color")

if not os.path.exists(path + "/test/right"):
    os.makedirs(path + "/test/right")


exif_bytes_rgb = piexif.dump({"0th": {piexif.ImageIFD.Make: u"Intel Realsense", piexif.ImageIFD.Model: u"D455"}, "Exif": {piexif.ExifIFD.FocalLength: (int(focal_length * 1000),1000), piexif.ExifIFD.BodySerialNumber: "8007232"}})


bag = rosbag.Bag(sys.argv[1])
bridge = cv_bridge.CvBridge()

topic_list = ["/camera/color/image_raw", "/camera/aligned_depth_to_color/image_raw"]
image_buffer = {topic:[] for topic in topic_list}

def save_image(topic, msg):
    encoding = msg.encoding
    if encoding == 'rgb8':
        
        cv2.imwrite("/home/slam/test/color/image_" + str(counter) + ".png", cv_image)
        image_buffer[topic].append((msg.header.stamp, cv_image))
    elif encoding == '16UC1':
        #convert to 16UC1
        cv2.imwrite("/home/slam/test/depth/image_" + str(counter) + ".png", cv_image)
        image_buffer[topic].append((msg.header.stamp, cv_image))
    else:
        print("Image encoding not supported - "+str(encoding))

print("storing pics...")
for topic, msg, t in bag.read_messages(topics=topic_list):
    save_image(topic, msg)

counter = 0
print("checking for synchronicity")
while len(image_buffer[topic_list[1]])>0 and len(image_buffer[topic_list[0]])>0: #TODO
    """ topic_list[1] depth
        topic_list[0] color
        function that look for the depth corresponding to the color image
    """
    stamp1, depth = image_buffer[topic_list[1]][0]
    stamp2, color = image_buffer[topic_list[0]][0]
    if stamp1 < stamp2:
        image_buffer[topic_list[1]].pop(0)
    elif stamp2 <= stamp1:
        
        img2 = cv2.resize(depth, (0, 0), fx=0.5, fy=0.5)
        img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)

        cv2.imshow("Synchronized image", cv2.resize(color, (0,0), fx=0.5, fy=0.5))
        cv2.imshow("synchronized images", img2)
        cv2.waitKey(1)
        cv2.imwrite("/home/slam/test/color/image_" + str(counter) + ".png", color)
        cv2.imwrite("/home/slam/test/depth/image_" + str(counter) + ".png", depth)
        counter += 1
        image_buffer[topic_list[0]].pop(0)