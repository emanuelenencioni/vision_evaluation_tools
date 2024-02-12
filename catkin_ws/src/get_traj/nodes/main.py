#!/usr/bin/python3.8

import rospy
from geometry_msgs.msg import PoseStamped
import threading
import time
import json
import sys

class get_traj():
    def __init__(self):
        self.point_and_pose = []
        self.input_topic = rospy.get_param("/get_traj/input_topic")
        self.json_path = rospy.get_param("/get_traj/json_path")
        self.counter = 0
        self.mutex = threading.Lock()
        self.main()

    def callback_global(self, msg):
        self.mutex.acquire()
        self.point_and_pose.append({"position":[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], "orientation":[msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], "timestamp": msg.header.stamp.secs, "nsec":  msg.header.stamp.nsecs})
        
        self.counter += 1
        self.mutex.release()
        print("took "+str(self.counter) +" poses")
        
        
    def main(self):
        rospy.init_node('get_traj')
        rospy.Subscriber(self.input_topic, PoseStamped, self.callback_global)
        thread = threading.Thread(target=self.save_loop)
        thread.start()
        
    def save_loop(self):
        old_len = -1
        check_new_data = False
        while not rospy.is_shutdown():
            print("looping")
            self.mutex.acquire()
            len_ = len(self.point_and_pose)
            self.mutex.release()
            if old_len == -1 or len_ == 0:
                old_len = len_
            elif old_len != len_ :
                time.sleep(1)
                print("option 2")
                old_len = len_
            elif old_len == len_:
                print("two same values")
                time.sleep(1)
                self.mutex.acquire()
                len_ = len(self.point_and_pose)
                self.mutex.release()
                if len_ == old_len:
                    with open(self.json_path, "w") as f:
                        print("saving...")
                        print(self.json_path)
                        json.dump(self.point_and_pose, f)
                        rospy.signal_shutdown("finish")
                        sys.exit()

            time.sleep(1)
            

get_traj()
