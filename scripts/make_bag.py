import sys
import rosbag
import time
import cv2
import numpy as np
import os
import glob
import cv_bridge
import rospy 

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from copy import deepcopy

def get_image(fname):
    #ensures 8 bit
    tstamp = os.path.basename(fname)[:-4]
    # print(tstamp)
    return cv2.imread(fname, 0) , tstamp
    # return cv2.cvtColor(cv2.imread(fname, 0), cv2.COLOR_GRAY2BGR) , tstamp

if __name__ == "__main__":

    # fname = "/media/colin/box_data/ir_data/nuance_data/kri_day_2/cam_3/matlab_clahe2/1689804908243000031.png"
    path = "/media/colin/box_data/ir_data/nuance_data/kri_day_2/cam_3/matlab_clahe2/"
    topic = "/boson_camera_array/cam_3/image_raw"
    start_stamp = 1689804920342999935

    files = sorted(glob.glob(os.path.join(path, "*.png")))

    bridge = cv_bridge.CvBridge()
    with rosbag.Bag(os.path.join(path,"out_bag_test.bag"), "w") as outbag:
        for i, f in enumerate(files):
            print(f)
            image, stamp = get_image(f)
            if int(stamp) < start_stamp:
                pass
            # tstamp = rospy.Time(secs=int(stamp[:10]), nsecs=int(stamp[10:]))
            image_message = bridge.cv2_to_imgmsg(image, header=Header(seq = i))
            image_message.header.stamp.secs = int(stamp[:10])
            image_message.header.stamp.nsecs = int(stamp[10:])
            # print(int(stamp[10:]), int(stamp[:10]))
            # quit()
            nstamp = deepcopy(image_message.header.stamp)
            # nstamp.secs = nstamp.secs + 1
            outbag.write(topic, image_message, t=image_message.header.stamp)

    # for file in files:
    #     print(file)
    
    # im, stamp = get_image(fname)
    # print(stamp)
