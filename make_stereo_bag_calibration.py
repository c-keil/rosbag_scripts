import sys
# import rosbag
import time
import cv2
import numpy as np
import os
import glob
# import cv_bridge
# import rospy 
import matplotlib.pyplot as plt

def stamp_to_ros_stamp(stamp):
    return int(stamp[:10]), int(stamp[10:])

if __name__ == "__main__":

    #read in images
    sn0 = 52360 #left cam_3
    sn1 = 52367 #right cam_2

    path1 = "/media/colin/DATA/ir_calibration/pair/0_cal/cal"
    path2 = "/media/colin/DATA/ir_calibration/pair/1_cal/cal"
    topic1 = "/boson_camera_array/cam_3/image_raw"
    topic2 = "/boson_camera_array/cam_2/image_raw"
    
    save_path = "/home/colin/calibration_bag.bag"
    start_stamp = "1689804919375999928"
    files1 = sorted(glob.glob(os.path.join(path1,"*.png")))
    files2 = sorted(glob.glob(os.path.join(path2,"*.png")))

    #actually make the bag
    import rospy
    import rosbag
    import cv_bridge
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Header
    bridge = cv_bridge.CvBridge()
    with rosbag.Bag(save_path, "w") as outbag:
        i = 0
        for i, (f1,f2) in enumerate(zip(files1,files2)):
            print(f1)
            print(f2)

            image1 = cv2.imread(f1, 0)
            image2 = cv2.imread(f2, 0)
            image_message1 = bridge.cv2_to_imgmsg(image1, header=Header(seq = i))
            image_message1.header.stamp.secs, image_message1.header.stamp.nsecs = stamp_to_ros_stamp(start_stamp)
            image_message1.header.stamp.secs = image_message1.header.stamp.secs + i
            image_message2 = bridge.cv2_to_imgmsg(image2, header=Header(seq = i))
            image_message2.header.stamp.secs, image_message2.header.stamp.nsecs = stamp_to_ros_stamp(start_stamp) #use stamp 1 for both
            image_message2.header.stamp.secs = image_message2.header.stamp.secs + i

            outbag.write(topic1, image_message1, t=image_message1.header.stamp)
            outbag.write(topic2, image_message2, t=image_message2.header.stamp)

    # print("Max error = {}".format(np.max(np.abs(error))))
    # print("mean error = {}".format(np.mean(error)))
    # # fig, ax = plt.subplots()
    # # ax.plot(error)
    # # plt.show()