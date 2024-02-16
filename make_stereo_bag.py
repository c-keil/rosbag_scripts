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

def camera_data(path):
    #read in camera data and get stamps
    files = sorted(glob.glob(os.path.join(path,"*.png")))
    stamps = [os.path.basename(f)[:-4] for f in files]
    stamps_float = np.array([float(s[:10] + "." + s[10:]) for s in stamps])
    data = {"path":path, "files":files, "stamps":stamps, "times":stamps_float}
    return data

if __name__ == "__main__":

    #read in images
    path1 = "/media/colin/box_data/ir_data/nuance_data/kri_day_2/cam_3/matlab_clahe2"
    path2 = "/media/colin/box_data/ir_data/nuance_data/kri_day_2/cam_2/cam_2_normalized"

    camera1_data = camera_data(path1)
    camera2_data = camera_data(path2)

    stamps1 = camera1_data["times"]
    stamps2 = camera2_data["times"]

    #synchronize frame start
    #find which camera started first
    if stamps1[0] < stamps2[0]:
        leader = camera1_data
        follower = camera2_data
    else:
        leader = camera2_data
        follower = camera1_data
    
    # print(leader["path"])
    # print(follower["path"])
    # print(len(leader["times"]))
    # print(len(follower["times"]))
    #get the best sync start frame
    start_window_n_frames = 40 #camera is at 30hz
    start_error = follower["times"][0] - leader["times"][:start_window_n_frames]
    min_error_index = np.argmin(np.abs(start_error))
    #trim starting frames
    leader["files"] = leader["files"][min_error_index:]
    leader["stamps"] = leader["stamps"][min_error_index:]
    leader["times"] = leader["times"][min_error_index:]

    #plot error
    print("frame worst case offset {}".format(1./60))
    error = leader["times"][:len(follower["times"])]-follower["times"]

# start_sync_window = 60
# start_error = stamps1_float[0]-stamps2_float[-start_sync_window]

# error = stamps1_float - stamps2_float[:len(stamps1_float)]

    fig, ax = plt.subplots()
    ax.plot(error)
    plt.show()
# # ax.plot(stamps1_float)
# # ax.plot(stamps2_float)