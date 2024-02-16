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


#read in images
path1 = "/media/colin/box_data/ir_data/nuance_data/kri_day_2/cam_3/matlab_clahe2"
path2 = "/media/colin/box_data/ir_data/nuance_data/kri_day_2/cam_2/cam_2_normalized"

files1 = sorted(glob.glob(os.path.join(path1,"*.png")))
files2 = sorted(glob.glob(os.path.join(path2,"*.png")))
 
stamps1 = [os.path.basename(f)[:-4] for f in files1]
stamps2 = [os.path.basename(f)[:-4] for f in files2]

stamps1_float = np.array([float(s[:10] + "." + s[10:]) for s in stamps1])
stamps2_float = np.array([float(s[:10] + "." + s[10:]) for s in stamps2])

print(stamps1_float[0], stamps2_float[0])
print(stamps1_float[-1], stamps2_float[-1])



# start_sync_window = 60
# start_error = stamps1_float[0]-stamps2_float[-start_sync_window]

error = stamps1_float - stamps2_float[:len(stamps1_float)]
print(error[0])
fig, ax = plt.subplots()
# ax.plot(stamps1_float)
# ax.plot(stamps2_float)
ax.plot(error)
plt.show()