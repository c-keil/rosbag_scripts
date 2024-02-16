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

def remove_frame(data, i):
    for key in data.keys():
        d = data[key]
        if type(d) == list:
            data[key] = d[:i]+d[i+1:]
        elif type(d) == np.ndarray:
            data[key] = np.hstack((d[:i],d[i+1:]))
        elif type(d) is str:
            pass
        else:
            raise TypeError("type is {}".format(type(d)))
    return data

def heal_data_one_way(leader, follower):
        #loop over data to find breaks
        i = 0
        last_delta = None
        frame_rate = 30
        while True:
            if i >= len(leader["times"]):
                break

            t = leader["times"][i]
            delta = t - leader["times"][i-1] if not i == 0 else 0
            #IDENTIFIED break
            if delta > 1.5/frame_rate:
                missing_frames = int(np.round(delta*frame_rate))
                print("missing {} frames at index {}".format(missing_frames, i))

                #skip corrsponding frames in the other stream
                follower["times"] = np.hstack((follower["times"][:i],follower["times"][i+missing_frames+1:]))
            i +=1
        return leader, follower

def heal_data(leader, follower):
        #loop over data to find breaks
        i = 0
        last_delta = None
        frame_rate = 30
        skip_idx = []
        while True:
            if i >= len(leader["times"]):
                break

            t = leader["times"][i]
            delta = t - leader["times"][i-1] if not i == 0 else 0
            #IDENTIFIED break
            if delta > 1.5/frame_rate:
                missing_frames = int(np.round(delta*frame_rate))
                print("time gap {}, frames {}".format(delta, delta*frame_rate))
                print("missing {} frames at index {}".format(missing_frames, i))
                try:
                    if i in leader["skip_idx"]:
                        i += missing_frames
                        continue
                except KeyError:
                    print("?")
                    pass
                #skip corrsponding frames in the other stream
                follower["times"] = np.hstack((follower["times"][:i],follower["times"][i+missing_frames+1:]))
                skip_idx.append(i)
            i +=1
        follower["skip_idx"] = skip_idx
        return leader, follower

def heal_bidirectional(leader, follower):
    #loop over data to find breaks
    i = 0
    j = 0
    last_delta = None
    frame_rate = 30
    skip_idx = []
    while True:
        #stop conditions
        if i >= len(leader["times"]):
            break
        if i >= len(follower["times"]):
            break
        
        if not last_delta is None:
            last_delta = delta
        
        t1 = leader["times"][i] 
        t2 = follower["times"][i]

        #check if gap has opened
        delta = t1 - t2
        if last_delta is None:
            last_delta = delta
        # delta = t1 - t2
        if np.abs(delta) > (0.5/frame_rate):
        # if np.abs(delta - last_delta) > (1.1/frame_rate):
            #remove frame from the non lagging stream
            if t2 > t1:
                print("delta = {}".format(delta))
                print("shortening leader at {} with len {}".format(i,len(leader["times"])))
                remove_frame(leader,i)
                # leader["times"] = np.hstack((leader["times"][:i],leader["times"][i+1:]))
                print("to len {}".format(len(leader["times"])))
                t1 = leader["times"][i] 
                t2 = follower["times"][i]
                delta = t1 - t2
                print("new delta : {}\n".format(delta))
                continue
            else:
                print("delta = {}".format(delta))
                print("shortening follower at {} with len {}".format(i,len(follower["times"])))
                # follower["times"] = np.hstack((follower["times"][:i],follower["times"][i+1:]))
                remove_frame(follower, i)
                print("to len {}".format(len(follower["times"])))
                t1 = leader["times"][i] 
                t2 = follower["times"][i]
                delta = t1 - t2
                print("new delta : {}\n".format(delta))
                continue
        else:
            # if follower gets ahhead of leader, remove 1 extra frame
            if t2 > t1:
                print("FOLLOWER AHHEAD")
                print("delta = {}".format(delta))
                print("shortening leader at {} with len {}".format(i,len(leader["times"])))
                remove_frame(leader,i)
                # leader["times"] = np.hstack((leader["times"][:i],leader["times"][i+1:]))
                print("to len {}".format(len(leader["times"])))
                t1 = leader["times"][i] 
                t2 = follower["times"][i]
                delta = t1 - t2
                print("new delta : {}\n".format(delta))
                continue
            # pass
        
        i +=1
        
    return leader, follower

if __name__ == "__main__":

    #read in images
    # path1 = "/media/colin/box_data/ir_data/nuance_data/kri_day_2/cam_3/matlab_clahe2"
    # path2 = "/media/colin/box_data/ir_data/nuance_data/kri_day_2/cam_2/cam_2_normalized"

    path1 = "/media/colin/DATA1/kri_day2/cam3/matlab_clahe2"
    path2 = "/media/colin/DATA1/kri_day2/cam2/matlab_clahe2"

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

    #heal data in both direactions
    leader, follower = heal_bidirectional(leader, follower)
    print("backwards")
    # follower, leader = heal_data(follower, leader)

    error = leader["times"][:len(follower["times"])]-follower["times"]
    # while True:
    #     if (i >= len(leader["times"])) or (j >= len(follower["times"])):
    #         break 
    #     t1 = leader["times"][i]
    #     # t2 = follower["times"][j]
        
    #     delta1 = t1 - t1_
        
        
    #     delta = t1 - t2
    #     abs_delta = np.abs(delta)
    #     if last_delta is None:
    #         last_delta = abs_delta
        
    #     #find discontinuity
    #     if np.abs(abs_delta - last_delta) > max_delta:
    #         print(i)
            
    #         #identify which steam is missing frames
    #         # if delta > 1
    # # else:
    #     #book keeping
    #     i += 1
    #     j += 1
    #     last_delta = abs_delta


# start_sync_window = 60
# start_error = stamps1_float[0]-stamps2_float[-start_sync_window]

# error = stamps1_float - stamps2_float[:len(stamps1_float)]

    fig, ax = plt.subplots()
    ax.plot(error)
    plt.show()
# # ax.plot(stamps1_float)
# # ax.plot(stamps2_float)