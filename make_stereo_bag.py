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

def camera_data(path, topic):
    #read in camera data and get stamps
    files = sorted(glob.glob(os.path.join(path,"*.png")))
    stamps = [os.path.basename(f)[:-4] for f in files]
    stamps_float = np.array([float(s[:10] + "." + s[10:]) for s in stamps])
    data = {"path":path, "files":files, "stamps":stamps, "times":stamps_float, "topic":topic}
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

def stamp_to_ros_stamp(stamp):
    return int(stamp[:10]), int(stamp[10:])

if __name__ == "__main__":

    #read in images
    # path1 = "/media/colin/box_data/ir_data/nuance_data/kri_day_2/cam_3/matlab_clahe2"
    # path2 = "/media/colin/box_data/ir_data/nuance_data/kri_day_2/cam_2/cam_2_normalized"

    path1 = "/media/colin/DATA1/kri_day2/cam3/matlab_clahe2"
    path2 = "/media/colin/DATA1/kri_day2/cam2/matlab_clahe2"
    topic1 = "/boson_camera_array/cam3/image_raw"
    topic2 = "/boson_camera_array/cam2/image_raw"
    save_path = "/home/colin/kri_day_bag.bag"
    save_path1 = "/media/colin/DATA1/kri_day2/stereo/cam_3"
    save_path2 = "/media/colin/DATA1/kri_day2/stereo/cam_2"

    start_stamp = 1689804919375999928
    camera1_data = camera_data(path1, topic1)
    camera2_data = camera_data(path2, topic2)

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

    #actually make the bag
    import rospy
    import rosbag
    import cv_bridge
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Header
    from shutil import copy2
    bridge = cv_bridge.CvBridge()

    with rosbag.Bag(save_path, "w") as outbag:
        i = 0
        leader_topic = leader["topic"]
        follower_topic = follower["topic"]
        for i in range(len(leader["times"])):
            f1 = leader["files"][i]
            f2 = follower["files"][i]
            print(f1)
            print(f2)

            stamp1 = leader["stamps"][i]
            if int(stamp1) < start_stamp:
                continue
            
            #write rosbag
            image1 = cv2.imread(f1, 0)
            image2 = cv2.imread(f2, 0)
            image_message1 = bridge.cv2_to_imgmsg(image1, header=Header(seq = i))
            image_message1.header.stamp.secs, image_message1.header.stamp.nsecs = stamp_to_ros_stamp(stamp1)
            image_message2 = bridge.cv2_to_imgmsg(image2, header=Header(seq = i))
            image_message2.header.stamp.secs, image_message2.header.stamp.nsecs = stamp_to_ros_stamp(stamp1) #use stamp 1 for both
            outbag.write(topic1, image_message1, t=image_message1.header.stamp)
            outbag.write(topic2, image_message2, t=image_message2.header.stamp)

            #copy files
            copy2(f1, save_path1)
            copy2(f2, os.path.join(save_path2,os.path.basename(f1)))

    print("Max error = {}".format(np.max(np.abs(error))))
    print("mean error = {}".format(np.mean(error)))
    # fig, ax = plt.subplots()
    # ax.plot(error)
    # plt.show()