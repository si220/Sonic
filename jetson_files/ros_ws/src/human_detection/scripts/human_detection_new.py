#!/usr/bin/env python3

#some of this code has been taken from the Intel RealSense Library Examples found at https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python/examples

import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
from ultralytics import YOLO
import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import math
import actionlib

def get_min(depth_data, x1, y1, x2, y2):
    step_size_x = int(abs(x1-x2)*0.5)
    if step_size_x <1:
        step_size_x = 1
        
    
    step_size_y = int(abs(y1-y2)*0.5)
    if step_size_y < 1:
        step_size_y = 1
        
    curr_min = None
    for i in range(x1, x2, step_size_x):
        for j in range(y1,y2, step_size_y):
            curr_d = depth_data.get_distance(i,j)
            if curr_min:
                if curr_d > 0:
                    curr_min = min(curr_d, curr_min)
            else:
                curr_min = curr_d

    if curr_min is None:
        curr_min = 0
    return curr_min
    
     
model = YOLO('yolov8n.pt')

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

pose_read = None
transform_matrix = None

def callback(data):
    global pose_read
    global transform_matrix
    pose_read = data
    print("got some pose info")
    print(pose_read.pose.pose)
    theta = 2 * math.atan(pose_read.pose.pose.orientation.z/pose_read.pose.pose.orientation.w)
    print(theta * 180/math.pi)
    transform_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])


def main():
    global pose_read
    pub1 = rospy.Publisher('human_coord', String, queue_size=10)

    rospy.init_node('human_detection', anonymous=True)
    rospy.Subscriber("/RosAria/pose", Odometry, callback)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rate = rospy.Rate(10)
   
    try:
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()


        while not rospy.is_shutdown():
       
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

        # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())


        # Render images:
        #   depth align to color on left
        #   depth on right
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        
            results = model.track(color_image, persist=True, classes=[0])
            plotted = results[0].plot()
        
            boxes = results[0].boxes.xyxy
            boxes = boxes.numpy()

            closest_human = [0, 0, 20]
            mid_min_x = 0
            mid_min_y = 0
            min_start_point = (0,0)
            min_end_point = (0,0)
            for box in boxes:
                x1, y1, x2, y2 = box
                x1 = int(x1)
                x2 = int(x2)
                y1 = int(y1)
                y2 = int(y2)
                start_point = (x1, y1)
                end_point = (x2, y2)
                box_color = (0,0,255)
                print(box)
                print(start_point)
                cv2.rectangle(color_image, start_point, end_point, box_color, 2)
                cv2.rectangle(depth_colormap, start_point, end_point, box_color, 2)




                mid_x = int((x1+x2)/2)
                mid_y = int((y1+y2)/2)
                dist = get_min(aligned_depth_frame, x1, y1, x2, y2)# aligned_depth_frame.get_distance(mid_x,mid_y)

                #converting to xyz coord
                X_rw = dist*(mid_x - intr.ppx)/intr.fx
                Y_rw = dist*(mid_y -intr.ppy)/intr.fy
                Z_rw = dist

                if Z_rw <= closest_human[2]:
                    closest_human = []
                    closest_human.append(X_rw)
                    closest_human.append(Y_rw)
                    closest_human.append(Z_rw)
                    mid_min_y = mid_y
                    mid_min_x = mid_x
                    min_start_point = start_point
                    min_end_point = min_end_point

            
            print(closest_human)
            print(type(closest_human[2]))
            coord_text = "({:.2f}, {:.2f}, {:.2f})".format(closest_human[0], closest_human[1], closest_human[2]) 
            font = cv2.FONT_HERSHEY_SIMPLEX
            depth_text = "{:.2f}".format(closest_human[2])
            cv2.putText(depth_colormap, depth_text, (mid_min_x, mid_min_y), font, 0.8, (255,255,255), 2, cv2.LINE_AA)
            cv2.putText(color_image, coord_text, (mid_min_x, mid_min_y), font, 0.8, (255,255,255), 2, cv2.LINE_AA)
        
        
            pub1.publish(str(closest_human))
            images = np.hstack((color_image, depth_colormap))
            #cv2.namedWindow('Human detection', cv2.WINDOW_NORMAL)
            cv2.imshow('Human detection', images)
            key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

            if pose_read != None:
                test = MoveBaseGoal()
                test.target_pose.header.frame_id = 'odom'
                test.target_pose.header.stamp = rospy.Time.now()

                point = np.array([closest_human[2] + 0.25, -closest_human[0]])
                rotated_point = np.dot(transform_matrix, point)

                test.target_pose.pose.position.x = pose_read.pose.pose.position.x + rotated_point[0]
                test.target_pose.pose.position.y = pose_read.pose.pose.position.y + rotated_point[1]
                test.target_pose.pose.orientation.z = pose_read.pose.pose.orientation.z
                test.target_pose.pose.orientation.w = pose_read.pose.pose.orientation.w

                go = input("Yes?")
                if go == ("y" or "Y"):
                    client.send_goal(test)
                    client.wait_for_result()
                    print("published")
                else:
                    pass
                

            rate.sleep()

    finally:
    # Stop streaming
        pipeline.stop()
        
if __name__ == '__main__':
    main()
