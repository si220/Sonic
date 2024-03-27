#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import pyrealsense2 as rs
import numpy as np
import cv2
import argparse

# Argument parsing
parser = argparse.ArgumentParser(description='Process depth data and publish as LaserScan message.')
parser.add_argument('--depth', action='store_true', help='Show the depth image using OpenCV.')
args = parser.parse_args()

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Initialize ROS node
rospy.init_node('depth_to_laserscan', anonymous=True)

# Create a ROS publisher for the LaserScan message
laser_scan_publisher = rospy.Publisher('/scan', LaserScan, queue_size=10)

# Rate of publishing
publish_rate = rospy.Rate(5)

# Total FOV of the camera
total_fov = 86

try:
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Show the depth image if the --depth flag is used
        if args.depth:
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            cv2.imshow('Depth Image', depth_colormap)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        image_height, image_width = depth_image.shape
        band_height = 100
        midpoint = image_height // 2

        # Calculate the start and end points for the narrow band
        start_index = max(0, midpoint - band_height // 2)
        end_index = min(image_height, midpoint + band_height // 2)

        # Extract the narrow band
        narrow_band = depth_image[start_index:end_index, :]

        # Split the horizontal view into sections
        num_sections = narrow_band.shape[1] // 20
        section_width = narrow_band.shape[1] / num_sections

        min_threshold = 0.2
        max_threshold = 6.0

        # Prepare LaserScan message
        laser_scan_msg = LaserScan()
        laser_scan_msg.header.stamp = rospy.Time.now()
        laser_scan_msg.header.frame_id = "map"
        laser_scan_msg.angle_min = -total_fov / 2 * np.pi / 180
        laser_scan_msg.angle_max = total_fov / 2 * np.pi / 180
        laser_scan_msg.angle_increment = (laser_scan_msg.angle_max - laser_scan_msg.angle_min) / (num_sections - 1)
        laser_scan_msg.range_min = min_threshold
        laser_scan_msg.range_max = max_threshold
        laser_scan_msg.ranges = []

        for section in range(num_sections):
            start_index = int(section * section_width)
            end_index = int(start_index + section_width)
            section_data = narrow_band[:, start_index:end_index]
            section_data_meters = section_data * depth_scale
            valid_distances = section_data_meters[(section_data_meters >= min_threshold) & (section_data_meters <= max_threshold)]

            # Calculate the minimum distance within the valid range for this section
            if valid_distances.size > 0:
                min_distance = np.min(valid_distances)
            else:
                min_distance = float('Inf')  # No valid measurement

            laser_scan_msg.ranges.append(min_distance)

        # Publish the LaserScan message
        laser_scan_publisher.publish(laser_scan_msg)

        publish_rate.sleep()

finally:
    pipeline.stop()
    if args.depth:
        cv2.destroyAllWindows()
