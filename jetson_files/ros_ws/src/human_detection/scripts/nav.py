#!/usr/bin/python3
import rospy
import actionlib
import math
import numpy as np
from std_msgs.msg import String, Int32, Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('Navigation', anonymous=True)
        self.twist_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size = 1)
        self.nudge_pub = rospy.Publisher('nudge', Bool, queue_size=1)
        self.ui_sub = rospy.Subscriber('state', String, self.ui_callback)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        self.vision_sub = rospy.Subscriber('human_coord', MoveBaseGoal, self.vis_callback)
        self.human_detect_sub = rospy.Subscriber('human_detected', String, self.hd_callback)
        self.pose_sub = rospy.Subscriber('/RosAria/pose', Odometry, self.pose_callback)
        # self.angle_sub = rospy.Subscriber('sound_angle', Int32, self.angle_callback)
        self.goal_reached = False
        self.state = "idle"
        self.human_detected = "False"
        self.coord = [0, 0, 0, 1]
        self.sound_loc = [0, 0, 0, 1]
        self.point_loc = [0, 0, 0, 1]
        self.theta = 0

    def send_goal(self, x, y, orientation_z, orientation_w):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Goal Translation
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0

        # Goal Orientation
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = orientation_z
        goal.target_pose.pose.orientation.w = orientation_w
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("Reached Goal")

    def status_callback(self, status_array):
        if status_array.status_list:
            status = status_array.status_list[0]
            if status.status == 3 and not self.goal_reached:  # Goal reached and not previously marked as reached
                self.goal_reached = True
                nudge_msg = Bool()
                nudge_msg.data = True
                self.nudge_pub.publish(nudge_msg) # publish to nudge topic for arduino to start arm movement

    def vis_callback(self, data):
        x = data.target_pose.pose.position.x 
        y = data.target_pose.pose.position.y 
        orientation_z = data.target_pose.pose.orientation.z 
        orientation_w = data.target_pose.pose.orientation.w 
        self.coord = [x, y, orientation_z, orientation_w]
        """
        # Wait for goal to be reached before moving to the next waypoint
        while not rospy.is_shutdown() and not self.goal_reached:
            rospy.sleep(1.5)  # Adjust sleep time as needed
        """

    def ui_callback(self, state):
        self.state = state.data
        print(self.state)
        
    def hd_callback(self, hd):
        self.human_detected = hd.data

    def pose_callback(self, pose):
        x = pose.pose.pose.position.x
        y = pose.pose.pose.position.y
        z = pose.pose.pose.orientation.z
        w = pose.pose.pose.orientation.w
        if self.state == "idle" :
            self.sound_loc = [x, y, z, w]
            print(self.sound_loc)
    """
    def point_to_quaternion(self, start, goal):
        # Calculate the direction vector from goal to start
        direction_vector = [start[0] - goal[0], start[1] - goal[1]]
    
        # Calculate the angle of the direction vector
        # atan2 handles the correct quadrant of the angle
        angle = math.atan2(direction_vector[1], direction_vector[0])
        
        # Convert the angle to quaternion components
        qw = math.cos(angle / 2.0)
        qz = math.sin(angle / 2.0)
        return qz, qw

    """
    """
    def angle_callback(self, angle):
        self.angle = angle.data

    """

    def main_loop(self):
        rate = rospy.Rate(10)  # 10 Hz
        run_once = 0
        run = 0
        point = 0
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.y = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = -0.5
        while not rospy.is_shutdown():
            if navigator.state == "idle":
                run_once = 0
                self.goal_reached = False
                if run == 0:
                    # z_rot, w_rot = point_to_quaternion(self, [0, 0], [0, 1])
                    
                    navigator.send_goal(1.6767678260803223, -1.612467885017395, 0.4522009426993339, 0.8919160876572604)
                    rospy.loginfo("Sent first point")
                    navigator.send_goal(2.8450024127960205, 0.13299666345119476, -0.30848213671793145, 0.9512301358377685)
                    run = 1
            elif navigator.state == "find user":
                
                while(self.human_detected == "False" and self.goal_reached == False):
                    self.twist_pub.publish(msg)
                if run_once == 0:
                    navigator.send_goal(navigator.coord[0]-0.2, navigator.coord[1], navigator.coord[2], navigator.coord[3])
                    rospy.loginfo("Sent goal")
                    run_once = 1
                navigator.state == "idle"
                
            # elif navigator.state == "point":
                
            #     theta = navigator.theta*180/math.pi
            #     new_z = np.sin(theta/2)
            #     new_w = np.cos(theta/2) 
                
            #     start = [0,0]
            #     goal= [0,1]
            #     direction_vector = [start[0] - goal[0], start[1] - goal[1]]
    
            #     # Calculate the angle of the direction vector
            #     # atan2 handles the correct quadrant of the angle
            #     angle = math.atan2(direction_vector[1], direction_vector[0])
                
            #     # Convert the angle to quaternion components
            #     qw = math.cos(angle / 2.0)
            #     qz = math.sin(angle / 2.0)
            #     print(qw, qz)
            #     if point == 0:
            #         # z_rot, w_rot = point_to_quaternion(self, [0, 0], [0, 1])
                    
            #         navigator.send_goal(0, 0, qz, qw)
            #         point = 1
            
            elif navigator.state == "guide":
                navigator.send_goal(navigator.sound_loc[0], navigator.sound_loc[1], navigator.sound_loc[2], navigator.sound_loc[3])
                rospy.loginfo("Move to stored sound loc")
                navigator.state == "idle"

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        rospy.loginfo("Start")
        navigator.main_loop()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
