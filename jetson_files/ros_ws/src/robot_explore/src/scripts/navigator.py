#!/usr/bin/python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        self.vision_sub = rospy.Subscriber('/test123', MoveBaseGoal, self.vision_callback)
        self.goal_reached = False
        self.state = 2

    def send_goal(self, x, y, orientation_z, orientation_w):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Goal Translation
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Goal Orientation
        goal.target_pose.pose.orientation.z = orientation_z
        goal.target_pose.pose.orientation.w = orientation_w

        self.goal_reached = False  # Reset goal reached flag
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def status_callback(self, status_array):
        if status_array.status_list:
            status = status_array.status_list[0]
            if status.status == 3 and not self.goal_reached:  # Goal reached and not previously marked as reached
                rospy.loginfo("Goal reached.")
                self.goal_reached = True

    def vision_callback(self, vision_goal):
        if self.state == 1:
            pass

        if self.state == 2:
            # Extract relevant information from the received vision goal
            x = vision_goal.target_pose.pose.position.x
            y = vision_goal.target_pose.pose.position.y
            orientation_z = vision_goal.target_pose.pose.orientation.z
            orientation_w = vision_goal.target_pose.pose.orientation.w

            # Send the goal to move_base
            self.send_goal(x, y, orientation_z, orientation_w)
            rospy.loginfo(f"Received human coord. Sent goal to ({x}, {y})")

    def navigate(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    try:
        rospy.loginfo("Launching")
        navigator = WaypointNavigator()

        navigator.navigate()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
