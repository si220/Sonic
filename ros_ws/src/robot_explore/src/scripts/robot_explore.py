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
        self.goal_reached = False

    def send_goal(self, x, y, orientation_z, orientation_w):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
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

        self.goal_reached = False  # Reset goal reached flag
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def status_callback(self, status_array):
        if status_array.status_list:
            status = status_array.status_list[0]
            if status.status == 3 and not self.goal_reached:  # Goal reached and not previously marked as reached
                rospy.loginfo("Goal reached.")
                self.goal_reached = True

    def navigate(self):
        while not rospy.is_shutdown():
            waypoints = [
                (7.4, -3.5, 0.0, 1.0),
                (1.4, -3.9, 1.0, 0.0),
                (-6.6, -3.1, 1.0, 0.0),
                (-6.3, 0.1, 0.0, 1.0),
                (-0.3, 2.2, 0.0, 1.0),
                (5.1, 1.3, 0.0, 1.0)
                # Add more waypoints as needed
            ]

            for waypoint in waypoints:
                self.goal_reached = False
                x, y, orientation_z, orientation_w = waypoint
                self.send_goal(x, y, orientation_z, orientation_w)
                rospy.loginfo(f"Sent goal to ({x}, {y})")

                # Wait for goal to be reached before moving to the next waypoint
                while not rospy.is_shutdown() and not self.goal_reached:
                    rospy.sleep(1.5)  # Adjust sleep time as needed

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        navigator.navigate()
        rospy.loginfo("Patrolling finished")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
