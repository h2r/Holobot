import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import numpy as np
from math import pow, atan2, sqrt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import sys

rospy.init_node('movo_tf_listener')
listener = tf.TransformListener()
pub = rospy.Publisher('movo/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10.0)

class Pose:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

class MovoTeleop:
    def __init__(self):
        rospy.init_node('movo_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10.0)
        self.pose = Pose()

    def update_pose(self):
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            euler_rot = np.rad2deg(tf.transformations.euler_from_quaternion(rot)[2])
            self.pose.x, self.pose.y = trans
            self.pos.theta = euler_rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'Pose update failed.'
            sys.exit(1)

    def get_distance(self, goal_x, goal_y):
        self.update_pose()
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pos.y), 2))
        return distance

    def move2goal(self):
        goal_pose = Pose()
        goal_pose.x = input('Set your x goal:')
        goal_pose.y = input('Set your y goal:')
        distance_tolerance = 0.5
        vel_msg = Twist()
        while sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= distance_tolerance:
            # Proportional Controller
            # Linear velocity in the x-axis
            vel_msg.linear.x = 1.5 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            vel_msg.linear.y = 0
            vel.msg.linear.z = 0
            # Angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 4 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        # Stop robot after movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.spin()

def main():
    try:
        movo = MovoTeleop()
        movo.move2goal()
    except rospy.ROSInterruptException:
        return

if __name__ == '__main__':
    main()
