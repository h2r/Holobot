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

velocity_publisher = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=10)
listener = tf.TransformListener()
max_speed = 0.2
min_speed = 0.1
rotation_tolerance = 0.05
distance_tolerance = 0.1

class Pose:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

class MovoTeleop:
    def __init__(self):
        rospy.init_node('movo_controller', anonymous=True)
        self.rate = rospy.Rate(50.0)
        self.pose = Pose()
        listener = tf.TransformListener()

    def update_pose(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                print 'Trans:', trans
                z_rot = tf.transformations.euler_from_quaternion(rot)[2]
                #euler_rot = np.rad2deg(tf.transformations.euler_from_quaternion(rot)[2])
                self.pose.x, self.pose.y = round(trans[0], 4), round(trans[1], 4)
                self.pose.theta = z_rot
                self.rate.sleep()
                return
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def get_distance(self, goal_x, goal_y):
        self.update_pose()
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def get_rot_distance(self, goal_x, goal_y, update_pose=True):
        if update_pose:
            self.update_pose()
        distance = atan2(goal_y - self.pose.y, goal_x - self.pose.x) - self.pose.theta
        return distance

    #def get_rot_distance2(self, goal_x, goal_y, update_pose=True):
    #    if update_pose:
    #        self.update_pose()
    #    return del_angle([np.cos(self.pose.theta), np.sin(self.pose.theta)], [goal_x-self.pose.x, goal_y-self.pose.y])

    def calibrate_rotation(self, goal_x, goal_y):
        rot_dist = self.get_rot_distance(goal_x, goal_y)
        while abs(rot_dist) >= rotation_tolerance:
            rot_dist = self.get_rot_distance(goal_x, goal_y)
            # Proportional Controller
            # Linear velocity in the x-axis
            self.vel_msg.linear.x = 0 
            self.vel_msg.linear.y = 0
            self.vel_msg.linear.z = 0
            # Angular velocity in the z-axis:
            self.vel_msg.angular.x = 0
            self.vel_msg.angular.y = 0
            self.vel_msg.angular.z = rot_dist
            #print 'rot err:', rot_dist #del_angle(unit([np.cos(self.pose.theta), np.sin(self.pose.theta)]), unit([goal_x-self.pose.x, goal_y-self.pose.y]))
            # Publishing our vel_msg
            velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()

    def calibrate_distance(self, goal_x, goal_y):
        while self.get_distance(goal_x, goal_y) >= distance_tolerance:
            if abs(self.get_rot_distance(goal_x, goal_y, update_pose=False)) >= rotation_tolerance:
                self.calibrate_rotation(goal_x, goal_y)
            # Proportional Controller
            # Linear velocity in the x-axis
            self.vel_msg.linear.x = 1.5 * sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
            self.vel_msg.linear.y = 0 
            self.vel_msg.linear.z = 0
            # Angular velocity in the z-axis:
            self.vel_msg.angular.x = 0
            self.vel_msg.angular.y = 0
            self.vel_msg.angular.z = 0
            # Publishing our vel_msg
            velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()

    def move2goal(self):
        goal_pose = Pose()
        self.update_pose()
        print 'Curr pose: ({}, {})'.format(self.pose.x, self.pose.y)
        goal_pose.x = input('Set your x goal:')
        goal_pose.y = input('Set your y goal:')
        self.vel_msg = Twist()
        self.calibrate_rotation(goal_pose.x, goal_pose.y)
        self.calibrate_distance(goal_pose.x, goal_pose.y)
        # Stop robot after movement is over
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        velocity_publisher.publish(self.vel_msg)
        #rospy.spin()

def unit(v):
    v = np.asarray(v)
    return v / np.linalg.norm(v)

def del_angle(v1, v2):
    v1 = np.asarray(v1)
    v2 = np.asarray(v2)
    u1 = v1 / np.linalg.norm(v1)
    u2 = v2 / np.linalg.norm(v2)
    s = np.linalg.norm(np.cross(v1, v2))
    c = np.dot(v1, v2)
    return np.arctan2(s, c)



def main():
    try:
        movo = MovoTeleop()
        movo.move2goal()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        return

if __name__ == '__main__':
    main()
