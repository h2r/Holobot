import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

rospy.init_node('movo_tf_listener')
listener = tf.TransformListener()
pub = rospy.Publisher('movo/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10.0)

start_pos = None
rotate_rate = 0.5
move_speed = 0.2
twist = Twist()

def main():
    print 'Started!'
    while True:
        coord = raw_input('Input goal coord: ')
        goal_pos = extract_coord(coord)
        print 'Goal:', goal_pos
        #navigate_to_goal(goal_pos)
        reset_rotation()
        break

def extract_coord(input_coord):
    vals = input_coord.split(',')
    assert len(vals) == 2
    coords = np.asarray((float(vals[0]), float(vals[1])))
    return coords

def get_pose_update():
    counter = 0
    while counter < 10:
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            euler_rot = np.rad2deg(tf.transformations.euler_from_quaternion(rot)[2])
            trans = np.asarray(trans)
            assert isinstance(trans, np.ndarray)
            rate.sleep()
            return trans, euler_rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'Pose update failed. Trying again...'
            counter += 1
    return None

def navigate_to_goal(goal_delta):
    trans, euler_rot = get_pose_update()
    print 'trans:', trans
    print 'rot:,', euler_rot
    start_pos = trans
    goal_pos = start_pos + goal_delta
    #move_to_goal(start_pos, goal_pos, euler_rot)
    #reset_rotation()
    rate.sleep()

def stop_detected():
    if data.buttons[0]:
        print 'brake!'
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        return True
    return False

def move_to_goal(start_pos, goal_pos, start_rot):
    curr_pos, curr_rot = get_pose_update()
    goal_rot = get_angle(start_pos, goal_pos)
    pass # TODO

def reset_rotation(): # negative -> right, positive -> left
    print 'resetting rotation...'
    _, curr_rot = get_pose_update()
    print 'start rot:', curr_rot
    while(curr_rot > 0): # turn right
        twist.angular.z = -rotate_rate
        _, curr_rot = get_pose_update()
        pub.publish(twist)
        print 'rotating right...'
    while(curr_rot < 0): # turn left
        twist.angular.z = rotate_rate
        _, curr_rot = get_pose_update()
        pub.publish(twist)
        print 'rotating left...'
    twist.angular.z = 0.0
    pub.publish(twist)
    print 'done!'

def near_goal(coord1, coord2):
    return np.linalg.norm(coord1-coord2) < 1

def unit_vector(vector):
    assert isinstance(vector, np.ndarray)
    return vector / np.linalg.norm(vector)

def get_angle(coord1, coord2):
    v1 = unit_vector(coord1)
    v2 = unit_vector(coord2)
    return np.rad2deg(np.arctan2(coord2[1], coord2[0]) - np.arctan2(coord1[1], coord1[0]))

if __name__ == '__main__':
    main()
