import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

pub = rospy.Publisher('movo/cmd_vel', Twist)

def callback(data):
    twist = Twist()
    if data.buttons[3]: # forward
        twist.linear.x = 0.2
    elif data.buttons[2]: # backward
        twist.linear.x = -0.2
    print twist.linear.x
    pub.publish(twist)

def start():
    #global pub
    rospy.Subscriber('joy', Joy, callback)
    rospy.init_node('gary_teleop_movo')
    rospy.spin()

if __name__ == '__main__':
    start()
