#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64 

vel_pub = rospy.Publisher('Linear', String, queue_size=10)

def callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

#  def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber("chatter", String, callback)
    rospy.Subscriber("/cmd_vel", Twist, callback) #/cmd_vel /key_vel /ps3_vel /joy
    vel_msg = Twist()
    x = vel_msg.linear.x
    z = vel_msg.angular.z
    print("the Value of x ".format(x))
    print("the Value of x ".format(z))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()