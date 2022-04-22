#!/usr/bin/env python3
from re import X
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64 
import odrive
import sys, termios, tty, os, time

# vel_pub = rospy.Publisher('Linear', String, queue_size=10)
pub_linearvel = rospy.Publisher("LinearVelocity", String , queue_size=10)
pub_angularvel = rospy.Publisher("AngularVelocity", String , queue_size=10) 

serial1 = "365938573131"

odrv1 = odrive.find_any(serial_number=serial1)

motor_l = odrv1.axis0
motor_r = odrv1.axis1

def right_speed(sp):
	print("Right Speed : ")
	print(sp)
	motor_r.controller.input_vel = sp
	
def left_speed(sp):
	print("Left Speed : ")
	print(sp)
	motor_l.controller.input_vel = sp

def callback(data):
        v_linear = data.linear.x
        w_angular = data.data.angular.z

        # print("This is a test ")
        # print()
        rospy.loginfo(rospy.get_caller_id() + "The linear velocity %s", v_linear)
        rospy.loginfo(rospy.get_caller_id() + "The angular velocity %s", data.angular.z)
#  def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Velocity', anonymous=True)

    # rospy.Subscriber("chatter", String, callback)
    rospy.Subscriber("/cmd_vel", Twist, callback) #/cmd_vel /key_vel /ps3_vel /joy
    vel_msg = Twist()
    # x = vel_msg.linear.x
    # pub_linearvel = x
    # z = vel_msg.angular.z
    # pub_angularvel = z 
    linVel = vel_msg.linear.x 
    angVel = vel_msg.angular.y
    
    if (angVel == 0):
        vleft = linVel 
        vright = linVel
        nl = 1.3 * vleft
        nr = 1.3 * vright
        right_speed(nr)
        left_speed(nl)

    if (angVel > 0):
        centerRot = linVel / angVel 
        vleft = angVel * ( centerRot + 0.2925)
        vright =  -1 * angVel * ( centerRot + 0.2925)    
        nl = 1.3 * vleft
        nr = 1.3 * vright
        right_speed(nr)
        left_speed(nl)
    
    if (angVel < 0):
        centerRot = linVel / angVel 
        vleft = -1 * angVel * ( centerRot + 0.2925)
        vright =  angVel * ( centerRot + 0.2925)    
        nl = 1.3 * vleft
        nr = 1.3 * vright
        right_speed(nr)
        left_speed(nl)

    # print("the Value of x ".format(x))
    # print("the Value of x ".format(z))
    


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

