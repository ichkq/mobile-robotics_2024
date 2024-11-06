#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Int32
from math import atan2, sqrt, hypot
import time

class SimplePoseController:
    def __init__(self) -> None:
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/line_position', Int32, self.line_position_callback)
        self.sub2 = rospy.Subscriber('/line_x_position', Float32, self.line_x_position_callback)
        self.sub3 = rospy.Subscriber('/image_width', Int32, self.image_width_callback)
        self.width = 1
        self.rate = rospy.Rate(1)

    # Controls the motion of the robot
    def line_position_callback(self, msg:Int32):
        if rospy.get_param('steering_method') == 'bangbang':
            position = msg.data
            speed : Twist = Twist()

            if position == 0: # center
                speed.linear.x = 0.22
            elif position > 0: # right
                speed.angular.z = -0.3
            elif position < 0: # left
                speed.angular.z = 0.3
            else: # ignore all other values
                pass
            
            self.pub.publish(speed)
    
    def line_x_position_callback(self, action:Float32) -> None:
        if rospy.get_param('steering_method') == 'proportional':
            action_val = float(action.data)
            speed : Twist = Twist()

            # Proportional control
            speed.linear.x = 0.22
            speed.angular.z = -(action_val - self.width/2) / 100

            self.pub.publish(speed)
            rospy.sleep(0.01)

    def image_width_callback(self, msg:Int32) -> None:
        self.width = msg.data

    def switch_steering_method(self) -> None:
        if rospy.get_param('steering_method') == 'proportional':
            rospy.set_param('steering_method', 'bangbang')
        else:
            rospy.set_param('steering_method', 'proportional')


if __name__ == '__main__':
    time.sleep(10)

    rospy.init_node('motion_node')

    rospy.set_param('steering_method', 'proportional') # Default mode on 

    simple_pose_controller = SimplePoseController()

    rospy.spin()
