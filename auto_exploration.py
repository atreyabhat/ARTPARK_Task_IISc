#!/usr/bin/env python


#author: Atreya G Bhat

import rospy
import numpy as np
from sensor_msgs.msg import Range,LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import sys, time, math

v_constant = 0.6  #linear velocity gain constant
w_constant = 0.4  #angular velocity gain constant
threshold = 0.15

phi = 90  #field of view
rays = 7  #N
roll = pitch = yaw = 0.0  #init

class Explorer:

        def __init__(self):

                #publishers and subscribers
                self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.laser_callback)  
                self.odom_sub = rospy.Subscriber('/odom',Odometry,self.odom_callback)     
                self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)
                self.yaw_pub = rospy.Publisher('yaw',Float32, queue_size=10)
                self.twist = Twist()



        def laser_callback(self, msg):

                self.ranges = msg.ranges

                N = self.ranges.index(max(self.ranges)) + 1  #index of max range
                initial_yaw = yaw   #store initial value of yaw

                if(N==4):
                        initial_yaw = 0   #if the maximum range is right infront of the robot
                        
                offset_angle = (phi/rays) * (N-4) * math.pi/180    #calculate the offset the robot needs to rotate towards the maximum range direction
                #self.twist.angular.z = w_constant * (offset_angle + initial_yaw - yaw) / min(self.ranges)

                #angular vel is controlled based on this controller, depending on the amount of angle to be rotated from initial yaw
                self.twist.angular.z = w_constant * (offset_angle + initial_yaw - yaw)

                
                #optional part to take a step back if the robot collides and gets stuck
                #################################################################

                if(min(self.ranges) < threshold):
                        self.twist.linear.x = v_constant
                        self.twist.angular.z = w_constant
                else:
                        self.twist.linear.x = -v_constant * min(self.ranges)

                ##################################################################

                self.cmd_vel_pub.publish(self.twist)  #command the velocities to the robot
                rospy.loginfo(yaw)

        def odom_callback(self,msg):
                
                #function to convert quaternions to roll pitch yaw
                #only the yaw value is used for the control
                global roll, pitch, yaw
                orientation_q = msg.pose.pose.orientation
                orientation = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (roll, pitch, yaw) = euler_from_quaternion (orientation)
                self.yaw_pub.publish(yaw)   #publish yaw to plot the profile

                
rospy.init_node('auto_explorer')
explorer = Explorer()
rospy.spin()