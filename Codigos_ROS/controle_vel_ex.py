#!/usr/bin/env python

import rospy
import numpy
from turtlesim.msg import Pose
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import cos, sin, pi

x0 = 0.0
y0 = 0.0
T = 0.0
k = 1
d = 1
Err = 0.1
vel_lim = Twist()
vel_lim.linear.x = 3
vel_lim.angular.z = 3

def callback(data):
	global x0, y0, T, k, d
	x0 = data.x + d * cos(data.theta)
	y0 = data.y + d * sin(data.theta)
	T = data.theta

def lasercallback(data):
	global pontos, increment, l_min
	pontos = []
	pontos = data.ranges
	increment = data.angle_increment*180/pi

def talker():
	rospy.init_node('controle_vel', anonymous=True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	rospy.Subscriber('/pose', Pose2D , callback)
	rospy.Subscriber('/scan', LaserScan, lasercallback)
	rate = rospy.Rate(20)
	vel_msg = Twist()
	while not rospy.is_shutdown():
		xf, yf = raw_input('Digite a coordenada (x,y) desejada: ').split()
		xf, yf = [float(xf) for xf in [xf, yf]]
		if xf > 17 or xf < 0 or yf > 17 or yf < 0:
			rospy.loginfo('\033[31 mCoordenadas invalidas , digite novamente ...\033[m')
			continue
		lim = 0
		while abs(xf - x0) > Err or abs(yf - y0) > Err:
			vel_msg.linear.x = k * ( cos(T) * (xf - x0) + sin(T) * (yf - y0) )
			vel_msg.angular.z = k * (-(sin(T) * (xf - x0)) / d + (cos(T) * (yf - y0)) / d)
			if vel_msg.linear.x > vel_lim.linear.x:
				vel_msg.linear.x = vel_lim.linear.x
			elif vel_msg.linear.x < -vel_lim.linear.x:
				vel_msg.linear.x = -vel_lim.linear.x
			if vel_msg.angular.z > vel_lim.angular.z:
				vel_msg.angular.z = vel_lim.angular.z
			elif vel_msg.angular.z < -vel_lim.angular.z:
				vel_msg.angular.z = -vel_lim.angular.z
			pub.publish(vel_msg)
			rate.sleep()
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass