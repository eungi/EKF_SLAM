#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy

def main() :
	print("EKF-SLAM Start")
	# heading = degree

	rospy.init_node('EKF-SLAM')
	rate = rospy.Rate(2)	#20ms
	while not rospy.is_shutdown():
		#print('aa')
		rate.sleep()

if __name__ == '__main__':
	try :
		main()
	except rospy.ROSInterruptException :
		pass
