#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import message_filters
from mi_msgs.msg import Car
from mi_msgs.msg import Car_LGIT

def msg_callback(car_state, target_info):
	#rospy.loginfo(car_state.velocity)
	velocity = car_state.velocity
	heading = car_state.heading
	target1 = [target_info.targetPosX1, target_info.targetPosY1]
	target2 = [target_info.targetPosX2, target_info.targetPosY2]

def main() :
	print("EKF-SLAM Start")

	rospy.init_node('EKF_SLAM')
	car_messages = message_filters.Subscriber("car_messages", Car)
	car_LGIT_messages = message_filters.Subscriber("car_LGIT_messages", Car_LGIT)

	ts = message_filters.ApproximateTimeSynchronizer([car_messages, car_LGIT_messages], 1, 1, allow_headerless=True)
	ts.registerCallback(msg_callback)

	rospy.spin()

if __name__ == '__main__':
	try :
		main()
	except rospy.ROSInterruptException :
		pass
