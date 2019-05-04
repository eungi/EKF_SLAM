#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import message_filters
from sensor_msgs.msg import Image
from mi_msgs.msg import Car
from mi_msgs.msg import Car_LGIT

import visualization as vs
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

slam_pub = rospy.Publisher("SLAM", Image, queue_size=1)

vehicle_poses = []
target1_criterion = []
target2_criterion = []

def avm_changing_coordinate(criterion, pose) :
	return (changed_x, changed_y)

def msg_callback(car_state, target_info):
	#rospy.loginfo(car_state)
	vehicle_pose = [car_state.PosX, car_state.PosY]
	velocity = car_state.velocity
	heading = car_state.heading
	target1 = [target_info.targetPosX1, target_info.targetPosY1]
	target2 = [target_info.targetPosX2, target_info.targetPosY2]
	#print('target1 : ', target1, '       target2 : ', target2)

	vehicle_poses.append(vehicle_pose)
	if len(target1_criterion) is 0 and len(target2_criterion) is 0 :
		target1_criterion.append(target1)
		target2_criterion.append(target2)

	t_state = vs.draw_t(vehicle_pose, heading, target1, target2)
	vs.draw_path(t_state, vehicle_poses)

	# Todo : Add EKF-SLAM

	slam_pub.publish(bridge.cv2_to_imgmsg(t_state, "bgr8"))

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
