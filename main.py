#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import message_filters
from sensor_msgs.msg import Image
from mi_msgs.msg import Car
from mi_msgs.msg import Car_LGIT

import visualization as vs
import EKF_SLAM

import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

import numpy as np

slam_pub = rospy.Publisher("SLAM", Image, queue_size=1)

vehicle_poses = []
target_origin_poses = []
targets_origin = []

target1s = []
target2s = []

N = 2
it = 0

red = (0, 0, 255)
green = (0, 255, 0)
blue = (255, 0, 0)
orange = (0, 155, 255)
yello = (0, 228, 255)
white = (255, 255, 255)

global max_tg1
max_tg1 = [0, 0, 0]
global min_tg1
min_tg1 = [100, 0, 0]
global max_tg2
max_tg2 = [0, 0, 0]
global min_tg2
min_tg2 = [100, 0, 0]

def changing_avm_coordinate(vehicle_pose_, heading_, target1_, target2_) :
	heading = np.deg2rad(-heading_)
	heading__ = np.deg2rad(heading_)

	x_t_1 = -1 * (vehicle_pose_[0] + np.cos(heading)*(2.7/2))
	y_t_1 = vehicle_pose_[1] + np.sin(heading)*(2.7/2)
	vehicle_pose = [x_t_1, y_t_1]

	"""
	map_x1 = np.sqrt(pow(x_t_1 - target1_[1], 2) + pow(y_t_1 - target1_[0], 2)) * np.cos(np.arctan2(target1_[0]-y_t_1, target1_[1]-x_t_1) - heading)
	map_y1 = np.sqrt(pow(x_t_1 - target1_[1], 2) + pow(y_t_1 - target1_[0], 2)) * np.sin(np.arctan2(target1_[0]-y_t_1, target1_[1]-x_t_1) - heading)
	target1 = [map_x1, map_y1]

	map_x2 = np.sqrt(pow(x_t_1 - target2_[1], 2) + pow(y_t_1 - target2_[0], 2)) * np.cos(np.arctan2(target2_[0]-y_t_1, target2_[1]-x_t_1) - heading)
	map_y2 = np.sqrt(pow(x_t_1 - target2_[1], 2) + pow(y_t_1 - target2_[0], 2)) * np.sin(np.arctan2(target2_[0]-y_t_1, target2_[1]-x_t_1) - heading)
	target2 = [map_x2, map_y2]
	"""

	target1 = [target1_[1] * np.cos(heading__) - target1_[0] * np.sin(heading__),
		    target1_[1] * np.sin(heading__) + target1_[0] * np.cos(heading__)]
	target2 = [target2_[1] * np.cos(heading__) - target2_[0] * np.sin(heading__),
		    target2_[1] * np.sin(heading__) + target2_[0] * np.cos(heading__)]
	#result = [(target1_[0] + target2_[0]) / 2, (target1_[1] + target2_[1]) / 2]

	#tg1 = [target1_[0] - result[0], target1_[1] - result[1]]
	#tg2 = [target2_[0] - result[0], target2_[1] - result[1]]

	if len(vehicle_poses) is 0 :
		pass
	else :
		moment_x = vehicle_poses[len(vehicle_poses)-1][0] - vehicle_poses[0][0]
		moment_y = vehicle_poses[len(vehicle_poses)-1][1] - vehicle_poses[0][1]

		target1 = [target1[0] + moment_x, target1[1] + moment_y]
		target2 = [target2[0] + moment_x, target2[1] + moment_y]

	target1s.append(target1)
	target2s.append(target2)

	return vehicle_pose, heading, target1, target2

def FVC(pose) :	# fix_visualization_coordinate
	fix_pose = [pose[0] - targets_origin[0][0], pose[1] - targets_origin[0][1]]

	return fix_pose

def msg_callback(car_state, target_info):
	global mu_0
	global mu_t_1
	global sigma_t_1
	global N
	global it

	global max_tg1
	global min_tg1
	global max_tg2
	global min_tg2

	# Parameter initialization
	vehicle_pose_ = [car_state.PosX, car_state.PosY]
	heading_ = car_state.heading
	target1_ = [target_info.targetPosX1, target_info.targetPosY1]
	target2_ = [target_info.targetPosX2, target_info.targetPosY2]
	velocity_ = car_state.velocity
	dt = car_state.yaw_rate_dt
	yaw_rate = car_state.yaw_rate

	vehicle_pose, heading, target1, target2 = changing_avm_coordinate(vehicle_pose_, heading_, target1_, target2_)

	if len(targets_origin) is 0 :
		result = [(target1[0] + target2[0]) / 2, (target1[1] + target2[1]) / 2]
		targets_origin.append(result)
		
		tg1 = [target1[0] - result[0], target1[1] - result[1]]
		tg2 = [target2[0] - result[0], target2[1] - result[1]]

		target_origin_poses.append(tg1)
		target_origin_poses.append(tg2)
	vehicle_poses.append(vehicle_pose)

	# EKF-SLAM
	print('\n\n********************\n[%d] iteration\n********************' % it)

	if it == 0 :	# EKF-SLAM initialization
		mu_t_1, sigma_t_1 = EKF_SLAM.initialization(N, car_state, target_info, vehicle_pose)

	it = it + 1

	v_t_1 = velocity_ / 3.6
	w_t_1 = yaw_rate + 1e-10
	w_t_1 = np.deg2rad(w_t_1)
	map_id1 = 1
	map_id2 = 2

	u_t = np.array([v_t_1, w_t_1])

	# check
	z_t = np.array([[np.sqrt(pow(target1[0]-vehicle_pose[0], 2)+pow(target1[1]-vehicle_pose[1], 2)),
		    np.sqrt(pow(target2[0]-vehicle_pose[0], 2)+pow(target2[1]-vehicle_pose[1], 2))],
		    [np.arctan2(target1[1]-vehicle_pose[1], target1[0]-vehicle_pose[0]) - heading,
		     np.arctan2(target2[1]-vehicle_pose[1], target2[0]-vehicle_pose[0]) - heading],
		    [map_id1, map_id2]])
	# check/
	c_t = [map_id1, map_id2]  # fix
	y_t = np.array([[vehicle_pose[0]], [vehicle_pose[1]], [heading], [target1[0]], [target1[1]], [target2[0]], [target2[1]]])
	# update/

	mu_t, sigma_t = EKF_SLAM.EKF_SLAM(mu_t_1, sigma_t_1, u_t, z_t, c_t, dt, N)

	# test
	print('\nreal state >>')
	print(y_t)
	print('\nmu_t >>')
	print(mu_t)
	print('\nsigma_t >>')
	print(sigma_t)
	# test/

	mu_t_1 = mu_t
	sigma_t_1 = sigma_t

	ekf_pose = mu_t[0:3]
	ekf_target1 = mu_t[3:5]
	ekf_target2 = mu_t[5:7]
	# EKF SLAM/

	# < For visualization >
	# raw
	t_state = vs.draw_t(vehicle_pose, heading, FVC(target1), FVC(target2))
	vs.draw_path(t_state, vehicle_poses, yello)
	vs.draw_point(t_state, target_origin_poses[0], target_origin_poses[1], white, 3, 0)
	# EKF-SLAM
	#vs.draw_point(t_state, FVC(ekf_target1), FVC(ekf_target2), green, 2, -1)
	#vs.draw_vehicle(t_state, FVC(ekf_pose[0:2]), FVC(ekf_pose[2]), red, 2)

	if len(target1s) is 0 :
		pass
	else :
		target1 = np.array(target1)
		target1 = np.array(target1)

		tg1_np = np.sqrt((target_origin_poses[0][0] - FVC(target1)[0])*target_origin_poses[0][0] - FVC(target1)[0]+(target_origin_poses[0][1] - FVC(target1)[1])*(target_origin_poses[0][1] - FVC(target1)[1]))
		tg2_np = np.sqrt((target_origin_poses[1][0] - FVC(target2)[0])*(target_origin_poses[1][0] - FVC(target2)[0])+(target_origin_poses[1][1] - FVC(target2)[1])*(target_origin_poses[1][1] - FVC(target2)[1]))

		if max_tg1[0] < tg1_np :
			max_tg1 = [tg1_np, target1[0], target1[1]]
		if max_tg2[0] < tg2_np :
			max_tg2 = [tg2_np, target2[0], target2[1]]
		if min_tg1[0] > tg1_np :
			min_tg1 = [tg1_np, target1[0], target1[1]]
		if min_tg2[0] > tg2_np :
			min_tg2 = [tg2_np, target2[0], target2[1]]
		
		print('\n[target 1]')
		print('-max_target-')
		print(max_tg1)
		print('-min_target-')
		print(min_tg1)

		print('\n[target 2]')
		print('-max_target-')
		print(max_tg2)
		print('-min_target-')
		print(min_tg2)

		vs.draw_point(t_state, FVC(max_tg1[1:3]), FVC(min_tg1[1:3]), red, 3, 0)
		vs.draw_point(t_state, FVC(max_tg2[1:3]), FVC(min_tg2[1:3]), red, 3, 0)
		

	slam_pub.publish(bridge.cv2_to_imgmsg(cv2.flip(t_state, 1), "bgr8"))

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
