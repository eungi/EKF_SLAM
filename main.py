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

ekf_slam_pub = rospy.Publisher("EKF_SLAM", Image, queue_size=1)
slam_pub = rospy.Publisher("SLAM", Image, queue_size=1)

vehicle_poses_ = []
vehicle_poses = []
target_origin_poses = []
targets_origin = []

target1s = []
target2s = []

slam_vehicle_poses = []
avm_vehicle_poses = []

N = 2
it = 0

red = (0, 0, 255)
green = (0, 255, 0)
blue = (255, 0, 0)
orange = (0, 155, 255)
yello = (0, 228, 255)
white = (255, 255, 255)
magenta = (255, 0, 255)
skyblue = (255, 216, 0)

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
	vehicle_pose__ = [-1*car_state.PosX, car_state.PosY]
	vehicle_poses_.append(vehicle_pose__)
	heading_ = car_state.heading
	target1_ = [target_info.targetPosX1, target_info.targetPosY1]
	target2_ = [target_info.targetPosX2, target_info.targetPosY2]
	velocity_ = car_state.velocity
	dt = car_state.yaw_rate_dt
	yaw_rate = car_state.yaw_rate

	vehicle_pose, heading, target1, target2 = changing_avm_coordinate(vehicle_pose_, heading_, target1_, target2_)

	avm_vehicle_pose, avm_heading, t1, t2 = changing_avm_coordinate([target_info.PosX, target_info.PosY], target_info.heading, [0,0], [0,0])
	avm_vehicle_poses.append(avm_vehicle_pose)

	if len(targets_origin) is 0 :
		result = [(target1[0] + target2[0]) / 2, (target1[1] + target2[1]) / 2]
		targets_origin.append(result)
		
		tg1 = [target1[0] - result[0], target1[1] - result[1]]
		tg2 = [target2[0] - result[0], target2[1] - result[1]]

		target_origin_poses.append(tg1)
		target_origin_poses.append(tg2)
	vehicle_poses.append(vehicle_pose)

	# EKF SLAM
	print('\n\n********************\n[%d] iteration\n********************' % it)

	if it == 0:
		# initialization
		mu_t_1, sigma_t_1 = EKF_SLAM.initialization(N, avm_vehicle_pose, avm_heading, target1, target2)

		mu_0 = mu_t_1
		# initialization/

	it = it + 1
	# update
	dt = car_state.yaw_rate_dt  # 0.02  # car_state.sampling_time  # Not yet
	v_t_1 = car_state.velocity / 3.6
	w_t_1 = car_state.yaw_rate + 1e-10  # 1 + 0.05 * np.random.randn() + 0  # car_state.yaw_rate  # Not yet
	w_t_1 = np.deg2rad(w_t_1)
	map_id1 = 1  # fix
	map_id2 = 2  # fix

	# eg coord.
	x_t_1 = vehicle_pose[0]
	y_t_1 = vehicle_pose[1]
	theta_t_1 = heading
	map_x1 = FVC(target1)[0]
	map_y1 = FVC(target1)[1]
	map_x2 = FVC(target2)[0]
	map_y2 = FVC(target2)[1]
	# eg coord./

	u_t = np.array([v_t_1, w_t_1])
	z_t = np.array([[np.sqrt(pow(map_x1-x_t_1, 2)+pow(map_y1-y_t_1, 2)),
		    np.sqrt(pow(map_x2-x_t_1, 2)+pow(map_y2-y_t_1, 2))],
		    [np.arctan2(map_y1-y_t_1, map_x1-x_t_1) - theta_t_1,
		     np.arctan2(map_y2-y_t_1, map_x2-x_t_1) - theta_t_1],
		    [map_id1, map_id2]])
	# check/
	c_t = [map_id1, map_id2]  # fix
	y_t = np.array([[x_t_1], [y_t_1], [theta_t_1], [map_x1], [map_y1], [map_x2], [map_y2]])
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

	# update
	mu_t_1 = mu_t
	sigma_t_1 = sigma_t
	# update/

	# EKF SLAM/

	ekf_pose = mu_t[0:3]
	ekf_target1 = mu_t[3:5]
	ekf_target2 = mu_t[5:7]

	slam_vehicle_poses.append(ekf_pose)
	# EKF SLAM/

	# < For visualization >
	# raw
	t_state = vs.draw_t(vehicle_pose, heading, FVC(target1), FVC(target2), blue, skyblue)
	#vs.draw_path(t_state, vehicle_poses_, yello)
	vs.draw_path(t_state, vehicle_poses, green)
	vs.draw_point(t_state, target_origin_poses[0], target_origin_poses[1], white, 3, 0)
	# raw + EKF-SLAM
	vs.draw_path(t_state, slam_vehicle_poses, yello)
	vs.draw_point(t_state, ekf_target1, ekf_target2, magenta, 2, -1)
	vs.draw_vehicle(t_state, ekf_pose[0:2], ekf_pose[2], red, 2)
	vs.draw_vehicle(t_state, avm_vehicle_pose, avm_heading, orange, 2)
	vs.draw_path(t_state, avm_vehicle_poses, blue)
	# Only EKF-SLAM result
	slam_coord_system = vs.draw_t(ekf_pose[0:2], ekf_pose[2], ekf_target1, ekf_target2, red, magenta)
	vs.draw_path(slam_coord_system, slam_vehicle_poses, yello)
	vs.draw_point(slam_coord_system, target_origin_poses[0], target_origin_poses[1], white, 3, 0)
	# AVM raw
	vs.draw_vehicle(slam_coord_system, avm_vehicle_pose, avm_heading, orange, 2)
	vs.draw_path(slam_coord_system, avm_vehicle_poses, blue)

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

		#vs.draw_point(t_state, FVC(max_tg1[1:3]), FVC(min_tg1[1:3]), red, 3, 0)
		#vs.draw_point(t_state, FVC(max_tg2[1:3]), FVC(min_tg2[1:3]), red, 3, 0)
		

	slam_pub.publish(bridge.cv2_to_imgmsg(cv2.flip(t_state, 1), "bgr8"))
	ekf_slam_pub.publish(bridge.cv2_to_imgmsg(cv2.flip(slam_coord_system, 1), "bgr8"))

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
