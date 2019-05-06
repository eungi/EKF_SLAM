#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import message_filters
from sensor_msgs.msg import Image
from mi_msgs.msg import Car
from mi_msgs.msg import Car_LGIT

import visualization as vs
import EKF_SLAM

from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

import numpy as np

slam_pub = rospy.Publisher("SLAM", Image, queue_size=1)

vehicle_poses = []
vehicle_poses_ = []
target_origin_poses = []
targets_origin = []

def avm_changing_coordinate(target1, target2, heading) :
	global movement
	heading_ = heading * (3.141592 / 180)
	target1_ = [target1[1]*np.cos(heading_) - target1[0]*np.sin(heading_), target1[1]*np.sin(heading_) + target1[0]*np.cos(heading_)]
	target2_ = [target2[1]*np.cos(heading_) - target2[0]*np.sin(heading_), target2[1]*np.sin(heading_) + target2[0]*np.cos(heading_)]
	result = [(target1_[0]+target2_[0])/2, (target1_[1]+target2_[1])/2]

	if len(targets_origin) is 0 :
		targets_origin.append(result)

		tg1 = [target1_[0] - result[0], target1_[1] - result[1]]
		tg2 = [target2_[0] - result[0], target2_[1] - result[1]]

		target_origin_poses.append(tg1)
		target_origin_poses.append(tg2)
	
	if len(vehicle_poses) is 0 :
		pass
	else :
		mv = [(vehicle_poses[len(vehicle_poses)-1][0] - vehicle_poses[len(vehicle_poses)-2][0]), (vehicle_poses[len(vehicle_poses)-1][1] - vehicle_poses[len(vehicle_poses)-2][1])]
		movement = [movement[0] - mv[0], movement[1] - mv[1]]

	target1_ = [target1_[0] + movement[0], target1_[1] + movement[1]]
	target2_ = [target2_[0] + movement[0], target2_[1] + movement[1]]
	
	target1_ = [target1_[0] - result[0], target1_[1] - result[1]]
	target2_ = [target2_[0] - result[0], target2_[1] - result[1]]
	
	if len(vehicle_poses) is 0 :
		pass
	else :
		mv = [(vehicle_poses[len(vehicle_poses)-1][0] - vehicle_poses[0][0]), (vehicle_poses[len(vehicle_poses)-1][1] - vehicle_poses[0][1])]
	
		changed_target1 = [target1_[0] + mv[0], target1_[1] + mv[1]]
		changed_target2 = [target2_[0] + mv[0], target2_[1] + mv[1]]

	return changed_target1, changed_target2

def msg_callback(car_state, target_info):
	global mu_t_1
	global sigma_t_1
	global N

	#rospy.loginfo(car_state)
	vehicle_pose = [car_state.PosX, car_state.PosY]
	velocity = car_state.velocity
	heading = car_state.heading
	heading_ = (heading + 180) * (3.141592 / 180)
	target1 = [target_info.targetPosX1, target_info.targetPosY1]
	target2 = [target_info.targetPosX2, target_info.targetPosY2]

	vehicle_poses.append(vehicle_pose)

	target1_, target2_ = avm_changing_coordinate(target1, target2, heading)

	t_state = vs.draw_t(vehicle_pose, heading_, target1_, target2_)
	vs.draw_path(t_state, vehicle_poses)
	vs.draw_origin_point(t_state, target_origin_poses[0], target_origin_poses[1])

	# Todo : Add EKF-SLAM
	dt = car_state.yaw_rate_dt  # Not yet
	v_t_1 = velocity
	w_t_1 = car_state.yaw_rate  # Not yet
	x_t_1 = vehicle_pose[0]
	y_t_1 = vehicle_pose[1]
	theta_t_1 = heading
	map_x1 = target1_[0]
	map_y1 = target1_[1]
	map_id1 = 1  # fix
	map_x2 = target2_[0]
	map_y2 = target2_[1]
	map_id2 = 2  # fix

	u_t = np.array([v_t_1, w_t_1])
	z_t = np.array([[map_x1, map_x2],
		        [map_y1, map_y2],
		        [map_id1, map_id2]])
	c_t = [map_id1, map_id2]  # fix
	# update/

	mu_t, sigma_t = EKF_SLAM.EKF_SLAM(mu_t_1, sigma_t_1, u_t, z_t, c_t, dt, N)

	mu_t_1 = mu_t
        sigma_t_1 = sigma_t

	slam_pub.publish(bridge.cv2_to_imgmsg(t_state, "bgr8"))

def main() :
	print("EKF-SLAM Start")
	
	# initialization
	global N
	global movement
	N = 2
	movement = [0, 0]
	dt = 0.02

	mu_0 = np.transpose([np.zeros(2*N + 3)])
	sigma_0 = np.inf*np.ones((2*N + 3, 2*N + 3))
	sigma_0[:3, :3] = np.zeros((3, 3))

	global mu_t_1
	global sigma_t_1
	mu_t_1 = mu_0
	sigma_t_1 = sigma_0

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
