#!/usr/bin/env python
#-*- coding:utf-8 -*-

import cv2
import numpy as np

HEIGHT = 400
WIDTH = 400

red = (0, 0, 255)
green = (0, 255, 0)
blue = (255, 0, 0)
orange = (0, 155, 255)
white = (255, 255, 255)

def changing_visualization_coordinate(pose) :
	changed_x = pose[0]*20*-1 + HEIGHT / 2
	changed_y = pose[1]*20*-1 + WIDTH / 2

	return (int(changed_x), int(changed_y))

def draw_t(vehicle_pose, heading, target1, target2) :
	#heading = (heading + 180) * (3.141592 / 180)
	#heading_ = (heading + 90) * (3.141592 / 180)
	basic_world = np.zeros((HEIGHT, WIDTH, 3), np.uint8)

	# 원점 표시
	cv2.line(basic_world, changing_visualization_coordinate((-0.5, 0)), changing_visualization_coordinate((0.5, 0)), white, 1)
	cv2.line(basic_world, changing_visualization_coordinate((0, -0.5)), changing_visualization_coordinate((0, 0.5)), white, 1)

	# 타겟 정보
	cv2.circle(basic_world, changing_visualization_coordinate(target1), 2, red, -1)
	cv2.circle(basic_world, changing_visualization_coordinate(target2), 2, orange, -1)

	# 차량 정보
	cv2.circle(basic_world, changing_visualization_coordinate(vehicle_pose), 3, blue, -1)
	r = 4.5
	end_point = ((-1*r*np.cos(heading))+vehicle_pose[0], (r*np.sin(heading)+vehicle_pose[1]))
	cv2.line(basic_world, changing_visualization_coordinate(vehicle_pose), changing_visualization_coordinate(end_point), blue, 2)

	return basic_world

def draw_path(basic_world, poses) :
	if len(poses) is 1 :
		pass
	else :
		for i in range(len(poses) - 1) :
			cv2.line(basic_world, changing_visualization_coordinate(poses[i]), changing_visualization_coordinate(poses[i+1]), green, 1)

def draw_origin_point(basic_world, target1, target2) :
	cv2.circle(basic_world, changing_visualization_coordinate(target1), 4, white, 0)
	cv2.circle(basic_world, changing_visualization_coordinate(target2), 4, white, 0)
