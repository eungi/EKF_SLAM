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
yello = (0, 228, 255)
white = (255, 255, 255)

def changing_visualization_coordinate(pose) :
	changed_x = pose[0]*20*-1 + HEIGHT / 2
	changed_y = pose[1]*20*-1 + WIDTH / 2# + 50

	return (int(changed_x), int(changed_y))

def draw_t(vehicle_pose, heading, target1, target2) :
	basic_world = np.zeros((HEIGHT, WIDTH, 3), np.uint8)

	cv2.line(basic_world, changing_visualization_coordinate((-0.5, 0)), changing_visualization_coordinate((0.5, 0)), white, 1)
	cv2.line(basic_world, changing_visualization_coordinate((0, -0.5)), changing_visualization_coordinate((0, 0.5)), white, 1)

	draw_point(basic_world, target1, target2, orange, 2, -1)

	draw_vehicle(basic_world, vehicle_pose, heading, blue, 2)

	return basic_world

def draw_path(basic_world, poses, color) :
	if len(poses) is 1 :
		pass
	else :
		for i in range(len(poses) - 1) :
			cv2.line(basic_world, changing_visualization_coordinate(poses[i]), changing_visualization_coordinate(poses[i+1]), color, 1)

def draw_point(basic_world, target1, target2, color, size, option = -1) :
	cv2.circle(basic_world, changing_visualization_coordinate(target1), size, color, option)
	cv2.circle(basic_world, changing_visualization_coordinate(target2), size, color, option)

def draw_vehicle(basic_world, vehicle_pose, heading, color, line_size) :
	cv2.circle(basic_world, changing_visualization_coordinate(vehicle_pose), 3, color, -1)
	r = 2.25
	start_point = ((r*np.cos(heading))+vehicle_pose[0], (-1*r*np.sin(heading)+vehicle_pose[1]))
	end_point = ((-1*r*np.cos(heading))+vehicle_pose[0], (r*np.sin(heading)+vehicle_pose[1]))
	cv2.line(basic_world, changing_visualization_coordinate(vehicle_pose), changing_visualization_coordinate(end_point), color, line_size)
	cv2.line(basic_world, changing_visualization_coordinate(vehicle_pose), changing_visualization_coordinate(start_point), color, line_size)
