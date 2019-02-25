#! /usr/bin/env python 

import math 
import fetch_api
import rospy 

def wait_for_time():
	while rospy.Time().now().to_sec() == 0:
		pass

def print_usage():
	print "rosrun fetch_gazebo motion_demo.py move 1.5"

def main():
	rospy.init_node('motion_demo')
	wait_for_time()
	argv = rospy.myargv()
	if len(argv) < 3:
		print_usage()
		return
	command = argv[1]
	value = float(argv[2])
	
	base = fetch_api.Base()
	gripper = fetch_api.Gripper()
	effort = gripper.MAX_EFFORT	
	head = fetch_api.Head()
	torso = fetch_api.Torso()
	arm = fetch_api.Arm()

	head.pan_tilt(0,0)
	torso.set_height(0)
	gripper.open() 

	base.go_forward(1.5) 
	head.pan_tilt(0, 0.78) #down 45 deg 
	head.pan_tilt(0, -1.56) #up 90 deg 
	head.pan_tilt(-1.56, 0) #left 90 deg
	head.pan_tilt(1.56, 0) #right 90 deg 
	head.pan_tilt(0, 0) # forward 90 deg 
	torso.set_height(0.4)
	# move arm to join values 
	arm.move_to_joints(fetch_api.ArmJoints.from_list([0, 0, 0, 0, 0, 0, 0]))
	gripper.close(effort)
	gripper.open()		

if __name__ == '__main__':
	main()

