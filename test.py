#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
import rospy

import copy
import actionlib

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Tools for grasping
class GraspingClient(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")
	#self.scene.removeCollisionObject("box1", False)
	#self.scene.removeCollisionObject("table1", False)
	#self.scene.addBox("box1", 1, 1, 1, 1, 1, 0)
	self.model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState) 

	
    def pick(self):
	grasps_to_try = [] #from graspit! 
	self.pickplace("demo_cube", grasps_to_try)

    def findAndAdd(self, blockName):
	self.robot_coords = self.model_coordinates("fetch", "")
	if blockName == "table1":
		print("Adding table1 to scene")
		resp_coordinates = self.model_coordinates(blockName, "link")
		x = resp_coordinates.pose.position.x - self.robot_coords.pose.position.x
		y = resp_coordinates.pose.position.y - self.robot_coords.pose.position.y
		z = resp_coordinates.pose.orientation.z	
		self.scene.addBox("table1", .56, .56, .04, x, y, 0.02)
		self.scene.addBox("table1_surface", .91, .91, 0.04, x, y, .755)	
		self.scene.addBox("table1_column", .042, .042, .74, x, y, .37)	
	
	elif blockName == "demo_cube":
		print("Adding longBox to scene")
		resp_coordinates = self.model_coordinates(blockName, "link")
		x = resp_coordinates.pose.position.x - self.robot_coords.pose.position.x
		y = resp_coordinates.pose.position.y - self.robot_coords.pose.position.y
		z = resp_coordinates.pose.position.z
		self.scene.addBox("demo_cube", .044, .044, .18, x, y, z)

class Tutorial:
    def show_gazebo_models(self):
        rospy.init_node("demo")
	# Make sure sim time is working
    	while not rospy.Time.now():
	        pass

	try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            blockName = "fetch"
            resp_coordinates = model_coordinates(blockName, "")
            print 'Status.success = ', resp_coordinates.success
  	    print("X value = " + str(resp_coordinates.pose.position.x))
   	    print("Y value = " + str(resp_coordinates.pose.position.y))
      	    print("Z value = " + str(resp_coordinates.pose.position.z))
	    print("Quat x: " + str(resp_coordinates.pose.orientation.x))

	    gc = GraspingClient()
	    gc.findAndAdd("table1") 
	    gc.findAndAdd("demo_cube")
	    #gc.updateScene()

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


if __name__ == '__main__':
    tuto = Tutorial()
    tuto.show_gazebo_models()
