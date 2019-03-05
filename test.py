#! /usr/bin/env python
from graspit_commander import GraspitCommander
import geometry_msgs
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
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes, Grasp
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from graspit_commander.graspit_exceptions import (
    LoadWorldException,
    ImportObstacleException,
    ImportRobotException,
    ImportGraspableBodyException,
)


# Tools for grasping
class GraspingClient(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")
	self.model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState) 
    
    def moveArm(self, pose):
	gripper_frame = 'wrist_roll_link'
	gripper_pose = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.17, -0.693, -.242, 0.657))
	gripper_pose_stamped = PoseStamped()
	gripper_pose_stamped.header.frame_id = 'base_link'
	gripper_pose_stamped.header.stamp = rospy.Time.now()
	gripper_pose_stamped.pose = gripper_pose #change to pose
	self.move_group.moveToPose(gripper_pose_stamped, gripper_frame)
	result = move_group.get_move_action().get_result()  
	return result
	
    def pick(self, grasps_to_try):
	return self.pickplace.pickup("demo_cube", grasps_to_try, support_name="table1_surface")

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
	    gc = GraspingClient()
	    gc.findAndAdd("table1") 
	    gc.findAndAdd("demo_cube")

	    pose = geometry_msgs.msg.Pose()
	    pose.orientation.y = 0.7071
	    pose.orientation.w = 0.7071

	    pose2 = geometry_msgs.msg.Pose()
	    pose2.position.x = 0.5
	    pose2.position.y = 0.5
	    pose2.position.z = .5
	
	    pose3 = geometry_msgs.msg.Pose()
	    pose3.position.x = -2
	    pose3.position.y = -2
	    pose3.position.z = -0.09

	    print("clearing world")
	    GraspitCommander.clearWorld()
	    GraspitCommander.importRobot("fetch_gripper", pose2)
	    GraspitCommander.importObstacle("floor", pose3)
	    GraspitCommander.importGraspableBody("longBox", pose)
	    print("loading world") 
	    result = GraspitCommander.planGrasps(max_steps=5000)
	    
	    grasp = None
	    for g in result.grasps:
		if g.dofs[0] > 1.0:
		    grasp = g
		    break
	    print(grasp) #end effector pose in graspit world frame 
	    pick = gc.moveArm(grasp.pose) 
	    #pick = gc.pick([g, ])
	    #print(pick)	    

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


if __name__ == '__main__':
    tuto = Tutorial()
    tuto.show_gazebo_models()
