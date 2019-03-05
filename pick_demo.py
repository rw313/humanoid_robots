#!/usr/bin/env python

'''
resources used: https://answers.ros.org/question/261782/how-to-use-getmodelstate-service-from-gazebo-in-python/ for using getModelState 
'''
import copy
import actionlib
import rospy
import fetch_api
from math import sin, cos, fabs
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv import GetModelState
from graspit_commander import GraspitCommander


# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Tools for grasping
class GraspingClient(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")

        #self.find_client.wait_for_server()
	self.model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

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
                self.scene.addBox("demo_cube", .0442, .0442, .181, x, y, z)


        self.scene.waitForSync()

    def moveArm(self, pose):
	self.robot_coords = self.model_coordinates("fetch", "")
	gripper_frame = 'wrist_roll_link'
	gripper_pose_stamped = PoseStamped()
	
        gripper_pose_stamped.header.frame_id = 'base_link'
        gripper_pose_stamped.header.stamp = rospy.Time.now()
	
	resp_coordinates = self.model_coordinates("demo_cube", "link")
        x = resp_coordinates.pose.position.x - self.robot_coords.pose.position.x +(pose.position.x) #should be less than .7 for reachability 
        y = resp_coordinates.pose.position.y - self.robot_coords.pose.position.y + (pose.position.y)
        z = resp_coordinates.pose.position.z + (pose.position.z) #should be around 0.87

	new_pose = Pose(Point(x, y, z), pose.orientation)
	print("new pose: " )
	print(new_pose)
	gripper_pose_stamped.pose = new_pose
	self.move_group.moveToPose(gripper_pose_stamped, gripper_frame) 
	result = self.move_group.get_move_action().get_result()
	self.scene.waitForSync()

	if result:
		return result.error_code.val == MoveItErrorCodes.SUCCESS
	return False
 
    def getSupportSurface(self, name):
        for surface in self.support_surfaces:
            if surface.name == name:
                return surface
        return None

    def getPlaceLocation(self):
        pass

    def pick(self, block, grasps):
        success, pick_result = self.pickplace.pick_with_retry(block.name,
                                                              grasps,
                                                              support_name=block.support_surface,
                                                              scene=self.scene)
        self.pick_result = pick_result
        return success

    def place(self, block, pose_stamped):
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = pose_stamped.header.frame_id

        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        # create another several places, rotate each by 360/m degrees in yaw direction
        m = 16 # number of possible place poses
        pi = 3.141592653589
        for i in range(0, m-1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(l))

        success, place_result = self.pickplace.place_with_retry(block.name,
                                                                places,
                                                                scene=self.scene)
        return success

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

if __name__ == "__main__":
    rospy.init_node("demo")
    while not rospy.Time.now():
        pass

    move_base = MoveBaseClient()
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()
    gc = GraspingClient()
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    gripper.open()
    #gc.tuck()
    rospy.loginfo("Moving to table...")
    #move_base.goto(2.250, 3.118, 0.0)
    #move_base.goto(3.100, 3.118, 0.0)

    rospy.loginfo("Raising torso...")
    torso_action.move_to([0.4, ])

    gc.findAndAdd("table1")
    gc.findAndAdd("demo_cube")

    gic = GraspitCommander()
    gic.clearWorld()
    gic.importObstacle("floor", Pose(Point(-2, -2, -0.09), Quaternion(0,0,0,1)))
    gic.importGraspableBody("longBox", Pose(Point(0,0,0),Quaternion(0,.7071,0,.7071)))
    gic.importRobot("fetch_gripper")
    result = gic.planGrasps(max_steps=50000)
    saved_pose = None
    for grasp in result.grasps:
	if grasp.dofs[0] < 1.05:
		continue
    	print("Found grasp: ")
    	print(grasp)
    	is_valid = gc.moveArm(grasp.pose)
	if is_valid:
		saved_pose = grasp.pose
    		break
    gripper.close(gripper.MAX_EFFORT)
    print("Closed gripper") 
    gc.moveArm(Pose(saved_pose.position, Quaternion(0, .7071, 0, .7071)) 
    #move_base.goto(gc.robot_coords.pose.position.x+0.1, gc.robot_coords.pose.position.y, 0)
    gc.scene.waitForSync()

    print("moving back to place")
    gc.moveArm(saved_pose) 
    #move_base.goto(gc.robot_coords.pose.position.x, gc.robot_coords.pose.position.y, 0) 
    gc.scene.waitForSync() 
    gripper.open()

    gc.tuck()
    torso_action.move_to([0.0, ])
