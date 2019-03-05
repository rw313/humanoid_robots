from graspit_commander import GraspitCommander
import geometry_msgs

from graspit_commander.graspit_exceptions import (
    LoadWorldException,
    ImportObstacleException,
    ImportRobotException,
    ImportGraspableBodyException,
)

pose = geometry_msgs.msg.Pose()
pose.orientation.y = 0.7071
pose.orientation.w = 0.7071

pose2 = geometry_msgs.msg.Pose()
pose2.position.x = .5
pose2.position.y = .5
pose2.position.z = .5

pose3 = geometry_msgs.msg.Pose()
pose3.position.x = -2
pose3.position.y = -2
pose3.position.z = -0.09

GraspitCommander.clearWorld()
GraspitCommander.importGraspableBody("longBox", pose)
GraspitCommander.importRobot("fetch_gripper", pose2)
GraspitCommander.importObstacle("floor", pose3) 

result = GraspitCommander.planGrasps(max_steps=50000)

print(result.grasps[0])



