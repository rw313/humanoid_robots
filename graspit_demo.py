from graspit_commander import GraspitCommander
import geometry_msgs

from graspit_commander.graspit_exceptions import (
    LoadWorldException,
    ImportObstacleException,
    ImportRobotException,
    ImportGraspableBodyException,
)

pose = geometry_msgs.msg.Pose()
pose.position.x = 2
pose.position.y = 2
pose.position.z = .1
pose.orientation.x = 0
pose.orientation.y = 0.7071
pose.orientation.z = 0
pose.orientation.w = 0.7071

pose2 = geometry_msgs.msg.Pose()
pose.position.x = 2
pose.position.y = 2
pose.position.z = .4

GraspitCommander.clearWorld()
#GraspitCommander.loadWorld("plannerMug")
GraspitCommander.importRobot("fetch_gripper", pose2)
GraspitCommander.importObstacle("floor")
GraspitCommander.importGraspableBody("longBox", pose)

#result = GraspitCommander.dynamicAutoGraspComplete(0)
result = GraspitCommander.planGrasps(max_steps=50000)
#PlanGraspsResult is the type for result
print(result.grasps[0])


