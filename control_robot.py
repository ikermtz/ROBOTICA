import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from typing import List
from geometry_msgs.msg import Pose, PoseStamped

class ControlRobot:
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("control robot", anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.add_floor()

    def get_motor_angles(self) -> list:
        return self.move_group.get_current_joint_values()
    
    def move_motors(self, joint_goal: List[float], wait: bool=True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)

    def get_pose(self) -> Pose:
        return self.move_group.get_current_pose().pose

    def move_to_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)

    def add_box_to_planning_scene(self, pose_caja: Pose, name: str, tamaño: tuple = (.1, .1, .1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose = pose_caja
        box_name = name
        self.scene.add_box(box_name, box_pose, size=tamaño)

    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.025
        self.add_box_to_planning_scene(pose_suelo, "suelo", (2,2,.05))

    def move_to_path(path) -> None:
        points = []     

        wpose = control.get_pose()
        wpose.position.z -= wpose.position.z * 0.1
        wpose.position.y -= wpose.position.y * 0.1
        wpose.position.x -= wpose.position.x * 0.1
        points.append(copy.deepcopy(wpose))

        wpose2 = control.get_pose()
        wpose2.position.z -= wpose2.position.z * 0.2
        wpose2.position.y -= wpose2.position.y * 0.2
        wpose2.position.x -= wpose2.position.x * 0.2
        points.append(copy.deepcopy(wpose2))

if __name__ == '__main__':
    control = ControlRobot()
    pose_act = control.get_pose()
    pose_act.position.z -= 0.1
    control.move_to_pose(pose_act)
    #configuracion_act = control.get_motor_angles()
    #configuracion_act[0] += pi
    #control.move_motors(configuracion_act)