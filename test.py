# Imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos, sin
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize
from typing import List
from geometry_msgs.msg import Pose, PoseStamped

# Clase para controlar las funciones del robot
class ControlRobot:

    # Constructor de la clase para controlar las funciones del robot
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.add_floor()

    # Devolvemos una lista de los angulos de los motores
    def get_motor_angles(self) -> list:
        return self.move_group.get_current_joint_values()
    
    # Movemos los motores del robot a la posicion que le indicamos
    def move_motors(self, joint_goal: List[float], wait: bool=True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)

    # Obtenemos la pose del efector final del robot
    def get_pose(self) -> Pose:
        return self.move_group.get_current_pose().pose

    # Mover el efector final del robot a una pose indicada, devuelve True al finalizar
    def move_to_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)

    # Añadimos un obstaculo, en este caso una caja
    def add_box_to_planning_scene(self, pose_caja: Pose, name: str, tamaño: tuple = (.1, .1, .1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        box_name = name
        self.scene.add_box(box_name, box_pose, size=tamaño)

    # Añadimos el suelo
    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.add_box_to_planning_scene(pose_suelo, "suelo", (2,2,.05))

    # Indicamos una trayectoria al robot, devuelve True al finalizar
    def move_trajectory(self, poses: List[Pose], wait: bool = True) -> bool:
        poses.insert(0, self.get_pose())
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01, 0.0)

        if fraction != 1.0:
            return False
        
        return self.move_group.execute(plan, wait = wait)

    def moverConfig1(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [-0.5412643591510218, -2.1325680218138636, -0.4804496467113495, -2.1262718639769496, 1.5534838438034058, 0.39564502239227295]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)
    
    def moverConfig2(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [-0.23798686662782842, -2.1974054775633753, -0.5155571699142456, -1.978826185266012, 1.5638964176177979, 0.7119466066360474]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)

    def moverConfig3(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [0.2328263819217682, -2.180206914941305, -0.42928189039230347, -2.1217647991576136, 1.6015007495880127, 1.05226731300354]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)
    
    def moverConfigBase(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [0.28942328691482544, -1.535173923974373, -0.9331789016723633, -2.233608385125631, 1.5858209133148193, 1.2375750541687012]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)
    

if __name__ == '__main__':

    """
    print("")
    print("Empieza la ejecucion de la practica 1")
    print("")
    control = ControlRobot()
    pose_act = control.get_pose()

    # Task 1: Añadir una caja a la escena de planificación
    print("TASK 1: Añadir un obstaculo en la posicion (0.5, 0.5, 0.25)")
    caja_pose = geometry_msgs.msg.Pose()
    caja_pose.position.x = 0.5
    caja_pose.position.y = 0.5
    caja_pose.position.z = 0.25
    control.add_box_to_planning_scene(caja_pose, "Caja_practica1", (0.1, 0.1, 0.5))
    print(" Caja añadida a la escena")
    print("")

    # Task 2: Mover el robot a una pose específica
    print("TASK 2: Mover el robot a una pose especifica")
    pose_goal = geometry_msgs.msg.Pose() 
    print(" Pose actual del robot:")
    print(pose_act)   
    pose_goal.position.x = 0.3  
    pose_goal.position.y = 0.1  
    pose_goal.position.z = 0.4  
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 0.0
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 1.0

    hecho = control.move_to_pose(pose_goal)
    if hecho:
        print("El robot ha alcanzado la pose objetivo")
    else:
        print("No se pudo alcanzar la pose objetivo")

    print(" Pose objetivo del robot:")
    print(pose_goal)
    print("")

    # Task 3: Rotar las articulaciones del robot
    print("TASK 3: Mover las articulaciones del robot")
    joints = control.get_motor_angles()
    print(" Ángulos actuales de los motores:")
    print(joints)

    i = 0
    while (i<6):

        # Uso el bug de que el rango de movimiento de las articulaciones es (-6.28, 6.28)
        if (joints[i]>2.0):
            joints[i] -= 0.33

        else:
            joints[i] += 0.33
   
        i += 1

    hecho = control.move_motors(joints)
    if hecho:
        print(" El robot ha rotado todas sus articulaciones tau/4 (90 grados)")

    else:
        print(" No se pudo alcanzar la nueva posición de las articulaciones")

    print(" Nuevos ángulos de los motores:")
    print(control.get_motor_angles())
    print("")

    #Task 4: Mover el robot a una pose específica
    print("TASK 4: Mover el robot en una trayectoria")

    # Definimos una trayectoria de poses en un vector
    poses = []

    # Waypoint 1
    pose1 = geometry_msgs.msg.Pose()
    pose1.position.x = 0.4
    pose1.position.y = 0.0
    pose1.position.z = 0.2
    pose1.orientation.w = 1.0
    poses.append(pose1)

    # Waypoint 2
    pose2 = geometry_msgs.msg.Pose()
    pose2.position.x = 0.5
    pose2.position.y = 0.0
    pose2.position.z = 0.2
    pose2.orientation.w = 1.0
    poses.append(pose2)

    # Waypoint 3
    pose3 = geometry_msgs.msg.Pose()
    pose3.position.x = 0.6
    pose3.position.y = 0.0
    pose3.position.z = 0.2
    pose3.orientation.w = 1.0
    poses.append(pose3)

    print(" Poses de la trayectoria:")
    print("")

    i = 0
    while(i<3):
        print("     Pose numero: " + str(i+1))
        print("         X :" + str(poses[i].position.x))
        print("         Y :" + str(poses[i].position.y))
        print("         Z :" + str(poses[i].position.z))
        print("")
        i += 1

    # Intentamos mover el robot a lo largo de la trayectoria
    hecho = control.move_trajectory(poses)

    if hecho:
        print(" El robot ha completado la trayectoria con éxito")
    else:
        print(" No se pudo completar la trayectoria")

    print("")
    print(" Pose final del robot:")
    print(control.get_pose())
    print("")
    """

    control = ControlRobot()
    print("Angulos de base:")
    control.moverConfigBase()
    print(control.get_motor_angles())
    
    print("Angulos a los que deberi ir:")
    print("-0.5412643591510218, -2.1325680218138636, -0.4804496467113495, -2.1262718639769496, 1.5534838438034058, 0.39564502239227295")
    control.moverConfig1()
    print(" Nuevos ángulos de los motores:")
    print(control.get_motor_angles())

    print("Angulos a los que deberi ir:")
    print("0.28942328691482544, -1.535173923974373, -0.9331789016723633, -2.233608385125631, 1.5858209133148193, 1.2375750541687012")
    control.moverConfigBase()
    print(" Nuevos ángulos de los motores:")
    print(control.get_motor_angles())
    print("")

    print("Angulos a los que deberi ir:")
    print("-0.23798686662782842, -2.1974054775633753, -0.5155571699142456, -1.978826185266012, 1.5638964176177979, 0.7119466066360474")    
    control.moverConfig2()
    print(" Nuevos ángulos de los motores:")
    print(control.get_motor_angles())
    print("")

    print("Angulos a los que deberi ir:")
    print("0.28942328691482544, -1.535173923974373, -0.9331789016723633, -2.233608385125631, 1.5858209133148193, 1.2375750541687012")
    control.moverConfigBase()
    print(" Nuevos ángulos de los motores:")
    print(control.get_motor_angles())
    print("")

    print("Angulos a los que deberi ir:")
    print("0.2328263819217682, -2.180206914941305, -0.42928189039230347, -2.1217647991576136, 1.6015007495880127, 1.05226731300354")    
    control.moverConfig3()
    print(" Nuevos ángulos de los motores:")
    print(control.get_motor_angles())
    print("")

    print("Angulos a los que deberi ir:")
    print("0.28942328691482544, -1.535173923974373, -0.9331789016723633, -2.233608385125631, 1.5858209133148193, 1.2375750541687012")
    control.moverConfigBase()
    print(" Nuevos ángulos de los motores:")
    print(control.get_motor_angles())
    print("")
