#!/usr/bin/env python
import rospy
import sys
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
import tf2_msgs.msg
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import *
from moveit_msgs.msg import Grasp
from tf import TransformListener
from std_msgs.msg import String, Int32, Int32MultiArray, Float32MultiArray

class Planner():
  def __init__(self):
    #Inicializamos la interfaz de moveit
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_py", anonymous=True)
    
    #Creamos el moveit commander del xarm y la escena
    xarm = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    #Creamos el move group del xarm
    group_name = "xarm6"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    #Creamos un publisher para el topico para desplegar la trayectoria
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    
    #Inicializamos algunas variables del Planner
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = xarm.get_group_names()
    self.box_name = ""
    self.robot = xarm
    self.scene = scene
    self.move_group = move_group
    #self.gripper_move_group = gripper_move_group 
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
  
  #Funcion para ir a la posicion deseada
  def goToPose(self,goal):
    #Selecionamos la parte del robot que vamos a mover (En esta caso todo el xarm)
    move_group = self.move_group
    #Creamos un arreglo de los puntos deseados
    waypoints = [1]
    #Obtenemos las posiciones del xarm y las cambiamos a las deseadas
    wpose = move_group.get_current_pose().pose
    wpose.position.z = goal[2]
    wpose.position.x = goal[0]
    waypoints[0]=copy.deepcopy(wpose)
    #Creamos el plan de trayectoria y lo ejecutamos
    (plan, fraction) = move_group.compute_cartesian_path(
      waypoints,   # waypoints a seguir
      0.01,        # eef_step
      0.0)         # jump_threshold
    move_group.execute(plan, wait=True)



  #Funcion para imprimir la posicion del xarm
  def printPose(self):
    move_group = self.move_group
    wpose = move_group.get_current_pose().pose
    print(wpose.position.x)
    print("x \n")
    print(wpose.position.y)
    print("y \n")
    print(wpose.position.z)
    print("z \n")



  #Funcion para recibir la posicion deseada y mandarla al goToPose
  def callback(self, data):
    print("Coordenadas recibidas")
    pose = [0, 0, 0]
    #if(data[0] > 10):
    pose[0] = data.data[0]
    #if(data[1] > 10):
    pose[2] = data.data[2]
    self.goToPose(pose)

  


class myNode():
  def __init__(self):
    print("Nodo iniciado")
    
  def main(self):
    #Creamos nuestro Planner y el topico de suscripcion
    self.planner = Planner()
    rospy.Subscriber("PosXarm", Float32MultiArray, self.planner.callback)
    while not rospy.is_shutdown():
      self.planner.printPose()
    #Apagamos el programa
    rospy.signal_shutdown("Task Completed")

if __name__ == '__main__':
  try:
    #Iniciamos nuestro Nodo y corremos el programa
    node = myNode()
    node.main()
  except rospy.ROSInterruptException:
    pass


