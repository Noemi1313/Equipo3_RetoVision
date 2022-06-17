#!/usr/bin/env python
# Reto 1: Vision
# Programa para seguir un objeto de color rojo con el puzzlebot
# Equipo 3
# Noemi Carolina Guerra Montiel A00826944
# Maria Fernanda Hernandez Montes A01704918
# Mizael Beltran Romero A01114973
# Izac Saul Salazar Flores A01197392
# Junio 2022


#Librerias que contienen funciones para el programa
from cmath import pi
from math import atan2, cos, sin, sqrt
import rospy, time
#Librerias de mensajes de ROS
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, String, Int32, Int32MultiArray

class Seguidor():

    #Iniciamos el seguidor
    def __init__(self):
        self.vw = Twist()
        self.vw.linear.x=0
        self.vw.linear.y=0
        self.vw.linear.z=0
        self.vw.angular.x=0
        self.vw.angular.y=0
        self.vw.angular.z=0

    #Funcion para publicar la velocidad
    def pubVel(self):
        #Publicamos la velocidad
        pubvel.publish(self.vw)

     #Funcion de callback para el topic del semaforo
    def semaforo(self, msg):
        if(msg.data == "Rojo"):
            self.vw.linear.x=0
            self.vw.angular.z=0
            self.pubVel()

    #Funcion para hacer el seguimiento
    def seguir(self,msg):
        #Revisamos primero si el objetivo esta centrado(Datos obtenidos experimentalmente)
        print("Pos x: " + str(msg.data[0]) + "\n")
        print("Distancia: " + str(msg.data[1]) + "\n")

        #Giro a la derecha
        if(msg.data[0] > 1000):
            self.vw.angular.z=-0.1
            self.vw.linear.x=0
            print("Girando derecha")
        #Giro a la izquierda
        if(msg.data[0] < 200):
            self.vw.angular.z=0.1
            self.vw.linear.x=0
            print("Girando izquierda")
        #Centro
        if(msg.data[0] >=200 and msg.data[0] <= 1000):
            self.vw.angular.z=0
            #Una vez que el objetivo esta centrado vemos su distancia(Datos obtenidos experimentalmente)
            # Si esta lejos
            if(msg.data[1] < 300): #(300)
                self.vw.linear.x=0.1
                print("Avanzando")
            # Si esta cerca
            if(msg.data[1] > 350): #(350)
                # Detenerse completamente
                self.vw.linear.x=0
        self.pubVel()


if __name__ == '__main__':
    try:
        #Iniciamos el puzzlebot junto con todos los topicos que necesitamos
        seguidor = Seguidor()
        pubvel = rospy.Publisher("cmd_vel", Twist, queue_size=10) #Topic de cmd_vel para publicar las velocidades
        rospy.init_node("seguidor")
        rospy.Subscriber("Mensaje_semaforo", String, seguidor.semaforo)#Topic del semaforo para recibir el color 
        rospy.Subscriber("Seguidor", Int32MultiArray, seguidor.seguir)
        #while not rospy.is_shutdown():
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
