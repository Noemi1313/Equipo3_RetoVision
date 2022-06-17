#!/usr/bin/env python
#Codigo usado actualmente como demo para pruebas de mover el xarm a una posicion deseada usando vision, Equipo 3
# Convertir de imagen de ROS a imagen de OpenCv y viceversa


import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    #Creamos los topicos para publicar el video de la camara y la posicion del xarm asi como el suscriptor de la camara del Puzzlebot
    self.image_pub = rospy.Publisher("Video",Image, queue_size = 10)
    self.msgs_xarm = rospy.Publisher("PosXarm", Float32MultiArray, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("video_source/raw", Image, self.callback)


  #Funcion cuando se recibe imagen del puzzlebot
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #Convertimos la imagen de BGR a HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #Creamos la mascara necesaria para detectar color amarillo
    lower_yellow = np.array([20,100,120]) #Azul (20, 90, 90)
    upper_yellow = np.array([40,255,255]) #Azul (255, 255, 255)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    #Usamos la mascara para detectar amarillo en la imagen 
    res_yellow=cv2.bitwise_and(cv_image, cv_image, mask=yellow_mask)

    #Invertimos blanco y negro
    invertY = cv2.bitwise_not(yellow_mask)
    ret, thY = cv2.threshold(invertY, 120, 255, cv2.THRESH_TOZERO)
    

    ##########################################################################
    #Eliminamos manchas en la imagen
    y_contours, y_hierarchy = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    threshold_blob_area = 5000

    #Reducimos el ruido del amarillo para filtrarlo
    for i in range(1, len(y_contours)):
      index_level = int(y_hierarchy[0][i][1])
      if index_level <= i:
        cnt = y_contours[i]
        area = cv2.contourArea(cnt)
        #print(area)
      if(area) <= threshold_blob_area:
        cv2.drawContours(yellow_mask, [cnt], -1, 0, -1, 1)

    #Encontramos las manchas
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    opening_yellow = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel, iterations=4)

    #Obtenemos las coordenadas del circulo amarillo
    xY,yY,wY,hY = cv2.boundingRect(opening_yellow)
    cv2.rectangle(opening_yellow, (xY, yY), (xY + wY, yY + hY), (139,0,0), 4)

  
    #Revisamos si se encontro un circulo amarillo
    if(wY > 5) and (wY<500):
      print("Encontre un circulo amarillo")
      print(str(xY) + "\n")
      print(str(yY) + "\n")
      list = Float32MultiArray()
      #Aproximamos su posicion en el plano cartesiano y definimo si esta arriba,abajo,izquierda o derecha
      #Para luego publicar la nueva posicion del xarm dependiendo de la posicion del circulo
      if(xY < 200 and yY < 150):
        list.data = [0.6, 0, 0.5]
        self.msgs_xarm.publish(list)
        print("Arriba derecha")
      elif(xY < 200 and yY > 400):
        list.data = [0.6, 0, 0.112]
        self.msgs_xarm.publish(list)
        print("Abajo derecha")
      elif(xY > 800 and yY < 150):
        list.data = [0.2, 0, 0.5]
        self.msgs_xarm.publish(list)
        print("Arriba izquierda")
      elif(xY > 800 and yY > 400):
        list.data = [0.2, 0, 0.112]
        self.msgs_xarm.publish(list)
        print("Abajo izquierda")
    else:
      print("No hay circulos") 


    #Publicamos la imagen del puzzlebot
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
