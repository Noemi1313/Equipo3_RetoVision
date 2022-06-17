#!/usr/bin/env python
# Codigo para detectar colores (amarillo, verde y rojo)
# Equipo 3
# Noemi Carolina Guerra Montiel A00826944
# Maria Fernanda Hernandez Montes A01704918
# Mizael Beltran Romero A01114973
# Izac Saul Salazar Flores A01197392
# Junio 2022

# Librerias
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Clase para convertir la imagen a opencv y hacer toda la deteccion de colores
class image_converter:

  def __init__(self):
    # Publicar y subscribir a diferentes topicos
    self.image_pub = rospy.Publisher("Video",Image, queue_size = 10)
    self.msgs_motores = rospy.Publisher("Mensaje_semaforo", String, queue_size = 10)
    self.pos = rospy.Publisher("Posicion_semaforo", Int32, queue_size = 10)
    self.width = rospy.Publisher("Width", Int32, queue_size=10)
    self.msgs_seguidor = rospy.Publisher("Seguidor", Int32MultiArray, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("video_source/raw", Image, self.callback)
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #Convierte la imagen a un espacio de color HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #Poner los limites superiores e inferiores de la mascara para el color rojo
    lower_red1 = np.array([0,100,20])
    upper_red1 = np.array([8,255,255])
    lower_red2 = np.array([175,100,20])
    upper_red2 = np.array([179,255,255])

    #Poner los limites superiores e inferiores de la mascara para el color amarillo
    lower_yellow = np.array([20,100,120]) #Azul (20, 90, 90)
    upper_yellow = np.array([40,255,255]) #Azul (255, 255, 255)

    #Crear una mascara para el rojo y amarillo
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.add(red_mask1,red_mask2)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    #Bitwise y imagen original con mascaras
    res_red = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)
    res_yellow=cv2.bitwise_and(cv_image, cv_image, mask=yellow_mask)

    #Invertir negro y blanco
    invertR = cv2.bitwise_not(red_mask)
    ret, thR = cv2.threshold(invertR, 120, 255, cv2.THRESH_TOZERO)
    invertY = cv2.bitwise_not(yellow_mask)
    ret, thY = cv2.threshold(invertY, 120, 255, cv2.THRESH_TOZERO)
    
    ##########################################################################
    #Eliminar pequenos blobs
    r_contours, r_hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    y_contours, y_hierarchy = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    # Area max aceptada de blobs
    threshold_blob_area = 5000

    # Ruido de los blobs Rojos
    for i in range(1, len(r_contours)):
      index_level = int(r_hierarchy[0][i][1])
      if index_level <= i:
        cnt = r_contours[i]
        area = cv2.contourArea(cnt)
        #print(area)
      if(area) <= threshold_blob_area:
        cv2.drawContours(red_mask, [cnt], -1, 0, -1, 1)

    #Ruido de los blobs Amarillos
    for i in range(1, len(y_contours)):
      index_level = int(y_hierarchy[0][i][1])
      if index_level <= i:
        cnt = y_contours[i]
        area = cv2.contourArea(cnt)
        #print(area)
      if(area) <= threshold_blob_area:
        cv2.drawContours(yellow_mask, [cnt], -1, 0, -1, 1)

    #Encontrar contorno del blob
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    opening_red = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=4)
    opening_yellow = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel, iterations=4)

    #Obtener coordenadas
    #Rectangulo Rojo
    xR,yR,wR,hR = cv2.boundingRect(opening_red)
    cv2.rectangle(opening_red, (xR, yR), (xR + wR, yR + hR), (139,0,0), 4)
    #Rectangulo Amarillo
    xY,yY,wY,hY = cv2.boundingRect(opening_yellow)
    cv2.rectangle(opening_yellow, (xY, yY), (xY + wY, yY + hY), (139,0,0), 4)
    
    #Notificar cuando se encuentra un color
    if(wR > 5) and (wR<500):
      print("Encontre un circulo rojo")
      self.msgs_motores.publish("Rojo")
      list = Int32MultiArray()
      list.data = [xR, wR, 0] # Para el reto 1
      self.msgs_seguidor.publish(list) # Para el reto 1
    elif(wY > 5) and (wY<500):
      print("Encontre un circulo amarillo")
      self.msgs_motores.publish("Amarillo")
      list = Int32MultiArray()
      #list.data = [xY, wY, 0] # Para el reto 2
      #self.msgs_seguidor.publish(list) # Para el reto 2
    else:
      self.msgs_motores.publish("Rojo")
      print("No hay circulos")
      
    #Display de las imagenes
    cv2.imshow("Original", cv_image)
    #cv2.imshow("Res red", res_red)
    #cv2.imshow("Red Mask", red_mask)
    cv2.imshow("Opening Red", opening_red)
    cv2.imshow("Opening Yellow", opening_yellow)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  # Inicializar nodo
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  # Exit
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
