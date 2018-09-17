#!/usr/bin/env python
# -*- coding: utf-8 -*-

# image_stopsign_node

# Info: Rosnode zur Stopschilderkennung
# Input: Image_Raw
# Output: Erkennung Ja/Nein

# Autor: Fabian Fitzer (fabian.fitzer@evomotiv.de)
# Version: 1.0, 15.01.2018


import rospy
from sensor_msgs.msg import Image
from autonomous.msg import stopsign
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np
import cv2

NODE_NAME = "image_stopsign_node"
SUB_TOPIC = "/autonomous/image_raw"
PUB_TOPIC = "stop_info"
QUEUE_SIZE = 1
stop_detect= cv2.CascadeClassifier('/home/nvidia/opencv/data/haarcascades/stopsign_classifier.xml')
stopper = stopsign()

SETVALUE_STOPSIGN_WIDTH = 135 # Größe des Stoppschildes in Pixel, um Erkennung zu aktivieren


class imagestopsignnode:

    def __init__(self, node_name, sub_topic, pub_topic):
        self.bridge = CvBridge()

        rospy.init_node(node_name, anonymous=True)
        
        rate = rospy.Rate(30) #publish Rate wird auf 30 Hz gesetzt, da Kamera maximal 30 Bilder/s liefert
        
        self.stop_pub = rospy.Publisher(pub_topic, stopsign, queue_size=QUEUE_SIZE)
        self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback) #subscribe to Rohbild
        
        while not rospy.is_shutdown():
            rate.sleep()    # Schleife entsprechend der publish rate, um Wiederholungsfrequenz einzuhalten
        
        rospy.spin()
        
        
        

    def callback(self, data):
        
        try:
            image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)              #Graustufenbild ganzes Bild
        image_gray = cv2.cvtColor(image_gray, cv2.COLOR_GRAY2RGB)
        img = image[0:image.shape[0],(image.shape[1]/2):image.shape[1]]   #nur rechte Bildseite durchlassen
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)                      #Graustufenbild rechte Seite
	stops = stop_detect.detectMultiScale(gray, 1.3, 5)                #Anwendung CascadeClassifier # original scalingFactor: 1.4
	
	
	#unrelevanter Bereich grau einfaerben
	overlay = image_gray.copy()
	cv2.rectangle(overlay, (0,0), (320,480), (150,150,150), -1)
	alpha = 0.8
        cv2.addWeighted(overlay, alpha, image_gray, 1-alpha, 0, image_gray)
	
	
	
	
	#Visualisierung erkanntes Stoppschild
	for (x,y,w,h) in stops:
            cv2.putText(image_gray,'Stop!',(x+320,y-10),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2,cv2.LINE_AA)
            cv2.rectangle(image_gray,(x+320,y),(x+w+320,y+h),(5,116,236),2)
            
            stop_x_px = x+320
            stop_y_px = y
            stop_breite_px = w  # theoretisch maximal moegliche Breite = 320px, praktisch aber maximal <= 255px --> kann in uint8 gespeichert werden, Ueberschreitung muss aber abgefangen werden
            stop_hoehe_px  = h
        
            cv2.putText(image_gray, "Breite: %.2fpx" % (h), (350, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            
        cv2.imshow('Stoppschild Image',image_gray)
        cv2.waitKey(1) 
        
        
        #Erkennung auf 1 setzen wenn erkannt (keine Erkennung: Format = tuple)
        if isinstance(stops, tuple):
            stopper.erkennung = 0
            stopper.breite_px = 0
            stopper.hoehe_px  = 0
            stopper.posx_px   = 0
            stopper.posy_px   = 0
        else:
            #wenn Breite und Hoehe Stoppschild > 60px dann Erkennung, muss <= 255px sein, sonst Speicherung nicht in uint8 moeglich
            #if stops[0,2]>60 and stops[0,2]<=255 and stops[0,3]>60 and stops[0,3]<=255:
            if stops[0,2]<=255 and stops[0,3]<=255:
                stopper.breite_px = stop_breite_px
                stopper.hoehe_px  = stop_hoehe_px
                stopper.posx_px   = stop_x_px
                stopper.posy_px   = stop_y_px
                
                if stops[0,3] >= SETVALUE_STOPSIGN_WIDTH:       # Groesse des Stoppschild ~ Abstand
                    stopper.erkennung = 1
                
            else:
                stopper.erkennung = 0


        try:
            self.stop_pub.publish(stopper)
        except CvBridgeError as e:
            rospy.logerr(e)



def main():
    try:
        imagestopsignnode(NODE_NAME, SUB_TOPIC, PUB_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()

