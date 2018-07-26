#!/usr/bin/env python
# -*- coding: utf-8 -*-

# car_control_node

# Info: Rosnode zum Testen der Datenverarbeitung
# Input: Spur, Lenkung, St
# Output: servo und esc

# Autor: Fabian Fitzer (fabian.fitzer@evomotiv.de)
# Version: 1.1, 29.01.2018


import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Float64
from autonomous.msg import carcontrol
from autonomous.msg import lane
import math
import numpy as np
import time

NODE_NAME = "car_control_node"
SUB_TOPIC1 = "/control_steering"
SUB_TOPIC2 = "/control_esc"
SUB_TOPIC3 = "/lane_info"
PUB_TOPIC1 = "car_control"
QUEUE_SIZE = 1
controller = carcontrol()
steer = []
esce = []
straightee = []
cha = 0
stra = 0
winkel = []
VELOCITY_FORWARD = 1600
VELOCITY_ZERO = 1500



class carcontrolnode:

    def __init__(self, node_name, sub_topic1, pub_topic1, sub_topic2, sub_topic3):
        
        self.steer_ = False
        self.esce_ = False
        self.erkennung_ = True
        self.control_pub = rospy.Publisher(pub_topic1, carcontrol, queue_size=QUEUE_SIZE)
        
        rospy.init_node(node_name, anonymous=True)
        rate = rospy.Rate(30) #publish Rate wird auf 30 Hz gesetzt, da Kamera maximal 30 Bilder/s liefert
        
        rospy.Subscriber(sub_topic1, Float64, self.callback1)   #subscribe to Lenkungsregler (Querregelung)
        rospy.Subscriber(sub_topic2, UInt16, self.callback2)    #subscribe to Motorsteuerung (Längsregelung)
        rospy.Subscriber(sub_topic3, lane, self.callback3, [straightee, cha, stra]) #subscribe to Spurerkennung
        
        while not rospy.is_shutdown():
            
            if self.steer_:
                if self.straight_:
                    controller.servo = (((self.steer.data - 90)/2)+90) #Umrechnung Reglerwert in Lenkwinkel, für Geraden nur halber Wert

                else:
                    controller.servo = self.steer.data

            
            else:
                controller.servo = 90
            #print (controller)
            if self.esce_ and self.erkennung_:
                controller.esc = self.esce.data
            else:
                controller.esc = VELOCITY_ZERO
            
            self.control_pub.publish(controller) #aussenden car_controller topic, wird vom Arduino empfangen
            rate.sleep()    # Schleife entsprechend der publish rate, um Wiederholungsfrequenz einzuhalten
        
        rospy.spin()




    
# callback zur Querregelung, input: Reglerwert von Ausgang control_steering PID-Regler
    def callback1(self, data):
        
        self.steer = UInt16(90 + (data.data))
        self.steer_ = True
        
        #if len(winkel)<10:
         #   winkel.append(data.data)
          #  self.steer = UInt16((data.data)+90)
        #else:
          #  if abs(data.data-winkel[9])<10:
          #      winkel.append(data.data)
           #     winkel.pop(0)
                
           # self.steer = UInt16((winkel[9])+90)
        
        
        
        
        
# callback zur Längsregelung, Input: esc-Daten                  
    def callback2(self, data):
        
        if data.data > 0:
            self.esce_ = True
            self.esce = data
            #print (data)
        else:
            self.esce_ = False
            
            
            
            
            
# callback zur Spurerkennung, Input: lane-Daten
    def callback3(self, data,  strchastra):
        #Algorithmus um Lenkwinkel nach Kurven offen zu halten
        if len(straightee)<25:
            strchastra[0].append(data.radius)
        else:
            strchastra[0].append(data.radius)
            strchastra[0].pop(0)
        
        if (sum(strchastra[0])/len(strchastra[0])) < 5:
            self.straight_ = False
            strchastra[1] = 1
        else:
            if strchastra[1] == 1 and strchastra[2] < 125:
                self.straight_ = False
                strchastra[2] = strchastra[2]+1
            else:
                self.straight_ = True
                strchastra[1] = 0
                strchastra[2] = 0
            
            
        if int(data.erkennung) == 0:
            self.erkennung_ = False
        else:
            self.erkennung_ = True
        



def main():
    try:
        carcontrolnode(NODE_NAME, SUB_TOPIC1, PUB_TOPIC1, SUB_TOPIC2, SUB_TOPIC3)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()


