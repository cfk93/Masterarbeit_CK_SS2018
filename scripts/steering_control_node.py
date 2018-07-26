#!/usr/bin/env python
# -*- coding: utf-8 -*-

# steering_control_node

# Info: Rosnode zum Verarbeiten der Steuersignale
# Input: Regler Output Lenken
# Output: servo

# Autor: Fabian Fitzer (fabian.fitzer@evomotiv.de)
# Version: 1.0, 24.01.2018


import rospy
from std_msgs.msg import Float64
from autonomous.msg import carcontrol
import math
import numpy as np
import time

NODE_NAME = "steering_control_node"
SUB_TOPIC = "/control_steering"
PUB_TOPIC = "car_control"
QUEUE_SIZE = 1
controller = carcontrol()
winkel = []



class steeringcontrolnode:

    def __init__(self, node_name, sub_topic, pub_topic):

        self.control_pub = rospy.Publisher(pub_topic, carcontrol, queue_size=QUEUE_SIZE)
        
        rospy.init_node(node_name, anonymous=True)
        rate = rospy.Rate(30) #publish Rate wird auf 30 Hz gesetzt, da Kamera maximal 30 Bilder/s liefert
        
        self.stop_sub = rospy.Subscriber(sub_topic, Float64, self.callback, winkel) #subscribe to Regler Output
        
        while not rospy.is_shutdown():
            rate.sleep()    # Schleife entsprechend der publish rate, um Wiederholungsfrequenz einzuhalten

        rospy.spin()

    def callback(self, data, winkel):
                
        #hier mögliche Filterung, aktuell ungenutzt und nur Umrechnung auf richtigen Lenkwinkel        
        if len winkel<10:
            winkel.append(data.data)
            controller.servo = int(data.data)+90
        else:
            if abs(data.data-winkel[9])<5:
                winkel.append(data.data)
                winkel.pop(0)
                
            controller.servo = int(winkel[9])+90
                

        self.control_pub.publish(controller)


def main():
    try:
        steeringcontrolnode(NODE_NAME, SUB_TOPIC, PUB_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
