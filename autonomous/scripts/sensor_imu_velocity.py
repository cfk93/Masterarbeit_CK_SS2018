#!/usr/bin/env python
# -*- coding: utf-8 -*-

# imu_velocity_node

# Info: Rosnode zum Verarbeiten der IMU Beschleunigung in Geschwindigkeit
# Input: Imu Data
# Output: servo

# Autor: Fabian Fitzer (fabian.fitzer@evomotiv.de)
# Version: 1.0, 24.01.2018


import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math
import numpy as np
import time

NODE_NAME = "steering_control_node"
SUB_TOPIC = "/imu/data"
PUB_TOPIC = "velocity"
QUEUE_SIZE = 1
veloa = []
velo = []
dista = []
dist = []
timea = []




class imuvelocitynode:

    def __init__(self, node_name, sub_topic, pub_topic):
        
        rospy.init_node(node_name, anonymous=True)
        
        self.velo_pub = rospy.Publisher(pub_topic, Float64, queue_size=QUEUE_SIZE)
        
        rospy.init_node(node_name, anonymous=True)
        rate = rospy.Rate(25) #publish Rate wird auf 25 Hz gesetzt, da Kamera maximal 25 Bilder/s liefert
        
        self.imu_sub = rospy.Subscriber(sub_topic, Imu, self.callback)
        
        while not rospy.is_shutdown():
            rate.sleep()    # Schleife entsprechend der publish rate, um Wiederholungsfrequenz einzuhalten

        rospy.spin()

    def callback(self, data):
        
        timea.append(rospy.get_time())
        if len(timea)>2:
            timea.pop(0)
            #print (timea[1])    
        else:
            timea.append(rospy.get_time())
            
       
        
        veloa.append(round(data.linear_acceleration.x * (timea[1]-timea[0]),4))
        #velo = sum(veloa)
        #print ((round(data.linear_acceleration.x * 0.01,3)))
        if len(veloa)>1:
            velo = veloa[0]+veloa[1]
            veloa[0] = velo
            del veloa[1]
        else:
            veloa.append(round(data.linear_acceleration.x * (timea[1]-timea[0]),4))
            velo = veloa[0]+veloa[1]
            veloa[0] = velo
            del veloa[1]
            
        
        
        dista.append((velo*(timea[1]-timea[0])))
        if len(dista)>1:
            dist = dista[0]+dista[1]
            dista[0] = dist
            del dista[1]
        else:
            dista.append((velo*0.01))
            dist = dista[0]+dista[1]
            dista[0] = dist
            del dista[1]
            
        self.velo_pub.publish(dist)
        
        #rint (velo)
        #rint dist
        #return self.veloa

        
        


def main():
    try:
        imuvelocitynode(NODE_NAME, SUB_TOPIC, PUB_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
