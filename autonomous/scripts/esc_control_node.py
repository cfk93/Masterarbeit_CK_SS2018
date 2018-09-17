#!/usr/bin/env python
# -*- coding: utf-8 -*-

# esc_control_node

# Info: Rosnode zum Verarbeiten der ESC-Steuersignale
# Input: Stopschilderkennung und Ultraschallabstand
# Output: esc

# Autor: Fabian Fitzer (fabian.fitzer@evomotiv.de)
# Version: 1.0, 29.01.2018


import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from autonomous.msg import stopsign
from scipy import signal
import numpy as np


NODE_NAME = "esc_control_node"
SUB_TOPIC1 = "/stop_info"
SUB_TOPIC2 = "/ultrasoundf1"
PUB_TOPIC1 = "control_esc"
PUB_TOPIC2 = "us_abstand"
QUEUE_SIZE = 1
esc = []
rangelist = []
#controller = carcontrol()
VELOCITY_FORWARD = 1610     # PWM-Wert fuer fahren
VELOCITY_ZERO = 1500        # PWM-Wert fuer halten
BURST = 10
go = 0
st = 0



class steeringcontrolnode:

    def __init__(self, node_name, sub_topic1, sub_topic2, pub_topic1, pub_topic2):
        
        rospy.init_node(node_name, anonymous=True)
        rate = rospy.Rate(30) #publish Rate wird auf 30 Hz gesetzt, da Kamera maximal 30 Bilder/s liefert

        self.stopee1 = True
        self.stopee2 = True
        self.stopee3 = True
        self.esc_pub = rospy.Publisher(pub_topic1, UInt16, queue_size=QUEUE_SIZE)
        self.us_dist = rospy.Publisher(pub_topic2, Float32, queue_size=QUEUE_SIZE)
        
        rospy.Subscriber(sub_topic1, stopsign, self.callbackstop, [go, st]) #subscribe to Stoppschild, Übergabe 2 Zähler
        rospy.Subscriber(sub_topic2, Range, self.callbackdist)  #subscribe to Ultraschall
        
        while not rospy.is_shutdown():
            
            #wenn kein Stoppschild erkannt wird und Ultraschallabstand >50cm dann fahren (ESC= 1600 langsames Fahren)
            if not self.stopee1 and not self.stopee2:
                esc = VELOCITY_FORWARD
            else:
                if self.stopee3:
                    esc = VELOCITY_ZERO
                else:
                    esc = VELOCITY_FORWARD
            
            self.esc_pub.publish(esc)
            
            rate.sleep()    # Schleife entsprechend der publish rate, um Wiederholungsfrequenz einzuhalten
        rospy.spin()

    
    def callbackstop(self, data, gost):
        
        #Algorithmus um 6 Sekunden am Stoppschild zu halten (durch Zähler), dann 10s ignorieren, um wieder anzufahren 
        if data.erkennung == 1:
            if gost[0]<150:
                self.stopee1 = True
                gost[0] = gost[0]+1
            else:
                if gost[1] < 250:
                    self.stopee1 = False
                    gost[1] = gost[1]+1
                else:
                    gost[0] = 0
                    gost[1] = 0
        else:
            self.stopee1 = False
            #self.stopee3 = True
            gost[0] = 0
            gost[1] = 0
        
        #print gost

    
    
    
    
    def callbackdist(self, data):
        
        #Ultraschallsignal wird gefiltert, um Fehlerkennungen auszuschließen; delta > 100 und Wert <10 dann fehlerhafter Wert; verhindert ruckartiges Fahren
        
        dist_raw = round(data.range,3)
        
        
        b,a = signal.butter(3, 0.5)
        
        if len(rangelist)<20:
            rangelist.append(dist_raw)
        else:
            rangelist.pop(0)                
            rangelist.append(dist_raw)
            
            filtlist = signal.filtfilt(b,a,rangelist)
            
            
            if abs(dist_raw-filtlist[len(filtlist)-2])<10 and abs(dist_raw-rangelist[len(filtlist)-1])>100:
                filtlist[len(filtlist)-1]=dist_raw
           # else:
                #rangelist.append(data.range)
                #rangelist.pop(0)
            
            if filtlist[len(filtlist)-1] < 30:
                self.stopee2 = True
            else:
                self.stopee2 = False
                
            #print "Range:", data.range
            #print "Liste:", filtlist[len(filtlist)-1]
            #rospy.loginfo("US-Abstand: %s", rangelist)
            mittelw = round(np.mean(filtlist),2)
            self.us_dist.publish(mittelw)

        
    # wird zur Zeit nicht verwendet    
    def drive(self, data):
        
        self.esc = VELOCITY_FORWARD
        self.esce_ = True
                
        
    def stop(self, data):
        
        self.esc = VELOCITY_ZERO
        self.esce_ = True


def main():
    try:
        steeringcontrolnode(NODE_NAME, SUB_TOPIC1, SUB_TOPIC2, PUB_TOPIC1, PUB_TOPIC2)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()

