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
from sensor_msgs.msg import Range
from autonomous.msg import stopsign


NODE_NAME = "esc_control_node"
SUB_TOPIC1 = "/stop_info"
SUB_TOPIC2 = "/ultrasoundf1"
PUB_TOPIC = "control_esc"
QUEUE_SIZE = 1
esc = []
rangelist = []
#controller = carcontrol()
go = 0
st = 0



class steeringcontrolnode:

    def __init__(self, node_name, sub_topic1, sub_topic2, pub_topic):
        
        rospy.init_node(node_name, anonymous=True)
        rate = rospy.Rate(25) #publish Rate wird auf 25 Hz gesetzt, da Kamera maximal 25 Bilder/s liefert

        self.stopee1 = True
        self.stopee2 = True
        self.stopee3 = True
        self.esc_pub = rospy.Publisher(pub_topic, UInt16, queue_size=QUEUE_SIZE)
        
        rospy.Subscriber(sub_topic1, stopsign, self.callbackstop, [go, st]) #subscribe to Stoppschild, Übergabe 2 Zähler
        rospy.Subscriber(sub_topic2, Range, self.callbackdist)  #subscribe to Ultraschall
        
        while not rospy.is_shutdown():
            
            #wenn kein Stoppschild erkannt wird und Ultraschallabstand >50cm dann fahren (ESC= 1555 langsames Fahren)
            if not self.stopee1 and not self.stopee2:
                esc = 1555
            else:
                if self.stopee3:
                    esc = 1500
                else:
                    esc = 1555
            
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
        
        print gost

    
    def callbackdist(self, data):
        
        #Ultraschallsignal wird gefiltert, um Fehlerkennungen auszuschließen; delta > 100 und Wert <10 dann fehlerhafter Wert; verhindert ruckartiges Fahren
        if len(rangelist)<10:
    
            rangelist.append(data.range)
        else:

            if abs(data.range-rangelist[8])<10 and abs(data.range-rangelist[9])>100:
                rangelist[9]=data.range
            else:
                rangelist.append(data.range)
                rangelist.pop(0)
            
            if rangelist[8] < 50:
                self.stopee2 = True
            else:
                self.stopee2 = False

        
    # wird zur Zeit nicht verwendet    
    def drive(self, data):
        
        self.esc = 1540
        self.esce_ = True
                
        
    def stop(self, data):
        
        self.esc = 1500
        self.esce_ = True


def main():
    try:
        steeringcontrolnode(NODE_NAME, SUB_TOPIC1, SUB_TOPIC2, PUB_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()

