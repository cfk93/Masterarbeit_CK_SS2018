#!/usr/bin/env python
# -*- coding: utf-8 -*-

# diagnostic_node

# Info:     Rosnode zum Testen des CAN-Datentransfers
# Input:    Sämtliche Signale, die auf dem ROS-Bus liegen
# Output:   - (nur externer Output der Dummy-Botschaft)

# Autor: Christof Kary (christof.kary@evomotiv.de)
# Version: 1.0, 04.06.2018

import rospy
import numpy as np
import time
import can
import struct
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt16
from std_msgs.msg import Float64
from autonomous.msg import lane
from autonomous.msg import stopsign
from autonomous.msg import carcontrol
from sensor_msgs.msg import Range


NODE_NAME  = "diagnostic_node"
SUB_TOPIC1 = "/lane_info"             # Publisher-Skript: image_lanedetection_node.py
SUB_TOPIC2 = "/stop_info"             # Publisher-Skript: image_stopsign_node.py
SUB_TOPIC3 = "/car_control"           # Publisher-Skript: car_control_node.py
SUB_TOPIC4 = "/ultrasoundf1"          # Publisher-Skript: ROS_1_3.ino
SUB_TOPIC5 = "/autonomous/image_raw"  # Publisher-Skript: image_lanedetection_node.py
SUB_TOPIC6 = "/control_esc"           # Publisher-Skript: esc_control_node.py
SUB_TOPIC7 = "/control_steering"      # Publisher-Skript: PID-Controller (siehe launch-File)

QUEUE_SIZE = 1

spur       = lane()
stopper    = stopsign()
controller = carcontrol()


#lane_right = np.int64(0)
#lane_left  = np.int64(0)
#lane_abw   = np.int64(0)
#lane_status= np.int64(0)
#lane_radius= np.float64(0)
#eng_rpm    = np.uint16(0)
#steer_angle= np.uint16(0)

straightee = []
cha = 0
stra = 0

bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)    # Initialisiert den CAN-Bus mit 500 kBit/s

# Message definieren und bei jedem Durchlauf Nutzdaten leeren, ansonsten werden Nutzdaten in andere Botschaften geschrieben   
reset_msg  = can.Message(arbitration_id = 0x00, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)
lane_msg_1 = can.Message(arbitration_id = 0x10, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)
lane_msg_2 = can.Message(arbitration_id = 0x11, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)
stop_msg   = can.Message(arbitration_id = 0x20, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)
eng_msg    = can.Message(arbitration_id = 0x30, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)
us_msg     = can.Message(arbitration_id = 0x40, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)
stat_img_info_msg   = can.Message(arbitration_id = 0x50, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)
stat_pid_info_msg_1 = can.Message(arbitration_id = 0x51, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)
stat_pid_info_msg_2 = can.Message(arbitration_id = 0x52, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)



#-----------------------------------------------------------------------------------------------------------------------    
    
    
# Initialisierung des Knotens und der abbonierten Topics
def diagnosticnode(NODE_NAME, SUB_TOPIC1, SUB_TOPIC2, SUB_TOPIC3, SUB_TOPIC4, SUB_TOPIC5, SUB_TOPIC6):
    
    rospy.init_node(NODE_NAME, anonymous=True)
    #rate = rospy.Rate(25) #publish Rate wird auf 25 Hz gesetzt, da Kamera maximal 25 Bilder/s liefert

    rospy.Subscriber(SUB_TOPIC1, lane,       callback1)        #subscribe to Spurerkennung
    rospy.Subscriber(SUB_TOPIC2, stopsign,   callback2)        #subscribe to Stoppschild
    rospy.Subscriber(SUB_TOPIC3, carcontrol, callback3)        #subscribe to Motordrehzahl+Lenkwinkel
    rospy.Subscriber(SUB_TOPIC4, Range,      callback4)        #subscribe to Ultraschallsensor
    rospy.Subscriber(SUB_TOPIC5, Image,      callback5)        #subscribe to Rohbild
    rospy.Subscriber(SUB_TOPIC6, UInt16,     callback6)        #subscribe to Motorsteuerung (Längsregelung)

    #while not rospy.is_shutdown():
        #rate.sleep()    # Schleife entsprechend der publish rate, um Wiederholungsfrequenz einzuhalten
        
    rospy.spin()



#-----------------------------------------------------------------------------------------------------------------------


# Routine fuer Signaldaten zur Spurerkennung
def callback1(lane_data):
# Schreibt Signaldaten in log-file im Verzeichnis /home/nvidia/.ros/log/...
    #rospy.loginfo("right:       %s", lane_data.right)
    #rospy.loginfo("left:        %s", lane_data.left)
    #rospy.loginfo("Abweichung:  %s", lane_data.abw)
    #rospy.loginfo("Erkennung:   %s", lane_data.erkennung)
    #rospy.loginfo("Radius:      %s", lane_data.radius)

# Signaldaten muessen zuerst in Byte-Array gewandelt werden    
    lane_right = struct.pack(">H", lane_data.right)         # Little Endian Format, unsigned short Variable (2 Byte)
    lane_left  = struct.pack(">H", lane_data.left)          # Little Endian Format, unsigned short Variable (2 Byte)
    lane_abw   = struct.pack(">h", lane_data.abw)           # Little Endian Format, signed   short Variable (2 Byte) (Eigentlich signed long, aber Aenderungen nur in 2 Byte)
    lane_status= struct.pack(">?", lane_data.erkennung)     # Little Endian Format, bool  Variable          (1 Byte)
    lane_radius= struct.pack(">f", lane_data.radius)        # Little Endian Format, float Variable          (4 Byte)
    
# Message definieren und bei jedem Durchlauf Nutzdaten leeren, ansonsten werden Nutzdaten in andere Botschaften geschrieben    

    

# Byte-Arrays im Motorolla-Format dem CAN-Frame zuweisen
    for i in reversed(range(0, len(lane_right))):           # Uebergabe der Signalwerte in CAN-Nutzdaten
        lane_msg_1.data[i+4] = lane_left[i]                   # Spur links in Byte 2...3  (vlnr)
        lane_msg_1.data[i+6] = lane_right[i]                  # Spur rechts in Byte 0...1 (vlnr)
        
    for i in reversed(range(0, len(lane_radius))):          # Uebergabe der Signalwerte in CAN-Nutzdaten
        lane_msg_1.data[i] = lane_radius[i]                   # Radius in Byte 4...7
    
    for i in reversed(range(0, len(lane_abw))):           # Uebergabe der Signalwerte in CAN-Nutzdaten
        lane_msg_2.data[i+6] = lane_abw[i]                    # Abweichung zur berechneten Spurmitte (vlnr)
        
    lane_msg_2.data[5] = lane_status
    
# CAN-Frame auf Bus senden
    #send_on_can(bus, lane_msg)
    bus.send(lane_msg_1)
    bus.send(lane_msg_2)
    

    
#-----------------------------------------------------------------------------------------------------------------------    

    
# Routine fuer Signaldaten zur Stoppschilderkennung    
def callback2(stop_data):
# Schreibt Signaldaten in log-file im Verzeichnis /home/nvidia/.ros/log/...
    #rospy.loginfo("stop_data: %s", stop_data)

# Signaldaten muessen zuerst in Byte-Array gewandelt werden   
    stop_status = struct.pack(">?", stop_data.erkennung)    # Little Endian Format, bool Variable                 (1 Byte)
    stop_breite = struct.pack(">B", stop_data.breite_px)    # Little Endian Format, unsigned char Variable        (1 Byte)
    stop_hoehe  = struct.pack(">B", stop_data.hoehe_px)     # Little Endian Format, unsigned char Variable        (1 Byte)
    stop_posx   = struct.pack(">H", stop_data.posx_px)      # Little Endian Format, unsigned short Variable       (2 Byte)
    stop_posy   = struct.pack(">H", stop_data.posy_px)      # Little Endian Format, unsigned short Variable       (2 Byte)
    
# Message definieren und bei jedem Durchlauf Nutzdaten leeren, ansonsten werden Nutzdaten noch in andere Botschaften geschrieben    
    #stop_msg = can.Message(arbitration_id = 0x40, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)
    
# Byte-Arrays im Motorola-Format dem CAN-Frame zuweisen
    stop_msg.data[1] = stop_status                          # Status Stoppschild erkannt / nicht erkannt
    stop_msg.data[2] = stop_hoehe                           # Hoehe in px des erkannten Stoppschilds
    stop_msg.data[3] = stop_breite                          # Breite in px des erkannten Stoppschilds
    for i in reversed(range(0, len(stop_posx))):            # Uebergabe der Signalwerte in CAN-Nutzdaten
        stop_msg.data[i+6] = stop_posx[i]                   # horizontale Position im Bild in px
        stop_msg.data[i+4] = stop_posy[i]                   # vertikale Position im Bild in px
        
# CAN-Frame auf Bus senden
    #send_on_can(bus, stop_msg)
    bus.send(stop_msg)
    

    
#-----------------------------------------------------------------------------------------------------------------------     


# Routine fuer Signaldaten zur Motor- und Servosteuerung      
def callback3(eng_data):
# Schreibt Signaldaten in log-file im Verzeichnis /home/nvidia/.ros/log/...
    #rospy.loginfo("eng_data: %s", eng_data)

# Signaldaten muessen zuerst in Byte-Array gewandelt werden      
    eng_rpm     = struct.pack(">H", eng_data.esc)           # Little Endian Format, unsigned short Variable (2 Byte)
    steer_angle = struct.pack(">H", eng_data.servo)         # Little Endian Format, unsigned short Variable (2 Byte)

# Message definieren und bei jedem Durchlauf Nutzdaten leeren, ansonsten werden Nutzdaten noch in andere Botschaften geschrieben      
    #eng_msg = can.Message(arbitration_id = 0x50, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)
 
     
# Byte-Arrays im Motorola-Format dem CAN-Frame zuweisen
    for i in reversed(range(0, len(eng_rpm))):              # Uebergabe der Signalwerte in CAN-Nutzdaten
        eng_msg.data[i+4] = steer_angle[i]                  # Lenkwinkel in Nutzdaten
        eng_msg.data[i+6] = eng_rpm[i]                      # Motordrehzahl in Nutzdaten
            
# CAN-Frame auf Bus senden
    #send_on_can(bus, eng_msg)
    bus.send(eng_msg)
    
    
    
#-----------------------------------------------------------------------------------------------------------------------     


# Routine fuer Signaldaten des Ultraschallsensors
def callback4(ultraschall_front):
# Schreibt Signaldaten in log-file im Verzeichnis /home/nvidia/.ros/log/...
    #rospy.loginfo("US_data: %s", ultraschall_front.max_range)

# Signaldaten muessen zuerst in Byte-Array gewandelt werden
    us_front_entfernung = struct.pack(">f", ultraschall_front.range)        # Little Endian Format, float Variable          (4 Byte) (wird in "ROS_1_3.ino" angelegt)
    us_front_min_range  = struct.pack(">H", ultraschall_front.min_range+50)    # Little Endian Format, unsigned short Variable (2 Byte)
    us_front_max_range  = struct.pack(">H", ultraschall_front.max_range)    # Little Endian Format, unsigned short Variable (2 Byte)

 # Message definieren und bei jedem Durchlauf Nutzdaten leeren, ansonsten werden Nutzdaten noch in andere Botschaften geschrieben        
    #us_msg = can.Message(arbitration_id = 0x60, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)
    
# Byte-Arrays im Motorola-Format dem CAN-Frame zuweisen    
    for i in reversed(range(0, len(us_front_min_range))):         # Uebergabe der Signalwerte in CAN-Nutzdaten
        us_msg.data[i]   = us_front_max_range[i]                    # minimaler Abstand in Byte 6...7
        us_msg.data[i+2] = us_front_min_range[i]                    # maximaler Abstand in Byte 4...5
    for i in reversed(range(0, len(us_front_entfernung))):        # Uebergabe der Signalwerte in CAN-Nutzdaten
        us_msg.data[i+4] = us_front_entfernung[i]                   # Entfernung zum Objekt in cm in Byte 0...3
        
# CAN-Frame auf Bus senden
    bus.send(us_msg)    

    
    
#----------------------------------------------------------------------------------------------------------------------- 
    
    
# Enthaelt aktuell nur den Lenkwinkel, dieser wird aber bereits auch in /car_control uebertragen
# Fuer spaetere Erweiterungen
def callback5(image_data):
    #rospy.loginfo("img_data:  %s", image_data)
    test = True



#-----------------------------------------------------------------------------------------------------------------------


# Enthaelt aktuell nur die Motordrehzahl, diese wird aber bereits auch in /car_control uebertragen
# Fuer spaetere Erweiterungen
def callback6(esc_data):
    #rospy.loginfo("esc_data:  %s", esc_data)
    test = True
    
    
    
    
#-----------------------------------------------------------------------------------------------------------------------    
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
#-----------------------------------------------------------------------------------------------------------------------    
    
    
def reset_send(bus):
    #reset_msg = can.Message(arbitration_id = 0x00, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)
    rospy.sleep(0.05)
    bus.send(reset_msg)
 
 

#----------------------------------------------------------------------------------------------------------------------- 
 
# Statische Infos werden zu Beginn ausgelesen und 1x versendet, aendern sich ueber die Programmlaufzeit nicht
def static_img_infos():
    
    rospy.sleep(0.05)
    
    img_device = rospy.get_param("/autonomous/uvc_camera/device")
    img_fps    = rospy.get_param("/autonomous/uvc_camera/fps")
    img_width  = rospy.get_param("/autonomous/uvc_camera/width")
    img_height = rospy.get_param("/autonomous/uvc_camera/height")
    
# Signaldaten muessen zuerst in Byte-Array gewandelt werden   
    camera_fps    = struct.pack(">H", img_fps)        # Little Endian Format, unsigned short Variable         (2 Byte)
    camera_width  = struct.pack(">H", img_width)      # Little Endian Format, unsigned short Variable        (2 Byte)
    camera_height = struct.pack(">H", img_height)     # Little Endian Format, unsigned short Variable        (2 Byte)

# Byte-Arrays im Motorola-Format dem CAN-Frame zuweisen    
    for i in reversed(range(0, len(camera_width))):         # Uebergabe der Signalwerte in CAN-Nutzdaten
        stat_img_info_msg.data[i+6] = camera_width[i]         # Bildbreite in Pixel (x-Richtung) in Byte 0...1
        stat_img_info_msg.data[i+4] = camera_height[i]        # Bildhoehe  in Pixel (y-Richtung) in Byte 2...3
        stat_img_info_msg.data[i+2] = camera_fps[i]           # Bildwiederholungsrate pro Sekunde in Byte 4...5
    
# CAN-Frame auf Bus senden
    bus.send(stat_img_info_msg)
 
 
 
#-----------------------------------------------------------------------------------------------------------------------


# Statische Infos werden zu Beginn ausgelesen und 1x versendet, aendern sich ueber die Programmlaufzeit nicht
def static_pid_infos():
    
    rospy.sleep(0.05)
    
    pid_Kd           = rospy.get_param("/steering_pid/Kd")
    pid_Ki           = rospy.get_param("/steering_pid/Ki")
    pid_Kp           = rospy.get_param("/steering_pid/Kp")
    pid_lower_limit  = rospy.get_param("/steering_pid/lower_limit")
    pid_upper_limit  = rospy.get_param("/steering_pid/upper_limit")
    pid_windup_limit = rospy.get_param("/steering_pid/windup_limit")
    pid_max_freq     = rospy.get_param("/steering_pid/max_loop_frequency")
    
# Signaldaten muessen zuerst in Byte-Array gewandelt werden   
    pid_Kd    = struct.pack(">f", pid_Kd)                       # Little Endian Format, float Variable          (4 Byte)
    pid_Ki    = struct.pack(">f", pid_Ki)                       # Little Endian Format, float Variable          (4 Byte)
    pid_Kp    = struct.pack(">f", pid_Kp)                       # Little Endian Format, float Variable          (4 Byte)
    pid_lower_limit  = struct.pack(">b", pid_lower_limit)       # Little Endian Format, signed char Variable    (1 Byte)
    pid_upper_limit  = struct.pack(">b", pid_upper_limit)       # Little Endian Format, signed char Variable    (1 Byte)
    pid_windup_limit = struct.pack(">b", pid_windup_limit)      # Little Endian Format, signed char Variable    (1 Byte)
    pid_max_freq     = struct.pack(">b", pid_max_freq)          # Little Endian Format, signed char Variable    (1 Byte)

# Byte-Arrays im Motorola-Format dem CAN-Frame zuweisen    
    for i in reversed(range(0, len(pid_Kd))):         # Uebergabe der Signalwerte in CAN-Nutzdaten
        stat_pid_info_msg_1.data[i+4] = pid_Kd[i]               # D-Anteil (Differenzierer) des PID-Reglers in Byte 0...3
        stat_pid_info_msg_1.data[i]   = pid_Ki[i]               # I-Anteil (Integrierer)    des PID-Reglers in Byte 4...7
        stat_pid_info_msg_2.data[i+4] = pid_Kp[i]               # P-Anteil (Proportional)   des PID-Reglers in Byte 0...3
        
    stat_pid_info_msg_2.data[3] = pid_lower_limit[i]            # Untere Begrenzung des PID-Reglers in Byte 4
    stat_pid_info_msg_2.data[2] = pid_upper_limit[i]            # Obere  Begrenzung des PID-Reglers in Byte 5
    stat_pid_info_msg_2.data[1] = pid_windup_limit[i]           # Windup-Begrenzung des PID-Reglers in Byte 6
    stat_pid_info_msg_2.data[0] = pid_max_freq[i]               # Maximale Reglerfrequenz in Byte 7

# CAN-Frame auf Bus senden
    bus.send(stat_pid_info_msg_1)
    rospy.sleep(0.05)
    bus.send(stat_pid_info_msg_2)
 
 
 
#-----------------------------------------------------------------------------------------------------------------------    
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
#-----------------------------------------------------------------------------------------------------------------------    

def shutdown_routine():
    
    reset_send(bus)
    time.sleep(1)
    bus.shutdown()
    rospy.loginfo("Shutting down node %s", NODE_NAME)
    
    

# Main-Routine, wird nach Initialisierung der Preamble ausgefuehrt
def main():
    global bla
# Warte auf empfangene Botschaft ueber den Bus    
    for msg in bus:
    # Pruefe, ob es sich um Diagnose-Startanweisung handelt, falls nicht, ignoriere Botschaft, falls ja, starte Diagnose-Knoten    
        if msg.arbitration_id == 0x7FF and msg.data[0] == 0x01:     # 0x7FF entspricht hoechster maximaler ID, da 2^11 IDs moeglich

            try:
                reset_send(bus)
            
                static_img_infos()
                static_pid_infos()
                
                diagnosticnode(NODE_NAME, SUB_TOPIC1, SUB_TOPIC2, SUB_TOPIC3, SUB_TOPIC4, SUB_TOPIC5, SUB_TOPIC6)
                rospy.loginfo("Diagnose-Modus gestartet!")
                
                for msg in bus:
                    if msg.arbitration_id == 0x7FF and msg.data[0] == 0x00:
                        print "Test3"                                                            
                        
                        rospy.loginfo("Diagnose-Modus beendet!")
                        #rospy.signal_shutdown("Diagnose shutdown")
                        shutdown_routine()
                    
            except KeyboardInterrupt:
                rospy.loginfo("Shutting down node %s", NODE_NAME)
                #reset_send(bus)
                #time.sleep(1)
                #bus.shutdown()
                #bla.unregister()
                #shutdown_routine()
                #rospy.spin()
                
                
        #while not rospy.is_shutdown():        
 #       if msg.arbitration_id == 0x7FF and msg.data[0] == 0x00:     # 0x7FF entspricht hoechster maximaler ID, da 2^11 IDs moeglich
                                                                        # data = 0x00 beendet Diagnose-Node wieder, wenn er aktiv ist, ansonsten wird 0x00 ignoriert
#            print "Test3"                                                            
#            rospy.loginfo("Diagnose-Modus beendet!")
            
#            rospy.signal_shutdown("Diagnose shutdown")
            #rospy.unregister()
        
        #shutdown_routine()    
    

        #rospy.signal_shutdown("Diagnose-Modus beendet!")
        
if __name__ == '__main__':
    main()
            