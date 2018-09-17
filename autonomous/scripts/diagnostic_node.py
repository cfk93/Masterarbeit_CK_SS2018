#!/usr/bin/env python
# -*- coding: utf-8 -*-

# diagnostic_node

# Info:     Rosnode der Diagnosefunktion. Es werden sämtliche Botschaften eingelesen, die zwischen den ROS-Nodes ausgetauscht werden
# Input:    Sämtliche Signale, die in ROS publiziert werden, werden durch den diagnostic_node abboniert
# Output:   - (nur externer Output über Busleitung, es werden keine Botschaften in ROS publiziert)

# Autor: Christof Kary (christof.kary@evomotiv.de)
# Version: 1.0, 04.06.2018

import rospy
import time
import can
import struct
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt16
from std_msgs.msg import Float32
from autonomous.msg import lane
from autonomous.msg import stopsign
from autonomous.msg import carcontrol
from sensor_msgs.msg import Range

NODE_NAME = "diagnostic_node"
SUB_TOPIC1 = "/lane_info"               # Publisher-Skript: image_lanedetection_node.py
SUB_TOPIC2 = "/stop_info"               # Publisher-Skript: image_stopsign_node.py
SUB_TOPIC3 = "/car_control"             # Publisher-Skript: car_control_node.py
SUB_TOPIC4 = "/ultrasoundf1"            # Publisher-Skript: ROS_1_3.ino
SUB_TOPIC5 = "/us_abstand"              # Publisher-Skript: esc_control_node.py (Gefiltertes US-Signal als Float64)
SUB_TOPIC6 = "/autonomous/image_raw"    # Publisher-Skript: image_lanedetection_node.py
SUB_TOPIC7 = "/control_esc"             # Publisher-Skript: esc_control_node.py

QUEUE_SIZE = 1

spur = lane()
stopper = stopsign()
controller = carcontrol()
DIAG_ISACTIVE = False

# lane_right = np.int64(0)
# lane_left  = np.int64(0)
# lane_abw   = np.int64(0)
# lane_status= np.int64(0)
# lane_radius= np.float64(0)
# eng_rpm    = np.uint16(0)
# steer_angle= np.uint16(0)

straightee = []
cha = 0
stra = 0

bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)  # Initialisiert den CAN-Bus mit 500 kBit/s

# Message definieren und bei jedem Durchlauf Nutzdaten leeren, ansonsten werden Nutzdaten in andere Botschaften geschrieben
reset_msg   = can.Message(arbitration_id=0x00, data=[0, 0, 0, 0, 0, 0, 0, 0], extended_id=False)
lane_msg_1  = can.Message(arbitration_id=0x10, data=[0, 0, 0, 0, 0, 0, 0, 0], extended_id=False)
lane_msg_2  = can.Message(arbitration_id=0x11, data=[0, 0, 0, 0, 0, 0, 0, 0], extended_id=False)
stop_msg    = can.Message(arbitration_id=0x20, data=[0, 0, 0, 0, 0, 0, 0, 0], extended_id=False)
eng_msg     = can.Message(arbitration_id=0x30, data=[0, 0, 0, 0, 0, 0, 0, 0], extended_id=False)
us_msg      = can.Message(arbitration_id=0x40, data=[0, 0, 0, 0, 0, 0, 0, 0], extended_id=False)
us_msg_gefiltert = can.Message(arbitration_id=0x41, data=[0, 0, 0, 0, 0, 0, 0, 0], extended_id=False)
stat_img_info_msg   = can.Message(arbitration_id=0x50, data=[0, 0, 0, 0, 0, 0, 0, 0], extended_id=False)
stat_pid_info_msg_1 = can.Message(arbitration_id=0x51, data=[0, 0, 0, 0, 0, 0, 0, 0], extended_id=False)
stat_pid_info_msg_2 = can.Message(arbitration_id=0x52, data=[0, 0, 0, 0, 0, 0, 0, 0], extended_id=False)


# -----------------------------------------------------------------------------------------------------------------------


# Initialisierung des Knotens und der abonnierten Topics
def diagnosticnode(NODE_NAME, SUB_TOPIC1, SUB_TOPIC2, SUB_TOPIC3, SUB_TOPIC4, SUB_TOPIC5, SUB_TOPIC6):
    """
    :param NODE_NAME:  diagnostic_node
    :param SUB_TOPIC1: lane_info
    :param SUB_TOPIC2: stop_info
    :param SUB_TOPIC3: car_control
    :param SUB_TOPIC4: ultrasoundf1
    :param SUB_TOPIC5: autonomous/image_raw TBA!
    :param SUB_TOPIC6: control_esc TBA!
    """
    rospy.init_node(NODE_NAME, anonymous=True)
    # rate = rospy.Rate(30) #publish Rate wird auf 30 Hz gesetzt, da Kamera maximal 30 Bilder/s liefert
    # Wird in diesem Skript nicht verwendet, da Busbotschaften immer bei Aufruf eines Callbacks bzw Erhalten einer abonnierten Botschaft gesendet werden

    rospy.loginfo("Diagnose-Modus gestartet!")

    a = rospy.Subscriber(SUB_TOPIC1, lane,       callback1) # subscribe to Spurerkennung
    b = rospy.Subscriber(SUB_TOPIC2, stopsign,   callback2) # subscribe to Stoppschild
    c = rospy.Subscriber(SUB_TOPIC3, carcontrol, callback3) # subscribe to Motordrehzahl+Lenkwinkel
    d = rospy.Subscriber(SUB_TOPIC4, Range,      callback4) # subscribe to Ultraschallsensor
    e = rospy.Subscriber(SUB_TOPIC5, Float32,    callback5) # subscribe to gefiltertes US-Signal
    f = rospy.Subscriber(SUB_TOPIC6, Image,      callback6) # subscribe to Rohbild
    g = rospy.Subscriber(SUB_TOPIC7, UInt16,     callback7) # subscribe to Motorsteuerung (Längsregelung)

    while not rospy.is_shutdown():
        for msg in bus:
            if msg.arbitration_id == 0x01 and msg.data[0] == 0x01:

                try:
                    DIAG_ISACTIVE = True
                    reset_send(bus)
                    static_img_infos()
                    static_pid_infos()
                    diagnosticnode(NODE_NAME, SUB_TOPIC1, SUB_TOPIC2, SUB_TOPIC3, SUB_TOPIC4, SUB_TOPIC5, SUB_TOPIC6)  # Re-Subscribe nach Bus Pause
                except KeyboardInterrupt:
                    rospy.loginfo("Shutting down node %s", NODE_NAME)


            elif msg.arbitration_id == 0x01 and msg.data[0] == 0x00:
                try:
                    DIAG_ISACTIVE = False

                    a.unregister()  # unsubscribe von allen Messages, damit wird Bus-Uebertragung gestoppt
                    b.unregister()
                    c.unregister()
                    d.unregister()
                    e.unregister()
                    f.unregister()
                    g.unregister()

                    reset_send(bus)

                    rospy.loginfo("Diagnose-Modus beendet!")
                except KeyboardInterrupt:
                    rospy.loginfo("Shutting down node %s", NODE_NAME)

    rospy.spin()


# -----------------------------------------------------------------------------------------------------------------------


# Routine fuer Signaldaten zur Spurerkennung
def callback1(lane_data):
    """
    :param lane_data: Pixelwert Spur links, Spur rechts, Abweichung Spurmitte, Status Spurerkennung, Kurvenradius in m
    """
    global DIAG_ISACTIVE
    # Schreibt Signaldaten in log-file im Verzeichnis /home/nvidia/.ros/log/...
    # rospy.loginfo("right:       %s", lane_data.right)
    # rospy.loginfo("left:        %s", lane_data.left)
    # rospy.loginfo("Abweichung:  %s", lane_data.abw)
    # rospy.loginfo("Erkennung:   %s", lane_data.erkennung)
    # rospy.loginfo("Radius:      %s", lane_data.radius)

    # Signaldaten muessen zuerst in Byte-Array gewandelt werden
    lane_right  = struct.pack(">H", lane_data.right)      # Little Endian Format, unsigned short Variable (2 Byte)
    lane_left   = struct.pack(">H", lane_data.left)       # Little Endian Format, unsigned short Variable (2 Byte)
    lane_abw    = struct.pack(">h", lane_data.abw)        # Little Endian Format, signed short Variable   (2 Byte)
    lane_status = struct.pack(">?", lane_data.erkennung)  # Little Endian Format, bool  Variable          (1 Byte)
    lane_radius = struct.pack(">f", lane_data.radius)     # Little Endian Format, float Variable          (4 Byte)

    # Message definieren und bei jedem Durchlauf Nutzdaten leeren, ansonsten werden Nutzdaten in andere Botschaften geschrieben

    # Byte-Arrays im Motorolla-Format dem CAN-Frame zuweisen
    for i in reversed(range(0, len(lane_right))):   # Uebergabe der Signalwerte in CAN-Nutzdaten
        lane_msg_1.data[i + 4] = lane_left[i]       # Spur links in Byte 2...3  (vlnr)
        lane_msg_1.data[i + 6] = lane_right[i]      # Spur rechts in Byte 0...1 (vlnr)

    for i in reversed(range(0, len(lane_radius))):  # Uebergabe der Signalwerte in CAN-Nutzdaten
        lane_msg_1.data[i] = lane_radius[i]         # Radius in Byte 4...7

    for i in reversed(range(0, len(lane_abw))):     # Uebergabe der Signalwerte in CAN-Nutzdaten
        lane_msg_2.data[i + 6] = lane_abw[i]        # Abweichung zur berechneten Spurmitte (vlnr)

    lane_msg_2.data[5] = lane_status

    # CAN-Frame auf Bus senden
    if DIAG_ISACTIVE:
        bus.send(lane_msg_1)
        bus.send(lane_msg_2)


# -----------------------------------------------------------------------------------------------------------------------


# Routine fuer Signaldaten zur Stoppschilderkennung    
def callback2(stop_data):
    """
    :param stop_data: Status Erkennung Stoppschild, Breite Stoppschild in Pixel, Höhe Stoppschild in Pixel, Pixelwert horizontal, Pixelwert vertikal
    """
    global DIAG_ISACTIVE
    # Schreibt Signaldaten in log-file im Verzeichnis /home/nvidia/.ros/log/...
    # rospy.loginfo("stop_data: %s", stop_data)

    # Signaldaten muessen zuerst in Byte-Array gewandelt werden
    stop_status = struct.pack(">?", stop_data.erkennung)    # Little Endian Format, bool Variable                 (1 Byte)
    stop_breite = struct.pack(">B", stop_data.breite_px)    # Little Endian Format, unsigned char Variable        (1 Byte)
    stop_hoehe  = struct.pack(">B", stop_data.hoehe_px)     # Little Endian Format, unsigned char Variable        (1 Byte)
    stop_posx   = struct.pack(">H", stop_data.posx_px)      # Little Endian Format, unsigned short Variable       (2 Byte)
    stop_posy   = struct.pack(">H", stop_data.posy_px)      # Little Endian Format, unsigned short Variable       (2 Byte)

    # Message definieren und bei jedem Durchlauf Nutzdaten leeren, ansonsten werden Nutzdaten noch in andere Botschaften geschrieben
    # stop_msg = can.Message(arbitration_id = 0x40, data = [0, 0, 0, 0, 0, 0, 0, 0], extended_id = False)

    # Byte-Arrays im Motorola-Format dem CAN-Frame zuweisen
    stop_msg.data[1] = stop_status  # Status Stoppschild erkannt / nicht erkannt
    stop_msg.data[2] = stop_hoehe   # Hoehe in px des erkannten Stoppschilds
    stop_msg.data[3] = stop_breite  # Breite in px des erkannten Stoppschilds

    for i in reversed(range(0, len(stop_posx))):    # Uebergabe der Signalwerte in CAN-Nutzdaten
        stop_msg.data[i + 6] = stop_posx[i]         # horizontale Position im Bild in px
        stop_msg.data[i + 4] = stop_posy[i]         # vertikale Position im Bild in px

    # CAN-Frame auf Bus senden
    if DIAG_ISACTIVE:
        bus.send(stop_msg)


# -----------------------------------------------------------------------------------------------------------------------


# Routine fuer Signaldaten zur Motor- und Servosteuerung      
def callback3(eng_data):
    """
    :param eng_data: PWM-Signal Servo, PWM-Signal ESC
    """
    global DIAG_ISACTIVE
    # Schreibt Signaldaten in log-file im Verzeichnis /home/nvidia/.ros/log/...
    # rospy.loginfo("eng_data: %s", eng_data)

    # Signaldaten muessen zuerst in Byte-Array gewandelt werden
    eng_rpm     = struct.pack(">H", eng_data.esc)    # Little Endian Format, unsigned short Variable (2 Byte)
    steer_angle = struct.pack(">H", eng_data.servo)  # Little Endian Format, unsigned short Variable (2 Byte)

    # Byte-Arrays im Motorola-Format dem CAN-Frame zuweisen
    for i in reversed(range(0, len(eng_rpm))):  # Uebergabe der Signalwerte in CAN-Nutzdaten
        eng_msg.data[i + 4] = steer_angle[i]    # Lenkwinkel in Nutzdaten
        eng_msg.data[i + 6] = eng_rpm[i]        # Motordrehzahl in Nutzdaten

    # CAN-Frame auf Bus senden
    if DIAG_ISACTIVE:
        bus.send(eng_msg)


# -----------------------------------------------------------------------------------------------------------------------


# Routine fuer Signaldaten des Ultraschallsensors
def callback4(ultraschall_front):
    """
    :param ultraschall_front: Abstand in m, minimaler Abstand, maximaler Abstand
    """
    global DIAG_ISACTIVE
    # Schreibt Signaldaten in log-file im Verzeichnis /home/nvidia/.ros/log/...
    # rospy.loginfo("US_data: %s", ultraschall_front.max_range)

    # Signaldaten muessen zuerst in Byte-Array gewandelt werden
    us_front_entfernung = struct.pack(">f", ultraschall_front.range)            # Little Endian Format, float Variable          (4 Byte)
    us_front_min_range  = struct.pack(">H", ultraschall_front.min_range + 50)   # Little Endian Format, unsigned short Variable (2 Byte)
    us_front_max_range  = struct.pack(">H", ultraschall_front.max_range)        # Little Endian Format, unsigned short Variable (2 Byte)

    # Byte-Arrays im Motorola-Format dem CAN-Frame zuweisen
    for i in reversed(range(0, len(us_front_min_range))):   # Uebergabe der Signalwerte in CAN-Nutzdaten
        us_msg.data[i] = us_front_max_range[i]              # minimaler Abstand in Byte 6...7
        us_msg.data[i + 2] = us_front_min_range[i]          # maximaler Abstand in Byte 4...5
    for i in reversed(range(0, len(us_front_entfernung))):  # Uebergabe der Signalwerte in CAN-Nutzdaten
        us_msg.data[i + 4] = us_front_entfernung[i]         # Entfernung zum Objekt in cm in Byte 0...3

    # CAN-Frame auf Bus senden
    if DIAG_ISACTIVE:
        bus.send(us_msg)


# -----------------------------------------------------------------------------------------------------------------------


# Routine für Signaldaten des gefilterten Ultraschallsensors
def callback5(us_gefiltert):
    """
    :param us_gefiltert: Gefiltertes US-Signal, Rohsignal wird in Knoten esc_control_node.py gefiltert
    """
    global DIAG_ISACTIVE
    # Schreibt Signaldaten in log-file im Verzeichnis /home/nvidia/.ros/log/...
    # rospy.loginfo("US_data_gefiltert:  %s", us_gefiltert)

    # Signaldaten muessen zuerst in Byte-Array gewandelt werden
    us_front_entf_gefiltert = struct.pack(">f", us_gefiltert.data)          # Little Endian Format, float Variable          (4 Byte)

    # Byte-Arrays im Motorola-Format dem CAN-Frame zuweisen
    for i in reversed(range(0, len(us_front_entf_gefiltert))):  # Uebergabe der Signalwerte in CAN-Nutzdaten
        us_msg_gefiltert.data[i + 4] = us_front_entf_gefiltert[i]         # Entfernung zum Objekt in cm in Byte 0...3

    # CAN-Frame auf Bus senden
    if DIAG_ISACTIVE:
        bus.send(us_msg_gefiltert)

# -----------------------------------------------------------------------------------------------------------------------


# Routine für Übertragung des Rohbildes, eigentlich nur zum Test, Rohbild kann aber theoretisch auch über CAN übertragen werden
def callback6(image_data):
    """
    :param image_data: Rohbild
    """
    global DIAG_ISACTIVE
    # rospy.loginfo("img_data:  %s", image_data)
    test = True


# -----------------------------------------------------------------------------------------------------------------------


# Routine für Übertragung der ESC-Fahrdaten, wird aktuell nicht verwendet, für spätere Erweiterungen
def callback7(esc_data):
    """
    :param esc_data: PWM-Signal ESC
    """
    global DIAG_ISACTIVE
    # rospy.loginfo("esc_data:  %s", esc_data)
    test = True



# -----------------------------------------------------------------------------------------------------------------------
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# -----------------------------------------------------------------------------------------------------------------------


def reset_send(bus):
    """
    :param bus: Bustyp, Channel, Baudrate
    """
    reset_msg = can.Message(arbitration_id=0x00, data=[0, 0, 0, 0, 0, 0, 0, 0], extended_id=False)
    rospy.sleep(0.05)
    bus.send(reset_msg)


# -----------------------------------------------------------------------------------------------------------------------

# Statische Infos werden zu Beginn ausgelesen und 1x versendet, ändern sich über die Programmlaufzeit nicht
def static_img_infos():
    """
    Statische Infos des Image-Streams
    """
    global DIAG_ISACTIVE

    rospy.sleep(0.05)

    img_device = rospy.get_param("/autonomous/uvc_camera/device")
    img_fps    = rospy.get_param("/autonomous/uvc_camera/fps")
    img_width  = rospy.get_param("/autonomous/uvc_camera/width")
    img_height = rospy.get_param("/autonomous/uvc_camera/height")

    # Signaldaten muessen zuerst in Byte-Array gewandelt werden
    camera_fps    = struct.pack(">H", img_fps)      # Little Endian Format, unsigned short Variable     (2 Byte)
    camera_width  = struct.pack(">H", img_width)    # Little Endian Format, unsigned short Variable     (2 Byte)
    camera_height = struct.pack(">H", img_height)   # Little Endian Format, unsigned short Variable     (2 Byte)

    # Byte-Arrays im Motorola-Format dem CAN-Frame zuweisen
    for i in reversed(range(0, len(camera_width))):         # Uebergabe der Signalwerte in CAN-Nutzdaten
        stat_img_info_msg.data[i + 6] = camera_width[i]     # Bildbreite in Pixel (x-Richtung) in Byte 0...1
        stat_img_info_msg.data[i + 4] = camera_height[i]    # Bildhoehe  in Pixel (y-Richtung) in Byte 2...3
        stat_img_info_msg.data[i + 2] = camera_fps[i]       # Bildwiederholungsrate pro Sekunde in Byte 4...5

    # CAN-Frame auf Bus senden
    if DIAG_ISACTIVE:
        bus.send(stat_img_info_msg)


# -----------------------------------------------------------------------------------------------------------------------


# Statische Infos werden zu Beginn ausgelesen und 1x versendet, ändern sich über die Programmlaufzeit nicht
def static_pid_infos():
    """
    Statische Infos zum PID-Regler
    """
    global DIAG_ISACTIVE

    rospy.sleep(0.05)

    pid_Kd = rospy.get_param("/steering_pid/Kd")
    pid_Ki = rospy.get_param("/steering_pid/Ki")
    pid_Kp = rospy.get_param("/steering_pid/Kp")
    pid_lower_limit  = rospy.get_param("/steering_pid/lower_limit")
    pid_upper_limit  = rospy.get_param("/steering_pid/upper_limit")
    pid_windup_limit = rospy.get_param("/steering_pid/windup_limit")
    pid_max_freq     = rospy.get_param("/steering_pid/max_loop_frequency")

    # Signaldaten muessen zuerst in Byte-Array gewandelt werden
    pid_Kd = struct.pack(">f", pid_Kd)                     # Little Endian Format, float Variable          (4 Byte)
    pid_Ki = struct.pack(">f", pid_Ki)                     # Little Endian Format, float Variable          (4 Byte)
    pid_Kp = struct.pack(">f", pid_Kp)                     # Little Endian Format, float Variable          (4 Byte)
    pid_lower_limit  = struct.pack(">b", pid_lower_limit)  # Little Endian Format, signed char Variable    (1 Byte)
    pid_upper_limit  = struct.pack(">b", pid_upper_limit)  # Little Endian Format, signed char Variable    (1 Byte)
    pid_windup_limit = struct.pack(">b", pid_windup_limit) # Little Endian Format, signed char Variable    (1 Byte)
    pid_max_freq     = struct.pack(">b", pid_max_freq)     # Little Endian Format, signed char Variable    (1 Byte)

    # Byte-Arrays im Motorola-Format dem CAN-Frame zuweisen
    for i in reversed(range(0, len(pid_Kd))):         # Uebergabe der Signalwerte in CAN-Nutzdaten
        stat_pid_info_msg_1.data[i + 4] = pid_Kd[i]   # D-Anteil (Differenzierer) des PID-Reglers in Byte 0...3
        stat_pid_info_msg_1.data[i] = pid_Ki[i]       # I-Anteil (Integrierer)    des PID-Reglers in Byte 4...7
        stat_pid_info_msg_2.data[i + 4] = pid_Kp[i]   # P-Anteil (Proportional)   des PID-Reglers in Byte 0...3

    stat_pid_info_msg_2.data[3] = pid_lower_limit[i]  # Untere Begrenzung des PID-Reglers in Byte 4
    stat_pid_info_msg_2.data[2] = pid_upper_limit[i]  # Obere  Begrenzung des PID-Reglers in Byte 5
    stat_pid_info_msg_2.data[1] = pid_windup_limit[i] # Windup-Begrenzung des PID-Reglers in Byte 6
    stat_pid_info_msg_2.data[0] = pid_max_freq[i]     # Maximale Reglerfrequenz in Byte 7

    # CAN-Frame auf Bus senden
    if DIAG_ISACTIVE:
        bus.send(stat_pid_info_msg_1)
        rospy.sleep(0.05)
        bus.send(stat_pid_info_msg_2)


# -----------------------------------------------------------------------------------------------------------------------
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# -----------------------------------------------------------------------------------------------------------------------

def shutdown_routine():
    """
    Routine zum Shutdown des Bus-Knotens, wird aktuell nicht verwendet
    """
    reset_send(bus)
    time.sleep(1)
    bus.shutdown()
    rospy.loginfo("Shutting down node %s", NODE_NAME)


# Main-Routine, wird nach Initialisierung der Preamble ausgeführt
def main():
    """
    main-Routine zur Initialisierung
    """
    global DIAG_ISACTIVE
    # Warte auf empfangene Botschaft über den Bus, wir 1x initial durchlaufen, sobald Diagnose aktiviert, Schleife nur in diagnosticnode, bis manueller Abbruch
    for msg in bus:
        # Prüfe, ob es sich um Diagnose-Startanweisung handelt, falls nicht, ignoriere Botschaft, falls ja, starte Diagnose-Knoten
        if msg.arbitration_id == 0x01 and msg.data[0] == 0x01:  # 0x7FF entspricht höchster maximaler ID, da 2^11 IDs möglich

            try:
                DIAG_ISACTIVE = True
                reset_send(bus)

                static_img_infos()
                static_pid_infos()

                diagnosticnode(NODE_NAME, SUB_TOPIC1, SUB_TOPIC2, SUB_TOPIC3, SUB_TOPIC4, SUB_TOPIC5, SUB_TOPIC6)

            except KeyboardInterrupt:
                rospy.loginfo("Shutting down node %s", NODE_NAME)
                reset_send(bus)
                time.sleep(1)
                bus.shutdown()


if __name__ == '__main__':
    main()