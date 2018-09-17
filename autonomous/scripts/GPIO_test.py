#!/usr/bin/env python
# -*- coding: utf-8 -*-
#PYTHONPATH=$PYTHONPATH:/home/nvidia/EVObot/src:/opt/ros/kinetic/share


import os
import sys
# sys.path.append("/home/nvidia/EVObot/src/")
# sys.path.append("/opt/ros/kinetic/share/")
for p in sys.path:
    print(p)

import rospy
import roslaunch

from twisted.internet import reactor
from sysfs.gpio import Controller
from sysfs.gpio import DIRECTIONS, INPUT, OUTPUT, RISING, FALLING, BOTH
import time

entprellen = []
erg = False
counter = 0

cli_args = ["home/nvidia/EVObot/src/autonomous/launch/lanefollowergray.launch"]

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["home/nvidia/EVObot/src/autonomous/launch/lanefollowergray.launch"])

Controller.available_pins = [388]

def pin_input(number, state):
    # print("Pin '%d' changed to %d state" % (number, state))
    if state == False:
        print("State: %d" % state)

pin = Controller.alloc_pin(388, INPUT, pin_input, FALLING)

while True:
    erg = pin.read()

    if len(entprellen) < 2:
        entprellen.append(erg)
    else:
        entprellen.append(erg)
        entprellen.pop(0)
        time.sleep(0.01)

        if entprellen[1] == False and entprellen[0] == True:       # Entspricht falling edge
            counter = counter+1
            if counter == True:
                print("Programm gestartet")
                #os.system("python /home/nvidia/Desktop/Skripte/lanedetect_bird2.py")
                launch.start()
                #raw.LauncherStarter('autonomous', 'lanefollowergray.launch')


            elif counter != True:
                print counter
                #os.system("kill -9")
                launch.shutdown()
                print("Programm beendet")
                counter = 0