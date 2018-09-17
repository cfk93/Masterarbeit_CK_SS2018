#!/usr/bin/env python
# -*- coding: utf-8 -*-

# image_preprocessing_node

# Info: Rosnode zum Imagepreprocessing
# Input: Rohbild
# Output: Binärbild Canny_Edge nur grüne Pixel

# Autor: Fabian Fitzer (fabian.fitzer@evomotiv.de)
# Version: 1.0, 10.01.2018
# Version: 1.1, 20.02.2018 Umstellung auf grüne Spur


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import numpy as np
import cv2

NODE_NAME = "image_preprocessing_node"
SUB_TOPIC = "/autonomous/image_raw"
PUB_TOPIC = "image_preproc_canny"
QUEUE_SIZE = 1


class imagepreprocessingnode:

    def __init__(self, node_name, sub_topic, pub_topic):
        self.bridge = CvBridge()

        self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=QUEUE_SIZE)

        rospy.init_node(node_name, anonymous=True)
        rate = rospy.Rate(30) #publish Rate wird auf 30 Hz gesetzt, da Kamera maximal 30 Bilder/s liefert
        
        self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback) #subscribe to Rohbild
        
        while not rospy.is_shutdown():
            rate.sleep()    # Schleife entsprechend der publish rate, um Wiederholungsfrequenz einzuhalten
        
        rospy.spin()

    def callback(self, data):
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            

        threshold_low = 50
        threshold_high = 100
         
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)             # Graustufenbild
        img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)                 # HSV Farbraum für Farbfilterung

        # lower_yellow = np.array([0, 0, 0], dtype = "uint8")
        # upper_yellow = np.array([180, 255, 50], dtype = "uint8")

        lower_yellow = np.array([35, 80, 50], dtype = "uint8")              # Werte für grün R: 67 G: 134 B: 28 Standard grün: [35, 80, 50]
        upper_yellow = np.array([90, 255, 200], dtype = "uint8")            # [90, 255, 200]

        mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)      # nur grüne Pixel durchlassen
        # mask_white = cv2.inRange(gray_image, 0, 0)
        # mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
        mask_yw_image = cv2.bitwise_and(gray_image, mask_yellow)            # zusammenführen mit Graustufenbild

        gauss_gray = cv2.GaussianBlur(mask_yw_image, (5, 5), 0)             # weichzeichnen
        canny_edges = cv2.Canny(gauss_gray, threshold_low, threshold_high)  # Kantenerkennung


        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(canny_edges, "mono8"))
        except CvBridgeError as e:
            rospy.logerr(e)

        # cv2.imshow('canny', canny_edges)
        # cv2.waitKey(1)

        # cv2.imshow('original', cv_image)
        # cv2.waitKey(1)

        # Visualisierung der ROI
        overlay_img = cv2.add(gray_image,canny_edges)                   # lines added 04.06.2018 by Christof Kary
        overlay_img = cv2.cvtColor(overlay_img, cv2.COLOR_GRAY2RGB)
        cv2.line(overlay_img, (100,120), (540,120), (255,0,0), 2)
        cv2.line(overlay_img, (540,120), (640,400), (255,0,0), 2)
        cv2.line(overlay_img, (640,400), (0  ,400), (255,0,0), 2)
        cv2.line(overlay_img, (0  ,400), (100,120), (255,0,0), 2)
        cv2.imshow('black_img', overlay_img)
        
        cv2.waitKey(1)
            

def main():
    try:
        imagepreprocessingnode(NODE_NAME, SUB_TOPIC, PUB_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()

