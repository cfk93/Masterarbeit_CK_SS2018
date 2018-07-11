#!/usr/bin/env python
# -*- coding: utf-8 -*-

# image_lanedetection_node

# Info: Rosnode zur Spurerkennung
# Input: Canny_Bild
# Output: Kurvenradius, Abweichung von Fahrspurmitte

# Autor: Fabian Fitzer (fabian.fitzer@evomotiv.de)
# Version: 1.0, 10.01.2018
# Version: 1.1, 25.02.2018 verbesserter Algorithmus für grüne Spur


import rospy
from sensor_msgs.msg import Image
from autonomous.msg import lane
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np
import cv2

NODE_NAME = "image_lanedetection_node"
SUB_TOPIC = "/image_preproc_canny"
PUB_TOPIC1 = "lane_info"
PUB_TOPIC2 = "abw_steering"
PUB_TOPIC3 = "set_steering"
QUEUE_SIZE = 1
left_fit_mean = []
right_fit_mean = []
radius = []
setp = 0
spur = lane()
right_mean = []
left_mean = []
nwindows = 10
line_dst_offset = 200
rightlist = []
leftlist = []
rmiss = 0
lmiss = 0



class imagelanedetectionnode:

    def __init__(self, node_name, sub_topic, pub_topic1, pub_topic2, pub_topic3):
        self.bridge = CvBridge()

        self.lane_pub = rospy.Publisher(pub_topic1, lane, queue_size=QUEUE_SIZE)
        self.state_pub = rospy.Publisher(pub_topic2, Float64, queue_size=QUEUE_SIZE)
        self.set_pub = rospy.Publisher(pub_topic3, Float64, queue_size=QUEUE_SIZE)
        
        rospy.init_node(node_name, anonymous=True)
        rate = rospy.Rate(25) #publish Rate wird auf 25 Hz gesetzt, da Kamera maximal 25 Bilder/s liefert
        
        self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback, [rmiss, lmiss]) #subscribe to Kantenbild
        
        while not rospy.is_shutdown():
            rate.sleep()    # Schleife entsprechend der publish rate, um Wiederholungsfrequenz einzuhalten
        
        rospy.spin()
        

    def callback(self, data, miss):

        try:
            canny_edges = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)

        w = canny_edges.shape[1]
        h = canny_edges.shape[0]
        n = 0
        m = 0
        setp = 0
        setpr = 0
        setpl = 0
        

	transform_src = np.float32([[210,120],[430,120],[640,400],[0,400]]) #Festlegung ROI
        transform_dst = np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]])
        
	M = cv2.getPerspectiveTransform(transform_src, transform_dst) #Transformationsmatrix

	warped_image = cv2.warpPerspective(canny_edges, M, (w,h), flags=cv2.INTER_LINEAR) #Berechnung transformiertes Bild
	histogram = np.sum(warped_image[warped_image.shape[0]/2:,:], axis=0)   #Pixelmaximum Bestimmung

	out_img = np.dstack((warped_image, warped_image, warped_image))*255

        midpoint = np.int(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint   
 
	window_height = np.int(warped_image.shape[0] / nwindows) #Berechnung der Fensterhöhe

	nonzero = warped_image.nonzero()
	nonzeroy = np.array(nonzero[0])
	nonzerox = np.array(nonzero[1])

	leftx_current = leftx_base
	rightx_current = rightx_base
 
	margin = 80

	minpix = 40
	#print leftx_current
 
	left_lane_inds = []
	right_lane_inds = []
	


        
        #Schleife durch alle Fenster Bestimmung Stützstelle für jedes
	for window in range(nwindows):

		win_y_low = warped_image.shape[0] - (window + 1) * window_height
		win_y_high = warped_image.shape[0] - window * window_height
		win_xleft_low = leftx_current - margin
		win_xleft_high = leftx_current + margin
		win_xright_low = rightx_current - margin
		win_xright_high = rightx_current + margin

		good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        	good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

		left_lane_inds.append(good_left_inds)
		right_lane_inds.append(good_right_inds)

		if len(good_left_inds) > minpix:
			leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
		if len(good_right_inds) > minpix:
			rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
                
                if window == 1:
                    right = rightx_current          # in horizontale Pixel, maximale Spurbreite: links 0 px, rechts 640 py 
                    left = leftx_current
        
        
        #Filterung der erkannten Fahrbahnmarkierung
        if len(rightlist)<10:
            leftlist.append(left)
            rightlist.append(right)
        else:
            rightlist.append(right)
            rightlist.pop(0)
            if abs(rightlist[9]-rightlist[8])>200:
                #print rightlist
                miss[0] = 1
                    
            leftlist.append(left)
            leftlist.pop(0)
            if abs(leftlist[9]-leftlist[8])>200:
                miss[1] = 1
                #print leftlist[9]-leftlist[8]
            if (rightlist[9]-leftlist[9])<200:
                if rightlist[9]<320 and leftlist[9]<320:
                    rightlist[9] = 640
                elif rightlist[9]>320 and leftlist[9]>320:
                    leftlist[9] = 0
                
            if abs(rightlist[9]-rightlist[8])>4:
                rightlist[9] = rightlist[8]+((-1 if((rightlist[9]-rightlist[8])<0) else 1)*4)
                
            if abs(leftlist[9]-leftlist[8])>4:
                leftlist[9] = leftlist[8]+((-1 if((leftlist[9]-leftlist[8])<0) else 1)*4)   
                
            spur.right = (rightlist[9]+rightlist[8]+rightlist[7]+rightlist[6])/4
            spur.left = (leftlist[9]+leftlist[8]+leftlist[7]+leftlist[6])/4
        
        
        
        #Berechnung der Abweichung von der Spurmitte
        spur.abw = (w/2 - (spur.right + spur.left)/2)
        
        #Abweichung zwischen -8 und +8 wird als Fahrschlauch festgelegt
        if spur.abw > 8:
            spur.abw = (spur.abw/4)-8
            #spur.abw = (0.0253*(spur.abw*spur.abw))-(0.2783*spur.abw)+(0.5538)
        elif spur.abw < -8:
            spur.abw = (spur.abw/4)+8
            #spur.abw = (-0.0253*(spur.abw*spur.abw))-(0.2783*spur.abw)-(0.5538)
        else:
            spur.abw = 0
         
	
	
	#Polynominterpolation für rechte und linke Spur
	left_lane_inds = np.concatenate(left_lane_inds)
	right_lane_inds = np.concatenate(right_lane_inds)

	leftx = nonzerox[left_lane_inds]
	lefty = nonzeroy[left_lane_inds]
	rightx = nonzerox[right_lane_inds]
	righty = nonzeroy[right_lane_inds]

        if leftx.size!=0 and lefty.size!=0 and rightx.size!=0 and righty.size!=0:
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)
            
            if len(left_fit_mean)<10 and len(right_fit_mean)<10:
                left_fit_mean.append(left_fit)
                right_fit_mean.append(right_fit)
                left_fit = sum(left_fit_mean)/len(left_fit_mean)
                right_fit = sum(right_fit_mean)/len(right_fit_mean)
            else:
                left_fit_mean.append(left_fit)
                left_fit_mean.append(left_fit)
                right_fit_mean.append(right_fit)
                right_fit_mean.append(right_fit)
                left_fit_mean.pop(9)
                left_fit_mean.pop(0)
                right_fit_mean.pop(9)
                right_fit_mean.pop(0)
                left_fit = sum(left_fit_mean)/10
                right_fit = sum(right_fit_mean)/10

            
            warp_zero = np.zeros_like(warped_image).astype(np.uint8)
            color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

            ploty = np.linspace(0, out_img.shape[0] - 1, out_img.shape[0])

            left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
            right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
	
            pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
            pts = np.hstack((pts_left, pts_right))
            
            #Umrechnung Pixel in Meter für Radiusbestimmung
            ym_per_pix = 0.97 / 480
            xm_per_pix = 0.42 / 640 
           
            y_eval = 640
    
            left_fit_cr = np.polyfit(ploty * ym_per_pix, left_fitx * xm_per_pix, 2)
            right_fit_cr = np.polyfit(ploty * ym_per_pix, right_fitx * xm_per_pix, 2)

            #Berechnung Kurvenradius, "Anlegen" eines Kreises an Kurve
            left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit_cr[0])

            right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit_cr[0])


            if len(radius)<10:
                radius.append(round((float(left_curverad) + float(right_curverad))/2.,2))
            else:
                radius.append(round((float(left_curverad) + float(right_curverad))/2.,2))
                radius.append(round((float(left_curverad) + float(right_curverad))/2.,2))
                radius.pop(9)
                radius.pop(0)
                radius_mean = sum(radius)/10
                spur.radius = radius_mean
            
        #Erkennung Abkommen von der Spur und Veränderung des Setpoint, um auf Spur zurück zu fahren
        if (spur.right - spur.left)<150:
            if miss[0]:
                setp = 200
                
            if miss[1]:
                setp = -200
        else:
            setp = 0        
            miss[0] = 0
            miss[1] = 0
        
        
        if spur.right == 320 and spur.left == 0:
            spur.abw = 0
            spur.erkennung = 0
        else:
            #if spur.right == 320:
                #setp = 200
            spur.erkennung = 1
    
        
        
        try:
            self.lane_pub.publish(spur)
            self.state_pub.publish(spur.abw)
            self.set_pub.publish(setp)
        except CvBridgeError as e:
            rospy.logerr(e)
	
            


def main():
    try:
        imagelanedetectionnode(NODE_NAME, SUB_TOPIC, PUB_TOPIC1, PUB_TOPIC2, PUB_TOPIC3)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
