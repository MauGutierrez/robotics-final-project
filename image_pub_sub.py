#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os.path as path
import numpy as np
import sys
bridge = CvBridge()

PY3 = sys.version_info[0] == 3

dirPath = path.dirname(path.abspath(__file__))
cascadePath = dirPath + '/haarcascade_files/'

intersection_cascade = cv2.CascadeClassifier(cascadePath + 'cascade.xml')



if PY3:
    xrange = range

def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

def find_squares(img):
    img = cv2.GaussianBlur(img, (5, 5), 0)
    squares = []
    for gray in cv2.split(img):
        for thrs in xrange(0, 255, 26):
            if thrs == 0:
                bin = cv2.Canny(gray, 0, 50, apertureSize=5)
                bin = cv2.dilate(bin, None)
            else:
                _retval, bin = cv2.threshold(gray, thrs, 255, cv2.THRESH_BINARY)
            bin, contours, _hierarchy = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                cnt_len = cv2.arcLength(cnt, True)
                cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
                if len(cnt) == 4 and cv2.contourArea(cnt) > 1000 and cv2.isContourConvex(cnt):
                    cnt = cnt.reshape(-1, 2)
                    max_cos = np.max([angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
                    if max_cos < 0.1:
                        squares.append(cnt)
    return squares

def coloFnc(frame1):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)

    # define range of green color in HSV
    lower_green = np.array([20,100,100])
    upper_green = np.array([40,255,255])

    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_green, upper_green)

    kernel = np.ones((5,5),'int')
    dilated = cv2.dilate(mask,kernel)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame1,frame1, mask=mask)
    ret,thrshed = cv2.threshold(cv2.cvtColor(res,cv2.COLOR_BGR2GRAY),3,255,cv2.THRESH_BINARY)
    _, contours,hier = cv2.findContours(thrshed,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    return contours


def image_callback(ros_image):
  print 'got an image'
  global bridge
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
    print(e)
  frame = cv_image
  ret = cv_image
  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  faces = intersection_cascade.detectMultiScale(gray)
  for (x, y, w, h) in faces:
  #buscar poner estas dos lineas adentro del otro for
	squares = find_squares(frame)
	color = coloFnc(frame)
	for cnt in color:
		#Contour area is taken
		area = cv2.contourArea(cnt)

		if area > 500:
	    		cv2.putText(frame, "Yellow Object Detected", (10,80), cv2.FONT_HERSHEY_SIMPLEX, 1, 1)
	    		cv2.rectangle(frame,(5,40),(400,100),(0,255,255),2)
			cv2.putText(frame, "Interseccion", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
			cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
			cv2.drawContours(frame, squares, -1, (0, 255, 0), 3)
			cv2.waitKey(10)
	#if frame tiene color amarillo, dibujar cuadros

  cv2.imshow("video", frame)
  key = cv2.waitKey(30)

  cap.release()
  cv2.destroyAllWindows()
  
def main(args):
  rospy.init_node('image_converter', anonymous=True)
  #for turtlebot3 waffle
  #image_topic="/camera/rgb/image_raw/compressed"
  #for usb cam
  #image_topic="/usb_cam/image_raw"
  image_sub = rospy.Subscriber("/usb_cam/image_raw",Image, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
