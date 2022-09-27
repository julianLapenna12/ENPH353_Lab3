#! /usr/bin/env python3

from __future__ import print_function
from concurrent.futures import process

#import roslib; roslib.load_manifest('node')
import sys
import rospy
import cv2
import time
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


class image_converter:

  def __init__(self):
    self.twist_pub = rospy.Publisher("/cmd_vel",Twist)
    self.move = Twist()

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)
    

  def PID(self,input,setpoint):
    kp = 0.5
    ki = 0.01
    kd = 1

    current_time = time.time()

    #positive when on the left, negative on the right
    error = setpoint - input

    #rate_error = (error - last_error) / elapsed_time
    out = kp * error

    return out


  def process_image(self,image):
    #image processing
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray,(3,3),0)
    ret,th = cv2.threshold(blur,100,255,cv2.THRESH_BINARY_INV)

    return th

    
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    processed_im = self.process_image(cv_image)

    # draw contours on the original image
    contours, hierarchy = cv2.findContours(image=processed_im, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    largest_item= sorted(contours, key=cv2.contourArea, reverse= True)[0]
    M = cv2.moments(largest_item)

    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    disp = cv2.circle(processed_im, (cx, cy), 2, (0,255,0), 2)
    cv2.drawContours(image=disp, contours=contours, contourIdx=0, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    # see the results
    cv2.imshow('None approximation', disp)

    feedback = self.PID(cx, int(processed_im.shape[1]/2)) #change to be modular `int(th.shape[1]/2)`

    self.move.linear.x = 0.15
    self.move.angular.z = feedback/100

    try:
      self.twist_pub.publish(self.move)
    except CvBridgeError as e:
      print(e)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)