#! /usr/bin/env python3

from __future__ import print_function

#import roslib; roslib.load_manifest('node')
import sys
import rospy
import cv2
import time
import math
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
    ret,th = cv2.threshold(blur,100,255,cv2.THRESH_BINARY)

    return th

  
  def get_road_center(self,frame):
    #looking at info in the 9/10*height row of pixels
    line1 = frame[int(9*frame.shape[0]/10)]
    line2 = frame[int(9*frame.shape[0]/10) - 40]
    line3 = frame[int(9*frame.shape[0]/10) - 80]
    line4 = frame[int(9*frame.shape[0]/10) - 120]

    a = 6
    b = 4
    c = 2
    d = 1
    e = 0

    #getting the average of the road pixels
    road_pixels1 = []
    road_pixels2 = []
    road_pixels3 = []
    road_pixels4 = []
    
    for x in range(len(line1)):
      if line1[x] == 0:
        road_pixels1.append(x)
      if line2[x] == 0:
        road_pixels2.append(x)
      if line3[x] == 0:
        road_pixels3.append(x)
      if line4[x] == 0:
        road_pixels4.append(x)

    road_center1 = sum(road_pixels1)/(len(road_pixels1) + 1)
    road_center2 = sum(road_pixels2)/(len(road_pixels2) + 1)
    road_center3 = sum(road_pixels3)/(len(road_pixels3) + 1)
    road_center4 = sum(road_pixels4)/(len(road_pixels4) + 1)

    if (road_center4 == 0):
      d = 0

    if (road_center3 == 0):
      c = 0   

    if (road_center2 == 0):
      b = 0
    
    if (road_center1 == 0):
      a = 0

    if (a == 0 and b ==0 and c == 0 and d ==0):
      e = 1

    

    road_center_avg = (road_center1*a + road_center2*b + road_center3*c + road_center4*d)/(a+b+c+d+e)

    return [road_center_avg, road_center1, road_center2, road_center3, road_center4]

    
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    processed_im = self.process_image(cv_image)

    road_centers = self.get_road_center(processed_im)

    feedback = self.PID(road_centers[0], int(processed_im.shape[1]/2)) #change to be modular `int(th.shape[1]/2)`

    self.move.linear.x = 0.15
    self.move.angular.z = feedback/100

    h = int(9*cv_image.shape[0]/10)

    col_im = cv2.cvtColor(processed_im,cv2.COLOR_GRAY2RGB)
    disp1 = cv2.circle(col_im, (math.floor(road_centers[1]), h), 2, (255,0,0), 2)
    disp2 = cv2.circle(disp1, (math.floor(road_centers[2]), h - 40), 2, (255,0,0), 2)
    disp3 = cv2.circle(disp2, (math.floor(road_centers[3]), h - 80), 2, (255,0,0), 2)
    disp4 = cv2.circle(disp3, (math.floor(road_centers[4]), h - 120), 2, (255,0,0), 2)

    # ##### Code that displays contours, not currently working in my alternate file
    # # draw contours on the original image
    # gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    # blur = cv2.GaussianBlur(gray,(3,3),0)
    # ret,th = cv2.threshold(blur,100,255,cv2.THRESH_BINARY_INV)
    # contours, hierarchy = cv2.findContours(image=th, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    # largest_item= sorted(contours, key=cv2.contourArea, reverse= True)[0]
    # M = cv2.moments(largest_item)

    # cx = int(M['m10']/M['m00'])
    # cy = int(M['m01']/M['m00'])
    # col_inv = cv2.cvtColor(th,cv2.COLOR_GRAY2RGB)
    # image_copy = col_inv.copy()
    # image_copy1 = cv2.circle(image_copy, (cx, cy), 2, (0,255,0), 2)
    # cv2.drawContours(image=image_copy1, contours=contours, contourIdx=0, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    # # see the results
    # cv2.imshow('None approximation', image_copy)

    cv2.imshow("Image window", disp4)
    cv2.waitKey(3)

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