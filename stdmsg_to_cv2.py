#!/usr/bin/env python
   2 from __future__ import print_function
   3 
   4 import roslib
   5 roslib.load_manifest('my_package')
   6 import sys
   7 import rospy
   8 import cv2
   9 from std_msgs.msg import String
  10 from sensor_msgs.msg import Image
  11 from cv_bridge import CvBridge, CvBridgeError
  12 
  13 class image_converter:
  14 
  15   def __init__(self):
  16     self.image_pub = rospy.Publisher("image_topic_2",Image)
  17 
  18     self.bridge = CvBridge()
  19     self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
  20 
  21   def callback(self,data):
  22     try:
  23       cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
  24     except CvBridgeError as e:
  25       print(e)
  26 
  27     (rows,cols,channels) = cv_image.shape
  28     if cols > 60 and rows > 60 :
  29       cv2.circle(cv_image, (50,50), 10, 255)
  30 
  31     cv2.imshow("Image window", cv_image)
  32     cv2.waitKey(3)
  33 
  34     try:
  35       self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
  36     except CvBridgeError as e:
  37       print(e)
  38 
  39 def main(args):
  40   ic = image_converter()
  41   rospy.init_node('image_converter', anonymous=True)
  42   try:
  43     rospy.spin()
  44   except KeyboardInterrupt:
  45     print("Shutting down")
  46   cv2.destroyAllWindows()
  47 
  48 if __name__ == '__main__':
  49     main(sys.argv)