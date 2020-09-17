#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
    #self.image_sub = rospy.Subscriber("/wide_stereo/left/image_raw_throttle",Image,self.callback)



  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
      faces = face_cascade.detectMultiScale(gray, 1.1, 4)
      for (x, y, w, h) in faces:

          cv2.rectangle(gray, (x, y), (x+w, y+h), (255, 0, 0), 2)
          cv2.rectangle(gray,(250,310),(390,170),(255,0,0),5)
          milliseconds = int(round(time.time() * 1000))
          #filename = '/home/ndw/Desktop/SCMS' + str(milliseconds) + '.jpeg'
          #cv2.imwrite(filename, gray)

          font = cv2.FONT_HERSHEY_SIMPLEX

          text = "Move"
         # cv2.imshow("Face", gray )

          if x<320 :
              text = "Move Left"
          elif x > 320 :
              text = "Move Right"
          elif x == 320 and y == 240 :
              text = "Perfect!"

          cv2.putText(gray, text, (150,360), font,2,(255,255,255),2, cv2.LINE_AA)
          cv2.imshow("Face", gray )

    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
        cv2.circle(cv_image, (50,50), 10, 255)

    #cv2.imshow("Image window", cv_image)
    cv2.waitKey(60)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
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
