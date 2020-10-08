############################################################
#       Sensors and Control for Mechatronic Systems
#       Visual Servoing using RGB-D
#       @file: main.py
#       @authors: Jamin Early, Nick Welsh, Tanya Nassir
############################################################

#!/usr/bin/env python
from __future__ import print_function

import rospy
import sys
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2

# For more info regarding Harr Cascades refer to https://docs.opencv.org/3.4/db/d28/tutorial_cascade_classifier.html
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# Uncomment for Jamin (Kinect v2)
imageTopic = "/kinect2/hd/image_color"
img_xMax = 1280
img_yMax = 800
squareSize = 200
frameTol = 80
depthTopic = "/kinect2/sd/image_depth_rect"
imgDir = "/home/lyingcake/Desktop/UTS-Sensors-and-Control/Assignment/python/imgout/facebw_"

# Uncomment for Nick (realsense)
# imageTopic = "/camera/color/image_raw"
# img_xMax = 640
# img_yMax = 480
# squareSize = 200
# frameTol = 80
# depthTopic = "/camera/depth/image_rect_raw"
# imgDir = "/home/ndw/SCMS/Assignment/images")


# Face location and feedback class
class face_loc:
    # Class initialisation
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            imageTopic, Image, self.callback)  # Image subscriber; calls the callback function every time a new image is published to the broker
    # Callback function

    def callback(self, data):
        try:
            # convert the ROS image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Harr Cascade face detection only works for BW images, so we need to convert it
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # return a list of 'faces'. Each face has an (x,y) coordinate within the image frame, as well as a (w,h) for the size of the face
            faces = face_cascade.detectMultiScale(gray, 1.1, 4)
            for (x, y, w, h) in faces:
                # plot a rectangle around the face
                cv2.rectangle(gray, (x, y), (x+w, y+h), (255, 0, 0), 2)
                font = cv2.FONT_HERSHEY_SIMPLEX
                # provide feedback depending on where the face is located within the frame.
                print("X Error: " + (img_xMax/2 - x))
                print("Y Error: " + (img_yMax/2 - y))
                if x < img_xMax/2 - frameTol:
                    text = "Move Left"
                elif x > img_xMax/2 + frameTol:
                    text = "Move Right"
                elif y < img_yMax/2 - frameTol:
                    text = "Move down"
                elif y > img_yMax/2 + frameTol:
                    text = "Move up"
                else:  # if the face is within the center of the frame, plus some tolerance, we know if we fetch the center of the depth image, we can get the distance to the face.
                    depthImg = rospy.wait_for_message(depthTopic, Image)
                    middle_x_depth = depthImg.width/2
                    middle_y_depth = depthImg.height/2
                    cv_depth_image = self.bridge.imgmsg_to_cv2(
                        depthImg, "32FC1")
                    # find the distance to the face within the frame
                    distToFace = cv_depth_image[middle_y_depth, middle_x_depth]
                    if distToFace < 650 and distToFace > 400:
                        text = "Perfect. Say cheese!"
                        milliseconds = int(round(time.time() * 1000))
                        filename = imgDir + \
                            str(milliseconds) + '.jpeg'
                        cv2.imwrite(filename, gray)  # save the image
                        print("Photo taken, closing script!")
                        rospy.signal_shutdown("action complete")
                    else:
                        text = "Move closer"
                cv2.putText(gray, text, (150, 360), font, 2,
                            (255, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow("Face", gray)
        except CvBridgeError as e:
            print(e)
        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (50, 50), 10, 255)
        cv2.waitKey(60)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(
                cv_image, "bgr8"))  # publish the image to ros
        except CvBridgeError as e:
            print(e)


def main(args):
    print("Starting...")
    ic = face_loc()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)