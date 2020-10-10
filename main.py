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
import numpy as np

# For more info regarding Harr Cascades refer to https://docs.opencv.org/3.4/db/d28/tutorial_cascade_classifier.html
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

camera_hardware = 1

if (camera_hardware == 0):
    # 0 for Jamin (Kinect v2)
    imageTopic = "/kinect2/hd/image_color"
    depthTopic = "/kinect2/sd/image_depth_rect"
    imgDir = "/home/lyingcake/Desktop/UTS-Sensors-and-Control/Assignment/python/imgout/facebw_"
elif (camera_hardware == 1):
    # 1 for Nick (realsense)
    imageTopic = "/camera/color/image_raw"
    depthTopic = "/camera/depth/image_rect_raw"
    imgDir = "/home/ndw/SCMS/Assignment/images/"

squareSize = 200
frameTol = 30


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
            # Detect image size
            img_yMax, img_xMax, c = cv_image.shape
            #print("width: ", img_xMax)
            #print("height: ", img_yMax)
            # Harr Cascade face detection only works for BW images, so we need to convert it
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # return a list of 'faces'. Each face has an (x,y) coordinate within the image frame, as well as a (w,h) for the size of the face
            faces = face_cascade.detectMultiScale(gray, 1.1, 4)
            for (x, y, w, h) in faces:
                # plot a rectangle around the face
                cv2.rectangle(gray, (x, y), (x+w, y+h), (255, 0, 0), 2)
                font = cv2.FONT_HERSHEY_SIMPLEX
                centre_x = x+w//2
                centre_y = y+h//2
                # provide feedback depending on where the face is located within the frame
                print("X Error: ", (img_xMax/2 - centre_x))
                print("Y Error: ", (img_yMax/2 - centre_y))
                # Print x, y coordinates of top left corner of face rectangle
                print("Xmax: ", img_xMax/2)
                print("Ymax: ", img_yMax/2)
                # Prints coordinates of the face centre
                print("X2: ", centre_x)
                print("Y2: ", centre_y)
                depthImgFace = rospy.wait_for_message(depthTopic, Image)
                face_cv_depth_image = self.bridge.imgmsg_to_cv2(
                    depthImgFace, "32FC1")
                depth_array = face_cv_depth_image[centre_y, centre_x]
                print("Centre of face Depth: ", depth_array)

                # find the distance to the face wihthin the frame
                if centre_x < img_xMax/2 - frameTol:
                    text = "Move Left"
                elif centre_x > img_xMax/2 + frameTol:
                    text = "Move Right"
                elif centre_y < img_yMax/2 - frameTol:
                    text = "Move down"
                elif centre_y > img_yMax/2 + frameTol:
                    text = "Move up"
                else:  # if the face is within the center of the frame, plus some tolerance, we know if we fetch the center of the depth image, we can get the distance to the face.
                    depthImg = rospy.wait_for_message(depthTopic, Image)
                    middle_x_depth = depthImg.width//2
                    middle_y_depth = depthImg.height//2
                    cv_depth_image = self.bridge.imgmsg_to_cv2(
                        depthImg, "32FC1")
                    # find the distance to the face within the frame
                    distToFace = cv_depth_image[middle_y_depth, middle_x_depth]
                    if distToFace < 450 and distToFace > 400:
                        text = "Perfect. Say cheese!"
                        milliseconds = int(round(time.time() * 1000))
                        filename = imgDir + \
                            str(milliseconds) + '.jpeg'
                        cv2.imwrite(filename, gray)  # save the image
                        time.sleep(2)
                        print("Photo taken, closing script!")
                        rospy.signal_shutdown("action complete")
                    else:
                        text = "Move closer"
                cv2.putText(gray, text, (150, 360), font, 2,
                            (255, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow("Face", gray)
        except CvBridgeError as e:
            print(e)
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
