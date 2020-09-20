#!/usr/bin/env python
from __future__ import print_function

import rospy
# import roslib
# roslib.load_manifest('my_package')
import sys
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pcl_msgs
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as point_cloud2
import ros_numpy
import pcl
import numpy as np


face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
imageTopic = "/kinect2/qhd/image_color"
img_xMax = 960
img_yMax = 540
squareSize = 200
frameTol = 80
pointTopic = "/kinect2/qhd/points"


class face_loc:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            imageTopic, Image, self.callback)
        #self.image_sub = rospy.Subscriber("/wide_stereo/left/image_raw_throttle",Image,self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.1, 4)
            for (x, y, w, h) in faces:

                cv2.rectangle(gray, (x, y), (x+w, y+h), (255, 0, 0), 2)
                # cv2.rectangle(gray, (250, 310), (390, 170), (255, 0, 0), 5)
                cv2.rectangle(gray, (img_xMax/2 + squareSize/2, img_yMax/2 + squareSize/2),
                              (img_xMax/2 - squareSize/2, img_yMax/2 - squareSize/2), (255, 0, 0), 5)
                milliseconds = int(round(time.time() * 1000))
                filename = "/home/lyingcake/Desktop/UTS-Sensors-and-Control/Assignment/python/imgout/facebw_" + \
                    str(milliseconds) + '.jpeg'
                cv2.imwrite(filename, gray)

                font = cv2.FONT_HERSHEY_SIMPLEX
                if x < img_xMax/2 - frameTol:
                    text = "Move Left"
                elif x > img_xMax/2 + frameTol:
                    text = "Move Right"
                elif y < img_yMax/2 - frameTol:
                    text = "Move down"
                elif y > img_yMax/2 + frameTol:
                    text = "Move up"
                else:
                    rosPtc = rospy.wait_for_message(pointTopic, PointCloud2)
                    pyPtc = ros_numpy.numpify(rosPtc)
                    points = np.zeros((pyPtc.shape[0], 3))
                    points[:, 0] = pyPtc['x']
                    points[:, 1] = pyPtc['y']
                    points[:, 2] = pyPtc['z']
                    p = pcl.PointCloud(np.array(points, dtype=np.float32))
                    print(p)
                rosPtc = rospy.wait_for_message(pointTopic, PointCloud2)
                pyPtc = ros_numpy.numpify(rosPtc)
                height = pyPtc.shape[0]
                width = pyPtc.shape[1]
                np_points = np.zeros((height * width, 3), dtype=np.float32)
                np_points[:, 0] = np.resize(pyPtc['x'], height * width)
                np_points[:, 1] = np.resize(pyPtc['y'], height * width)
                np_points[:, 2] = np.resize(pyPtc['z'], height * width)
                p = pcl.PointCloud(np.array(np_points, dtype=np.float32))
                print(p)
                cv2.putText(gray, text, (150, 360), font, 2,
                            (255, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow("Face", gray)

        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (50, 50), 10, 255)

        #cv2.imshow("Image window", cv_image)
        cv2.waitKey(60)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    ic = face_loc()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
