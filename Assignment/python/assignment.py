import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time

bridge = CvBridge()
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

def image_callback(msg):
    print("Received an image!")
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)
        for (x, y, w, h) in faces:
            cv2.rectangle(gray, (x, y), (x+w, y+h), (255, 0, 0), 2)
            milliseconds = int(round(time.time() * 1000))
            filename = '/home/lyingcake/Desktop/UTS-Sensors-and-Control/Assignment/python/imgout/facebw_' + str(milliseconds) + '.jpeg'
            cv2.imwrite(filename, gray)
    except CvBridgeError, e:
        print(e)
    # else:
    #     cv2.imwrite('images/camera_image.jpeg', cv2_img)

def main():
    rospy.init_node('image_listener')
    image_topic = "/kinect2/qhd/image_color"
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()