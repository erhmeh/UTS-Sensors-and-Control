Sensors and Control for Mechatronic Systems 41014
Group 8 project: Visual Servoing using RGB-D
Jamin Early (99133391), Nick Welsh (99132021), Tanya Nassir (98062033)
  
System overview, setup and dependencies

Tested on:
Ubuntu 18.04
ROS Melodic
Python 3.6.10
OpenCV 4.4.0
Matlab R2020a
Peter Corke's RVC Toolbox (modified) - this has been included in the git *TODO

Files included:
1 x main.py *TODO if happy with edits, change the file Nick upoloaded main2.py to main.py
1 x haarcascade_frontalface_default.xml
1 x Visual_servoing_SCMS.m
1 x rosbag_record_kinetic.bag
1 x rosbag_record_realsense.bag

2 rosbag files included to test demonstration to match different hardware
Depending on which hardware is tested line #24 of main.py must be changed to reflect the rostopics that are subscribed to.
0 = Kinetic
1 = Realsense

Program overview (main.py):
Our application aims to perform the task of aligning a face in the centre of the image plane at a specified depth.
This project has real world applications and requires addressing the perception problems image detection and feature extraction.

To detect a target image (a face), our application utilises the Haar Cascade classifier through openCV to determine if a face is located in the image plane. This allows the location of the face to be recorded and a rectangle to be annotated to the output window, track the target face across the screen.
#For more info regarding Harr Cascades refer to https://docs.opencv.org/3.4/db/d28/tutorial_cascade_classifier.html

Our application has been developed to work with multiple cameras, subscribing to different rostopics for rgb and depth. We have included a simple configuration variable to allow for quick changes. Refer to note on rosbags above

Analysis:
One of the key aims of visual servoing is to determine the velocity of camera required to track features accurately. 





