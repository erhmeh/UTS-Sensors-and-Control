# Sensors and Control for Mechatronic Systems 41014
## Group 8 project: Visual Servoing using RGB-D
### Jamin Early (99133391), Nick Welsh (99132021), Tanya Nassir (98062033)
  
### System overview, setup and dependencies

**Tested on:**  
Ubuntu 18.04    
ROS Melodic   
Python 3.6.10   
OpenCV 4.4.0   
Matlab R2020a   
Peter Corke's RVC Toolbox (modified) - this has been included in the git *TODO    
  
**Files included:**  
1 x main.py **TODO if happy with edits, change the file Nick upoloaded main2.py to main.py**  
1 x haarcascade_frontalface_default.xml  
1 x Visual_servoing_SCMS.m  
1 x rosbag_record_kinetic.bag  
1 x rosbag_record_realsense.bag  
  
**To run:**  
python main.py  
  
To run python3 with ROS, a virtual environment (using anaconda) can be used. For instructions: https://medium.com/@zuxinl/ubuntu-18-04-ros-python3-anaconda-cuda-environment-configuration-cb8c8e42c68d      
  
**Program overview (main.py):**  
Our application aims to perform the task of aligning a face in the centre of the image plane at a specified depth.
This project has real world applications and requires addressing the perception problems image detection and feature extraction.

To detect a target image (a face), our application utilises the Haar Cascade classifier through openCV to determine if a face is located in the image plane. This allows the location of the face to be recorded and a rectangle to be annotated to the output window, track the target face across the screen.
#For more info regarding Harr Cascades refer to https://docs.opencv.org/3.4/db/d28/tutorial_cascade_classifier.html

Our application has been developed to work with multiple cameras, subscribing to different rostopics for rgb and depth. We have included a simple configuration variable to allow for quick changes. 

2 rosbag files included to test application on different hardware
Depending on which hardware is tested line #24 of main.py must be changed to reflect the rostopics that are subscribed to.
0 = Kinetic
1 = Realsense

variable 'frameTol' sets the tolerance of the allow distance of the face to the centre of the frame (P-controller).

ROS callback function provides the main functionality of the program.
 - Convert the ROS image message to an OpenCV image
 - Detect image size. Useful for using different hardware
 - Convert images to grayscale
 - If a face is detected using the Haar Cascade classifier, the top left hand corner is recorded (x,y) and a rectangle is annotated 
   to the screen.
 - A calculation to determine the centre x,y is used to determine the error from centre and depth of the centre of the face.
 - The depth values obtained through ROS are converted to cv float value that can be used for feedback.
 - Using the values obtained, the controller provides feedback to the user on how to move and providing visual servoing
   through the difference in the observed position and desired position.
 - Once the face is centred, the program provides feedback on the distance of the target to the camera. When the target is in the        correct location (x,y,z) a photo is taken and stored on the system. (This directory will need to changed depending on local system.    variable 'imgDir').
 - The bgr image is published through ros if required

The main function runs.
 

**Matlab Analysis (Visual_servoing_SCMS.m):**
One of the key aims of visual servoing is to determine the velocity of camera required to track features accurately. 
To perform analysis of how this project could be integrated with a 6-DOF robot (eye-in-hand) we chose Matlab to calcuate the interaction matrix. Using the camera's intrinsic parameters and the values computed in main.py, this script can be used to simulate the velocity required for a UR10 to track the target points.  
  
This code was modified from UTS Robotics 41013 week 8 lab and uses a modified version of Peter Corke's RVC Toolbox.  
  
The target image plane points are set to be centered and match the size in the program. A tolerance of 30 (left and right of centre).
  
The 3D points were obtained from main.py  
A UR10 is intialised with an inital pose.  
The realsense intrinsic parameters are used to set camera.  
A lambda is set. This is the gain of the control system.  
The system simulation is created.  
The main function is in section 1.4   
 - A loop runs while the camera/end effector move to the goal position  
 - At each step the feature error is calculated  
 - The feature Jacobian established the realtionship between camera fetaures and the velocity. Depth is estimated in this simulation   
 - The velocity is calculated as the combination of gain (lambda), psuedo Moore-Pentose and error
 - The inverse of the robot's Jacobian is used to update joint velocities






