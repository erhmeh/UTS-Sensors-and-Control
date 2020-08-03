clc;
clear;

%Get the following parameters from your calibration
px=284.38730;%Principal point X
py=244.34837; %Principal point Y

fx=692.00043; %Focal length
fy=689.87401; 


%Homogenous transformation matrix
K = [fx,0,px;
     0,fy,py;
     0,0,1];
 
X_cam = [8;5;80;1]; %3D location
IM = eye(3,4);

x = K*IM*X_cam;

u = x(1)/x(3)
v = x(2)/x(3)


