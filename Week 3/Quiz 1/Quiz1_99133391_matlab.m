clc;
clear;

px = 513.91775;
py = 423.03256;

fx = 881.28914;
fy = 879.87153;

%Homogenous transformation matrix
K = [fx,0,px;
     0,fy,py;
     0,0,1];
 
X_cam = [18;-30;60;1]; %3D location
IM = eye(3,4);
x = K*IM*X_cam;
 
u = x(1)/x(3)
v = x(2)/x(3)
