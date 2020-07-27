%% Question 1
clear all
close all
bag = rosbag('/home/maleen/rosbags/tutorial_RGBD.bag');

depthImages = select(bag,'Topic', '/camera/depth/image_raw');
firstDepthImage = readMessages(depthImages, 1);

RGBImages = select(bag,'Topic', '/camera/rgb/image_raw');
firstRGBImage = readMessages(RGBImages, 1);

rgb_img=readImage(firstRGBImage{1});
d_img=readImage(firstDepthImage{1});

figure
imshow(rgb_img)
figure
imshow(uint8(double(d_img)/6000*256))