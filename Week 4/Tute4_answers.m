%% Question 1

clear all
close all

bag = rosbag('tutorial4.bag');
laserscans = select(bag,'Topic', '/scan');
firstlaserscan = readMessages(laserscans, 1);

cart=firstlaserscan{1}.readCartesian;

plot(cart(:,1),cart(:,2))

%% QUESTION 2

clear all
close all

ranges=[1.4142 1.1547 1.0353 1.0000 1.0353 1.1547 1.4142];
angles= linspace(-45,45,7);

Xvals=ranges.*cos(deg2rad(angles)); %You can also use the pol2cart function. See help for pol2cart function on how to use it
Yvals=ranges.*sin(deg2rad(angles));

scatter(Xvals,Yvals)

%% Question 3

clear all
close all

scan=load('intel_LASER_.txt');

angles= linspace(-90,90,180);

scan1=scan(1,:);
scan32=scan(32,:);

[scan1x, scan1y] = pol2cart(angles,scan1);
[scan32x, scan32y] = pol2cart(angles,scan32);

scan1_cart=[scan1x',scan1y', zeros(180,1)];
scan32_cart=[scan32x', scan32y', zeros(180,1)];

scan1_ptcloud=pointCloud(scan1_cart,'Color',[ones(180,1),zeros(180,1),zeros(180,1)]);
scan32_ptcloud=pointCloud(scan32_cart,'Color',[zeros(180,1),ones(180,1),zeros(180,1)]);

figure
hold on
pcshow(scan1_ptcloud)
pcshow(scan32_ptcloud)

tform = pcregistericp(scan1_ptcloud,scan32_ptcloud);
