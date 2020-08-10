I=imread('Sydney_Harbour_Bridge_from_Circular_Quay.jpg'); %Read image

%Convert to gray scale
GSI=rgb2gray(I); 

%Define kernal
kern=[0 1 0, 1 -4 1, 0 1 0];

%Call convolution function

imgresult=convolve_with_kernal(GSI,kern);

figure(1);
imshow(I);
title('Original');
 
% figure(2);
% imshow(GSI);
% title('Gray scale');

figure(3);
imshow(imgresult);
title('Img result');