
function [imgresult] = convolve_with_kernal(input_img, kernal)


imgresult = conv2(input_img,kernal,'same');

imgresult=uint8(imgresult);

imshow(imgresult)