function [imgGS] = convertRGBtoGrayscale_student(imgRGB);

imgRGB = imread('image2.jpg');

% Get the size of the input image
[rows, cols, channels] = size(imgRGB);

% Create an empty matrix for the new greyscale image
imgGS = zeros(rows,cols);

for i = 1:rows
    for j = 1:cols
        % Your logic goes in here
         imgGS(i,j) = (imgRGB(i,j,1)*0.3 + imgRGB(i,j,2)*0.59 + imgRGB(i,j,3)*0.11);
    end
end

imgGS = uint8(imgGS);
imshow(imgGS);

end