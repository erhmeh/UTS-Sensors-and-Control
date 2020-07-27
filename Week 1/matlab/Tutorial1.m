imgRGB = imread('image2.jpg');
[rows, cols, channels] = size(imgRGB);
imgBW = zeros(rows,cols);

for i = 1:rows
    for j = 1:cols
        imgBW(i,j) =  round((imgRGB(i,j,1)/2 + imgRGB(i,j,2)/2 + imgRGB(i,j,3)/2)/3)
    end
end

imgBW = uint8(imgBW);
imshow(imgBW);