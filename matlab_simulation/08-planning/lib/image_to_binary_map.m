function [imout] = image_to_binary_map( input_image)
%image_to_binary converts an input image to a binary image
%   input image: any image (can be grey scale or 3 channel RGBD
%   It is assumed the image is alreay grey scale or black and white
%   Also assume that all white space, color = [0 , 0, 0] is equal to the
%   free space in the map

size_image = size(input_image);
if(size_image(3) == 3)
    %sum all entries in input image into 1 array
    imout = input_image(:,:,1) + input_image(:,:,2)+ input_image(:,:,3);    
    %set all non zero cells to white (225)
    imout(imout(:,:) > 0) = 225;
end


end

