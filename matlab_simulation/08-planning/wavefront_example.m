%test script
close all
addpath('./lib');
testimage = imread('example_environments/E5_Third_Floor.png');
testimage = image_to_binary_map(testimage);

startpos = [103, 480];
endpos = [110, 400];

[wavefrontmap, path] = wavefront(testimage, startpos, endpos);

% you can just run this function without rebuilding the wavefront map if the
% goal location (or end point) does not change. 

path = shortest_wavefront_path(wavefrontmap, startpos);
imagesc(wavefrontmap);
hold on
plot(path(:,1), path(:,2), '-r');
% in imagesc the x axis is up down and the y runs left right, so to plot you
% need to reverse the ordering of the x and y of the start and end positions
plot(startpos(2), startpos(1), 'gx', 'markersize', 15);
plot(endpos(2), endpos(1), 'ro', 'markersize', 15);
colorbar;

