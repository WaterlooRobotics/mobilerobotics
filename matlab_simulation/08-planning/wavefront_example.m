%test script
close all
testimage = imread('example_environments/E5_Third_Floor.png');

testimage = image_to_binary_map(testimage);
startpos = [103, 480];
endpos = [100, 100];

[wavefrontmap, path] = wavefront(testimage, endpos, startpos);

%you can just run this function without rebuilding the wavefront map if the
%goal location (or end point) does not change. 
%path = shortest_wavefront_path(wavefrontmap, startpos);
imagesc(wavefrontmap);
set(gca,'YDir','normal')
hold on
plot(path(:,1), path(:,2), '-r');
colorbar;

