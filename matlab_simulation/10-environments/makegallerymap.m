function [boundary, obs, tour] = makegallerymap()
%% Gallery map 
% Constructs a simple gallery with obstacles for motion planning

boundary = [0 0; 30 0; 30 30; 0 30];
obs{1} = [3 14; 3 16; 8 16; 8 14];
obs{2} = [12 14; 12 16; 18 16; 18 14];
obs{3} = [21 14; 21 16; 27 16; 27 14];
obs{4} = [20 24; 22 24; 24 26; 26 24; 24 28];
obs{5} = [10 0; 10 10; 20 10; 20 0]; 
obs{6} = [30 20; 25 20; 25 22; 30 22]; 
obs{7} = [15 30; 15 22; 13 22; 13 30];
obs{8} = [5 20; 5 22; 20 22; 20 20];


galleryMap = [boundary; boundary(1,:); NaN NaN;];
for i=1:length(obs)
    galleryMap = [galleryMap; obs{i}; obs{i}(1,:); NaN NaN;];
end              
tour = [4 28; 5 3; 27 27; 26 5];
          
figure(1); clf; hold on;
plot(galleryMap(:,1),galleryMap(:,2));
plot(tour(:,1), tour(:,2), 'rx');
title('Gallery Tour');
axis([-1 31 -1 31])

return;
