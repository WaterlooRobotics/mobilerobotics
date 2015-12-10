% Gallery map

boundary = [0 0; 10 0; 10 10; 20 10; 20 0; 30 0; 30 20; 25 20; 25 22; 30 22; 30 30; 15 30; 15 22; 20 22; 20 20; 5 20; 5 22; 13 22;13 30; 0 30];
obstacle1 = [3 14; 3 16; 8 16; 8 14];
obstacle2 = [12 14; 12 16; 18 16; 18 14];
obstacle3 = [21 14; 21 16; 27 16; 27 14];
obstacle4 = [20 24; 22 24; 24 26; 26 24; 24 28];

galleryMap = [boundary; boundary(1,:); NaN NaN; 
              obstacle1; obstacle1(1,:); NaN NaN;
              obstacle2; obstacle2(1,:); NaN NaN;
              obstacle3; obstacle3(1,:); NaN NaN;
              obstacle4; obstacle4(1,:); NaN NaN];
              
tour = [4 28; 5 3; 27 27; 26 5];
          
figure(1); clf; hold on;
plot(galleryMap(:,1),galleryMap(:,2));
plot(tour(:,1), tour(:,2), 'rx');
title('Gallery Tour');
axis([-1 31 -1 31])

