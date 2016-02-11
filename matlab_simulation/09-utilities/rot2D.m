function R = rot2D(ang)
% Standard 2D rotation using 3D function
R = rot(ang,3);
R = R(1:2,1:2);

