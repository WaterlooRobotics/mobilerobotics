function xddot = motion_model(t, T, xd)
% User can define any motion model to  
% generate any motions in this file.
% Choose "5" in the main program to test the result.

v = sin(0.2*T);
w = ones(size(T));

xddot = [v(t)*cos(xd(3,t));
          v(t)*sin(xd(3,t));
          w(t)];

end
