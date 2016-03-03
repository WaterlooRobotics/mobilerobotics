function xddot = motion_model(t, T, xd)
v = sin(0.2*T);
w = ones(size(T));
xddot = [v(t)*cos(xd(3,t));
          v(t)*sin(xd(3,t));
          w(t)];

end