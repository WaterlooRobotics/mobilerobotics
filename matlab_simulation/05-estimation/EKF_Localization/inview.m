function yes = inview(f,x, rmax, thmax)
% Checks if a feature is in view
yes = 0;

dx = f(1)-x(1);
dy = f(2)-x(2);

r = sqrt(dx^2+dy^2);
th = mod(atan2(dy,dx)-x(3),2*pi);
if (th > pi)
    th = th-2*pi;
end

if ((r<rmax) && (abs(th)<thmax))
    yes = 1;
end

