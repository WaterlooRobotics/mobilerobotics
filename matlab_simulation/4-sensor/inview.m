function viewable = inview(f,x, rmax, thmax)
% Checks if a vector of 2D features are in view given a vector of feature 
% locations, f = [ fx1, fy1; fx2, fy2;...], a 2D robot location, x = (x,y, 
% theta), a maximum range, rmax, and a half field of view, thmax (field of 
% view is +/- thmax).  
%
% To be used to generate feature measurements with limited fov sensors.

% TODO: In view check could be vectorized

%% Input checks
if (length(f(1,:) ~= 2)) error('Feature vector incorrectly dimensioned'); end
if (length(x) ~= 3) error('Robot pose vector incorrectly dimensioned'); end
 
% Output initialization
n = length(f(:,1));
viewable = zeros(1,n);

% Relative distance and orientation
dx = f(1,:)-x(1);
dy = f(2,:)-x(2);
r = sqrt(dx.^2+dy.^2);
th = mod(atan2(dy,dx)-x(3)+pi,2*pi)-pi;

% Check if feature is in field of view 
for i=1:n
    if ((r(i)<rmax) && (abs(th(i))<thmax))
        viewable(i) = 1;
    end
end
