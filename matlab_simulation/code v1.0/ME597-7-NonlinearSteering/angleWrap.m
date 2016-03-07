%% angleWrap
% A function that receives an angle and binds it between [-pi, pi].
%
% Input:
% angle - Any arbitray angle measurement in radians
%
% Output:
% angle - The same angle except numerically bound from -pi to pi
function [angle] = angleWrap(angle)

while ( angle < -pi || angle > pi)
    if (angle < -pi)
        angle = angle + 2*pi;
    else
        angle = angle - 2*pi;
    end
end
