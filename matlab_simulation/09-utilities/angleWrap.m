%% angleWrap
% A function that receives an angle and binds it between [-pi, pi].
%
% Input:
% angle - Any arbitray angle measurement in radians
%
% Output:
% angle - The same angle except numerically bound from -pi to pi
function [wrapped_angle] = angleWrap(angle)

wrapped_angle = mod(angle + pi, 2*pi) - pi;