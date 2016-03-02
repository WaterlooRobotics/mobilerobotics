function [og]=ogmap(map, og, x, y, theta, phi, meas_r(i), rmax, mode)
% Takes a map, existing occupancy grid, and laser scan information and 
% returns an updated occupancy grid
% Input:
%   map = True map of environment
%   og = Existing occupancy grid
%   x = Robot x position
%   y = Robot y position
%   theta = Robot heading
%   [phi] = list of scan angles
%   rmax = Max range of laser
%   mode = Occupancy grid update mode
%       0 = Windowed block update mode
%       1 = Bresenham ray trace mode

% True map dimensions
[M, N] = size(map);

% Belief map
L0 = log(m./(1-m));
L=L0;

% Generate a measurement data set
meas_r = getranges(map,x(:,t),meas_phi,rmax);

%% Map update
measL = zeros(M,N);
for i = 1:length(meas_phi)
    % Get inverse measurement model
    invmod = inversescannerbres(M,N,x(1,t),x(2,t),meas_phi(i)+x(3,t),meas_r(i),rmax);
    for j = 1:length(invmod(:,1));
        ix = invmod(j,1);
        iy = invmod(j,2);
        il = invmod(j,3);
        % Calculate updated log odds
        L(ix,iy) = L(ix,iy) +log(il./(1-il))-L0(ix,iy);
        measL(ix,iy)= measL(ix,iy) +log(il./(1-il))-L0(ix,iy);
    end
end

% Calculate probabilities
og = exp(L)./(1+exp(L));
invmod_T = exp(measL)./(1+exp(measL));