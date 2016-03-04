function [og, lom, r_m] = ogmap(map, og, x, phi_m, rmax, mode)
% Takes a map, existing occupancy grid, and laser scan information and 
% returns an updated occupancy grid
% Input:
%   [map] = True map of environment
%   [og] = Existing occupancy grid in log odds form
%   [x] = Robot state vector [x position; y position; heading]
%   [phi_m] = Array of measurement angles relative to robot
%   rmax = Max range of laser
%   mode = Occupancy grid update mode
%       0 = Bresenham ray trace mode
%       1 = Windowed block update mode
% Output:
% 	[og] = Updated occupancy grid in log odds form
% 	[lom] = Log odds of the inverse measurement model
%   [r_m] = Array of range measurements

% True map dimensions
[M, N] = size(map);

% Initial belief map
m = 0.5*ones(M,N);
L0 = log(m./(1-m));

% Generate a measurement data set
r_m = getranges(map, x, phi_m, rmax);

% The cells affected by this specific measurement (in log odds)
lom = zeros(M,N);

% Loop through each laser measurement
for i = 1:length(phi_m)
    % Get inverse measurement model
    switch (mode)
		case 0
			invmod = inversescannerbres(M,N,x(1),x(2),phi_m(i)+x(3), r_m(i),rmax);
		case 1
			invmod = inversescanner(M,N,x(1),x(2),x(3),phi_m,r_m,rmax,alpha,beta);
		otherwise 
			invmod = inversescannerbres(M,N,x(1),x(2),phi_m(i)+x(3), r_m(i),rmax);
    end
			
	% Loop through each cell from measurement model		
    for j = 1:length(invmod(:,1));
        ix = invmod(j,1);
        iy = invmod(j,2);
        il = invmod(j,3);
        % Calculate updated log odds
        og(ix,iy) = og(ix,iy) +log(il./(1-il))-L0(ix,iy);
        lom(ix,iy)= lom(ix,iy) +log(il./(1-il))-L0(ix,iy);
    end
end