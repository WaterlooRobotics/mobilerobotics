function [og, imml, r_m] = ogmap(map, og, x, phi_m, r_max, alpha, beta, mode)
% Takes a map, existing occupancy grid, and laser scan information then 
% returns an updated occupancy grid.
%
% Input:
%   [map] = True map of environment
%   [og] = Existing (o)ccupancy (g)rid in log odds form
%   [x] = Robot state vector [x position; y position; heading]
%   [phi_m] = Array of measurement angles relative to robot
%   r_max = Max range of laser
%   alpha = Width of an obstacle (Distance about measurement to fill in)
%   beta = Width of a beam (Angle beyond which to exclude) 
%   mode = Occupancy grid update mode
%       0 = Laser scanner using Bresenham ray trace mode (default)
%       1 = Laser scanner using windowed block update mode
%       2 = Sonar using windowed block update mode
% Output:
% 	[og] = Updated (o)ccupancy (g)rid in log odds form
% 	[imml] = Log odds of the (i)nverse (m)easurement (m)odel
%   [r_m] = Array of (r)ange (m)easurements

% True map dimensions
[M, N] = size(map);

% Initial belief map
m = 0.5*ones(M,N);
L0 = log(m./(1-m));

% Probabilites of cells
p_occ = 0.7;
p_free = 0.3;

% The cells affected by this specific measurement in log odds (only used
% in Bresenham ray trace mode)
imml = zeros(M,N);

if (mode == 1)
    % -Windowed block update mode (could be optimized further based on FOV
    % and angle of sensor)
    
    % Generate a measurement data set
    r_m = getranges(map, x, phi_m, r_max);
    
    % Get inverse measurement model
    invmod = inverse_scanner_window(M, N, x, phi_m, r_m, r_max, alpha, ...
        beta, p_occ, p_free);
    
    % Calculate updated log odds
    og = og + log(invmod./(1-invmod)) - L0;
    imml = imml + log(invmod./(1-invmod)) - L0;	
elseif (mode == 2)
    % --- Windowed block update mode with sonar
    % Generate a measurement data set
    r_m = get_sonar_range(map, x, beta, r_max);

    % Get inverse measurement model
    invmod = inverse_scanner_window(M, N, x, phi_m, r_m, r_max, alpha, ...
        beta, p_occ, p_free);
    
    % Calculate updated log odds
    og = og + log(invmod./(1-invmod)) - L0;
    imml = imml + log(invmod./(1-invmod)) - L0;	
else
    % --- Bresenham ray trace mode
    % Generate a measurement data set
    r_m = getranges(map, x, phi_m, r_max);
    
    % Loop through each laser measurement
    for i = 1:length(phi_m)
        % Get inverse measurement model
        invmod = inverse_scanner_bres(M, N, x(1), x(2), phi_m(i)+x(3), ...
            r_m(i), r_max, p_occ, p_free);

        % Loop through each cell from measurement model		
        for j = 1:length(invmod(:, 1));
            ix = invmod(j, 1);
            iy = invmod(j, 2);
            il = invmod(j, 3);
            
            % Calculate updated log odds
            og(ix, iy) = og(ix, iy) + log(il./(1-il)) - L0(ix, iy);
            imml(ix, iy)= imml(ix, iy) + log(il./(1-il)) - L0(ix, iy);
        end
    end
end