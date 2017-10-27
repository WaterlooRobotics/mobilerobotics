function [og, imml] = ogmap_update(og, x, phi_m, r_m, r_max, alpha, beta, p_occ, p_free, mode)
% Takes a map, existing occupancy grid, and laser scan information then 
% returns an updated occupancy grid.
%
% Input:
%   [og] = Existing (o)ccupancy (g)rid in log odds form
%   [x] = Robot state vector [x position; y position; robot heading; sensor heading]
%   [phi_m] = Array of measurement angles relative to robot
%   [r_m] = Array of corresponding range measurements
%   r_max = Maximum measurement range for sensor
%   alpha = Width of an obstacle (Distance about measurement to fill in)
%   beta = Width of a beam (Angle beyond which to exclude) 
%   mode = Occupancy grid update mode
%       0 = Bresenham ray trace mode (default)
%       1 = Windowed block update mode
% Output:
% 	[og] = Updated (o)ccupancy (g)rid in log odds form
% 	[imml] = Log odds of the (i)nverse (m)easurement (m)odel

% True map dimensions
[M, N] = size(og);

% Default logodds probabilities
p0 = 0.5;
L0 = log(p0/(1-p0))*ones(M,N);

% The cells affected by this specific measurement in log odds (only used
% in Bresenham ray trace mode)
imml = L0;

if ((mode == 1) || (mode == 2))
    % -Windowed block update (could be optimized further based on FOV
    % and angle of sensor)
       
    % Get inverse measurement model
    invmod = inverse_scanner_window(M, N, x, phi_m, r_m, r_max, alpha, ...
        beta, p_occ, p_free);
    
    % Calculate updated log odds
    og = og + log(invmod./(1-invmod)) - L0;
    imml = imml + log(invmod./(1-invmod)) - L0;	

else
    % --- Bresenham ray trace mode
    % Loop through each laser measurement
    for i = 1:length(phi_m)
        % Get inverse measurement model
        invmod = inverse_scanner_bres(M, N, x(1), x(2), phi_m(i)+x(4), ...
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