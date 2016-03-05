function [og, imml, r_m] = ogmap(map, og, x, phi_m, r_max, alpha, beta, mode)
% Takes a map, existing occupancy grid, and laser scan information and 
% returns an updated occupancy grid
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
%       0 = Bresenham ray trace mode (default)
%       1 = Windowed block update mode
% Output:
% 	[og] = Updated (o)ccupancy (g)rid in log odds form
% 	[imml] = Log odds of the (i)nverse (m)easurement (m)odel
%   [r_m] = Array of (r)ange (m)easurements

% True map dimensions
[M, N] = size(map);

% Initial belief map
m = 0.5*ones(M,N);
L0 = log(m./(1-m));

% Generate a measurement data set
r_m = getranges(map, x, phi_m, r_max);

% The cells affected by this specific measurement in log odds (only used
% in Bresenham ray trace mode)
imml = zeros(M,N);

if (mode == 1)
    % -Windowed block update mode (could be optimized further based on FOV
    % and angle of sensor)
    
    % Bound the update window within the map dimensions
    w_Mi = max(1,min(M,round(x(1) - r_max)));
    w_Mf = max(1,min(M,round(x(1) + r_max)));
    w_Ni = max(1,min(N,round(x(2) - r_max)));  
    w_Nf = max(1,min(N,round(x(2) + r_max)));  
    w_M = w_Mf - w_Mi + 1;
    w_N = w_Nf - w_Ni + 1;
    win_pos(1) = x(1) - w_Mi + 1;
    win_pos(2) = x(2) - w_Ni + 1;
    
    % Get inverse measurement model
    invmod = inverse_scanner_window(w_M, w_N, x, win_pos, phi_m, r_m, ...
        r_max, alpha, beta, 0.7, 0.3);
    
    % Calculate updated log odds
    og(w_Mi:w_Mf, w_Ni:w_Nf) = og(w_Mi:w_Mf, w_Ni:w_Nf) + ...
        log(invmod./(1-invmod))-L0(w_Mi:w_Mf, w_Ni:w_Nf);
    imml(w_Mi:w_Mf, w_Ni:w_Nf) = imml(w_Mi:w_Mf, w_Ni:w_Nf) + ...
        log(invmod./(1-invmod))-L0(w_Mi:w_Mf, w_Ni:w_Nf);
else
    % -Bresenham ray trace mode
    
    % Loop through each laser measurement
    for i = 1:length(phi_m)
        % Get inverse measurement model
        invmod = inversescannerbres(M, N, x(1), x(2), phi_m(i)+x(3), ...
            r_m(i), r_max);

        % Loop through each cell from measurement model		
        for j = 1:length(invmod(:, 1));
            ix = invmod(j, 1);
            iy = invmod(j, 2);
            il = invmod(j, 3);
            
            % Calculate updated log odds
            og(ix, iy) = og(ix, iy) + log(il./(1-il)) - L0(ix,iy);
            imml(ix, iy)= imml(ix, iy) + log(il./(1-il)) - L0(ix,iy);
        end
    end
end