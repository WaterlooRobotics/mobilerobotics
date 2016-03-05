function [imml] = inverse_scanner_window(M, N, x, win_pos, phi_m, ...
                r_m, rmax, alpha, beta, p_occ, p_free)
% Calculates the inverse measurement model for a laser scanner based on
% model in Probabilistic Robotics. The function only generates a specified 
% window of cells. Note: slow but easy to understand Identifies three 
% regions, the first where no new information is available, the second 
% where objects are likely to exist and the third where objects are 
% unlikely to exist. Returns an occupancy grid map of size M x N with 
% p(occupied|measurements) for each cell.
%
% Input:
%   M = Height of window
%   N = Width of window
%   [x] = Robot state vector [x position; y position; heading]
%   [win_pos] = position of the robot relative to the window frame
%   [phi_m] = Array of measurement angles relative to robot
%   [r_m] = Array of (r)ange (m)easurements
%   r_max = Max range of laser
%   alpha = Width of an obstacle (Distance about measurement to fill in)
%   beta = Width of a beam (Angle beyond which to exclude) 
%   p_occ = Probability of an occupied cell
%   p_free = Probability of an unoccupied cell
% Output:
% 	[imml] = Log odds of the (i)nverse (m)easurement (m)odel

% TODO: Modify to take in grid cell size.

% Initialize measurement model
imml = 0.5 * ones(M, N);

% Range finder inverse measurement model
for i = 1:M
    for j = 1:N
        % Map coordinates of window loop cell
        x_wi = x(1) - win_pos(1) + i;
        y_wj = x(2) - win_pos(2) + j;
        
        % Find range and bearing to the current cell
        r = sqrt((x_wi - x(1))^2 + (y_wj - x(2))^2);
        phi = mod(atan2(y_wj - x(2), x_wi - x(1)) - x(3) + pi, 2*pi) - pi;
        
        % Find the applicable range measurement 
        [meas_cur,k] = min(abs(phi - phi_m));

        % If out of range, or behind range measurement, or outside of field
        % of view, no new information is available
        if (r > min(rmax, r_m(k)+alpha/2) || (abs(phi-phi_m(k)) > beta/2))
            imml(i,j) = 0.5;

        % If the range measurement was in this cell, likely to be an object
        elseif ((r_m(k) < rmax) && (abs(r-r_m(k)) < alpha/2))
             imml(i,j) = p_occ;
        
        % If the cell is in front of the range measurement, likely to be
        % empty
        elseif (r < r_m(k)) 
            imml(i,j) = p_free;
        end
    end
end
