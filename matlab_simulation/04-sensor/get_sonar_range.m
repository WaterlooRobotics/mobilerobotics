function r_m = get_sonar_range(map, x, fov, r_max)
% Finds the closest occupied cell within the fov and max range and returns 
% the inverse measurement model for the sonar sensor. Uses windowing to
% reduce checked cells.
%
% Input:
%   [map] = True map of environment
%   [x] = Robot state vector [x position; y position; heading]
%   fov = Sonar field of view
%   r_max = Max range of sonar
% Output:
%   r_m = Range of object

% Bound the update window within the map dimensions
[M, N] = size(map);
w_Mi = max(1,min(M,round(x(1) - r_max)));
w_Mf = max(1,min(M,round(x(1) + r_max)));
w_Ni = max(1,min(N,round(x(2) - r_max)));  
w_Nf = max(1,min(N,round(x(2) + r_max)));  
w_M = w_Mf - w_Mi + 1;
w_N = w_Nf - w_Ni + 1;
win_pos(1) = x(1) - w_Mi + 1;
win_pos(2) = x(2) - w_Ni + 1;

% FOV limits
phi_min = -fov/2;
phi_max = fov/2; 

r_m = r_max;

% Range finder inverse measurement model
for i = 1:w_M
    for j = 1:w_N
        % Map coordinates of window loop cell
        x_wi = x(1) - win_pos(1) + i;
        y_wj = x(2) - win_pos(2) + j;
        
        % Check if cell is occupied 
        if (map(x_wi, y_wj) == 0)
            % Check if edge map (edge of map is considered occupied)
            if ~(x_wi == 1 || x_wi == M || y_wj == 1 || y_wj == N)
                continue
            end
        end
        
        % Find range and bearing to the current cell
        r = sqrt((x_wi - x(1))^2 + (y_wj - x(2))^2);
        phi = mod(atan2(y_wj - x(2), x_wi - x(1)) - x(3) + pi, 2*pi) - pi;
        
        % If within fov and range
        if ((phi > phi_min && phi < phi_max) && (r < r_max))
            % If current feature is closest
            if (r < r_m)
                r_m = r;
            end
        end
    end
end