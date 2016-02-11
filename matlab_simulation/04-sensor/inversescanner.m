function [m] = inversescanner(M,N,x,y,theta,meas_phi,meas_r,rmax,alpha,beta,p_occ,p_free)
% Calculates the inverse measurement model for a laser scanner based on
% model in Probabilistic Robotics - note: slow but easy to understand
% Identifies three regions, the first where no new information is
% available, the second where objects are likely to exist and the third
% where objects are unlikely to exist.  Returns an occupancy grid map of
% size MXN with p(occupied|measurements) for each cell.
% M, N - size of occupancy grid map (MXN)
% x,y,theta - position of robot in map ( assumes 1 unit grid cells)

% TODO: Modify to take in grid cell size.

% Range finder inverse measurement model
for i = 1:M
    for j = 1:N
        % Find range and bearing to the current cell
        r = sqrt((i-x)^2+(j-y)^2);
        phi = mod(atan2(j-y,i-x)-theta+pi,2*pi)-pi;
        
        % Find the applicable range measurement 
        [meas_cur,k] = min(abs(phi-meas_phi));

        % If out of range, or behind range measurement, or outside of field
        % of view, no new information is available
        if (r > min(rmax, meas_r(k)+alpha/2) || (abs(phi-meas_phi(k))>beta/2))
            m(i,j) = 0.5;

        % If the range measurement was in this cell, likely to be an object
        elseif ((meas_r(k)< rmax) && (abs(r-meas_r(k))<alpha/2))
             m(i,j) = p_occ;
        
        % If the cell is in front of the range measurement, likely to be
        % empty
        elseif (r < meas_r(k)) 
            m(i,j) = p_free;
        end
    end
end
