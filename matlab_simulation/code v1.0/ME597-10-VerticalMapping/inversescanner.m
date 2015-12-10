function [m] = inversescanner(M,N,x,z,theta,meas_phi,meas_r,rmax,alpha,beta)
% Calculates the inverse measurement model for a laser scanner
% Identifies three regions, the first where no new information is
% available, the second where objects are likely to exist and the third
% where objects are unlikely to exist

% Range finder inverse measurement model
for i = 1:M
    for j = 1:N
        % Find range and bearing to the current cell
        r = sqrt((i*alpha-x)^2+(j*alpha-z)^2);
        phi = mod(atan2(j*alpha-z,i*alpha-x)-theta,2*pi);
        
        % Find the applicable range measurement 
        [meas_cur,k] = min(abs(phi-meas_phi));
        phi_s(i,j) = phi;
        
        % If behind out of range measurement, or outside of field
        % of view, no new information is available
        if ((meas_r(k) == rmax) && (r - rmax >= -alpha/2)) || (abs(phi-meas_phi(k))>beta/2)
            m(i,j) = 0.5;
        % If the range measurement was before this cell, likely to be an object
        elseif ((r - meas_r(k) >= -alpha/2))
            m(i,j) = 0.7 - 0.2*(1-exp(-(r-meas_r(k))));
        % If the cell is in front of the range measurement, likely to be
        % empty
        else 
            m(i,j) = 0.3;
        end
        % Solid ground under robot
        %if (abs(phi+theta-3*pi/2) <= beta/2) && (r >= 1)
        %    m(i,j) = 0.9;
        %end
    end
end
