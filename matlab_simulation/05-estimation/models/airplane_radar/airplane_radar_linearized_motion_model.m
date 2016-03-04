function M = airplane_radar_linearized_motion_model(~, dt)
    if nargin == 1
        dt = 0.1;
    end
    Ad = [ 1 dt 0 ; 0 1 0; 0 0 1];
    M = Ad;
end