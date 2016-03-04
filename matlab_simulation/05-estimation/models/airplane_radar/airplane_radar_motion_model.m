function M = airplane_radar_motion_model(mu, ~, dt)
    if ~exist('dt', 'var')
        dt = 0.1;
    end
    Ad = [ 1 dt 0 ; 0 1 0; 0 0 1];
    M = Ad*mu;
end