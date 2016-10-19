function Gt = omnibot_linearize_motion_model(mu,v,dt)
    Gt=[1 0 -v*sin(mu(3))*dt; 0 1 v*cos(mu(3))*dt; 0 0 1];
end