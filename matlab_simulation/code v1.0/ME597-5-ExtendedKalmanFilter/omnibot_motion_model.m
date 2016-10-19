function mup = omnibot_motion_model(mu,w,v,dt)

    mup=[mu(1)+v*cos(mu(3))*dt; mu(2)+v*sin(mu(3))*dt; mu(3)+w*dt];
    
end