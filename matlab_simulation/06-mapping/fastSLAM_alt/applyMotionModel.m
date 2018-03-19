function xNew = applyMotionModel( x, u, noiseFactor, dt )
    %% applyMotionModel( X, u, noiseFactor )
    e = noiseFactor * randn( size(x,1), 1 );
    xNew = [ x(1) + u(1) * cos( x(3) ) * dt;
             x(2) + u(1) * sin( x(3) ) * dt;
             x(3) + u(2) * dt ] + e;
end
