function Ht = range_bearing_meas_linearized_model2(mu,i)
    %% range_bearing_meas_linearized_model(mu,i)
    % linearized measurement model of range and bearing
    % can be found P.22 of Mapping II slides
    dx = mu(3+2*(i-1)+1)-mu(1);
    dy = mu(3+2*i)-mu(2);
    rp = sqrt((dx)^2+(dy)^2);

    N=length(mu);
    Fi = zeros(5,N);
    Fi(1:3,1:3) = eye(3);
    Fi(4:5,3+2*(i-1)+1:3+2*i) = eye(2);
    % Multiplying Ht by Fi maps Ht into the correct space
    Ht = [ -dx/rp,     -dy/rp,     0,   dx/rp,   dy/rp;
            dy/rp^2,   -dx/rp^2,  -1,  -dy/rp^2, dx/rp^2 ] * Fi;
end
