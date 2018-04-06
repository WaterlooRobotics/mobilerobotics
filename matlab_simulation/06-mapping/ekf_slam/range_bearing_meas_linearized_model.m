function Ht = range_bearing_meas_linearized_model(mu,i)
% linearized measurement model of range and bearing
% can be found P.22 of Mapping II slides
            dx = mu(3+2*(i-1)+1)-mu(1);
            dy = mu(3+2*i)-mu(2);
            rp = sqrt((dx)^2+(dy)^2);

            N=length(mu);
            Ht = zeros(2,N);
            Ht(1:2,1:3) = [ -dx/rp,     -dy/rp,     0;
                             dy/rp^2,   -dx/rp^2,  -1 ];
            Ht(1:2, 3+2*(i-1)+1:3+2*i) = [  dx/rp,   dy/rp;
                                           -dy/rp^2, dx/rp^2 ];
end
