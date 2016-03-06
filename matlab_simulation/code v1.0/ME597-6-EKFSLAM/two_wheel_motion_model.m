function xnew = two_wheel_motion_model(xold,u,dt)
% motion model of a 2-wheel robot
% can be found P.11 of Mapping II slides
  if nargin == 2
        dt = 0.1;
  end
    xnew = [xold(1)+u(1)*cos(xold(3))*dt;
            xold(2)+u(1)*sin(xold(3))*dt;
            xold(3)+u(2)*dt];
end
