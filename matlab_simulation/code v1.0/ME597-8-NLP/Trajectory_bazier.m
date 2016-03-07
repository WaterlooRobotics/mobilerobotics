function xdT=Trajectory_bazier(xd_start,xd_end,dt,TTot)
xd_midpoint=(xd_end(1)-xd_start(1)).*(rand(1,2));
P=[xd_start;xd_start+xd_midpoint;xd_end];
BezierCurve(P); %generates Bz_fun
xdT=[Bz_fun(([0:dt:(TTot)*dt]./(TTot*dt)).'),zeros(length([0:dt:(TTot*dt)]),1)];
