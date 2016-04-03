%% This file develops a desired trajectory using Bazier Curves.
%% P is a set of points including start point, bazier control point and target.

function xdT=Trajectory_bazier(xd_start,xd_end,dt,TTot)
xd_midpoint=(xd_end(1)-xd_start(1)).*(rand(1,2));
P=[xd_start;xd_start+xd_midpoint;xd_end];

syms g
Np = size(P, 1); 

for i = 1:Np
   Bp(:,i) = nchoosek(Np,i-1).*(g.^(i-1)).*((1-g).^(Np-i+1)); %Bp is the Bernstein polynomial value
end

Bp1= (nchoosek(Np,Np).*(g.^Np)).';

Sp= simplify(Bp*P + Bp1*P(Np,:));
Bz_fun=matlabFunction(Sp);

xdT=[Bz_fun(([0:dt:(TTot)*dt]./(TTot*dt)).'),zeros(length([0:dt:(TTot*dt)]),1)];
