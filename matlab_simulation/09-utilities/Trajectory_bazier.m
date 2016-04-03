function xdT=Trajectory_bazier(xd_start,xd_midpoint,xd_end,dt,TTot)
% This file generates random Bezier Curves by using 3 control points: start, middle, final ponit
% Inputs of the function are arbitrary starting point, midle point, final point in x-y plane: [x,y]
% dt is tme step, TTtot is number of iterations
% The output of this function is [x,y,0]

P=[xd_start;ones(size(xd_midpoint,1),1)*xd_start+xd_midpoint;xd_end]; % P is a set of points including start point, bezier control point and target.

syms g
Np = size(P, 1);                                                      % total number of points

for i = 1:Np                                                           
   Bp(:,i) = nchoosek(Np,i-1).*(g.^(i-1)).*((1-g).^(Np-i+1));         % Bp is the Bernstein polynomial value
end

Bp1= (nchoosek(Np,Np).*(g.^Np)).';                  

Sp= simplify(Bp*P + Bp1*P(Np,:));

Bz_fun=matlabFunction(Sp);                                            % matlabFunction is used as an alternative to subs() to improve the run time. 

xdT=[Bz_fun(([0:dt:(TTot)*dt]./(TTot*dt)).'),zeros(length([0:dt:(TTot*dt)]),1)];
