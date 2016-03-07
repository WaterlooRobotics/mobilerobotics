% ----------------------------------------------------------------
%
% nlashkar@uoguelph.ca
% 
% ----------------------------------------------------------------

function [y_fun] = BezierCurve(P)
syms g
Np = size(P, 1); 
% u = linspace(0, 1, N);
% B = zeros(N, Np);
for i = 1:Np
   Bp(:,i) = nchoosek(Np,i-1).*(g.^(i-1)).*((1-g).^(Np-i+1)); %B is the Bernstein polynomial value
end

Bp1= (nchoosek(Np,Np).*(g.^Np)).';

Sp= simplify(Bp*P + Bp1*P(Np,:));
y_fun=matlabFunction(Sp,'File','Bz_fun');

