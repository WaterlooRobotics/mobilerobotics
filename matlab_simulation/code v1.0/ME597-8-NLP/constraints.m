function [C, Ceq] = constraints(x)

global n N T dt obs withobs

C = [];
Ceq = zeros(n*(T-1),1);

% Dynamics
for i = 1:T-1
    xprev = x((N)*(i-1)+1:(N)*(i-1)+N);
    xcur = x((N)*(i)+1:(N)*(i)+N);
    Ceq(n*(i-1)+1:n*i, 1) = xcur(1:3)-xprev(1:3)-dt*[cos(xprev(3))*xprev(4); sin(xprev(3))*xprev(4); xprev(5)];
end

% Environment
if (withobs)
    nobs = length(obs(:,1));

    for i = 1:T
        for j = 1:nobs
            C(nobs*(i-1)+j, 1) = obs(j,3)^2-norm(obs(j,1:2)'-x(N*(i-1)+1:N*(i-1)+2))^2;
        end
    end
end
