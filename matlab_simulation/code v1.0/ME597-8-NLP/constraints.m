function [C, Ceq] = constraints(x)

global n N T dt obs withobs vd_cnst

C = [];
Ceq = zeros(n*(T-1),1);

% Dynamics
for i = 1:T-1
    xprev = x((N)*(i-1)+1:(N)*(i-1)+N);
    xcur = x((N)*(i)+1:(N)*(i)+N);
    Ceq(n*(i-1)+1:n*i, 1) = xcur(1:3)-xprev(1:3)-dt*[cos(xprev(3))*xprev(4); sin(xprev(3))*xprev(4); xprev(5)];
    C = [C; x(4)-vd_cnst]; % Velocity constraint
end

% Environment
C = [];
if (withobs)
    nobs = length(obs(:,1));
    for i = 1:T
        for j = 1:nobs
            C = [C; obs(j,3)^2-norm(obs(j,1:2)'-x(N*(i-1)+1:N*(i-1)+2))^2];
        end
    end
end

% Velocity constraints
ind_end=length(C);
C(ind_end+1:ind_end+T,1)=x(4:N:end)-vd_cnst;
