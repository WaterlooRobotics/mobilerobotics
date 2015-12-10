function [C, Ceq] = constraints(x)
% Returns vectors C, Ceq which define the value of the constraints. fmincon
% uses these to determine whether a solution x is feasible and which
% constraints it is currently violating
% C(x) <= 0 and Ceq(x) = 0

global nv nx n N Tr ds dt

% Collision Avoidance inequality constraints
% There are nv(nv-1)/2 pairs of vehicles and Tr time steps to evaluate (do
% not include the initial positions, as they are already fixed).
C = zeros(nv*(nv-1)/2*Tr,1);
k = 1; % Increment constraint evaluations (short cut to computing proper index)
for t = 2:Tr+1 % For each time step after initial
    for i = 1:nv % For each vehicle i
        for j = i+1:nv % For each other vehicle j greater than i
            xcuri = x(N*(t-1)+n*(i-1)+1:N*(t-1)+n*(i-1)+2); % Position i
            xcurj = x(N*(t-1)+n*(j-1)+1:N*(t-1)+n*(j-1)+2); % Position j
            C(k) = ds - norm(xcuri-xcurj); % min distance constraint
            k = k +1;
        end
    end
end



% Dynamics, nonlinear equality constraints, nv vehicles times nx states
% times Tr periods (from 0 to 1, 1 to 2,... Tr to Tr+1)
Ceq = zeros(nv*nx*Tr,1);
k = 1; % Increment constraint evaluations (short cut)
for t = 1:Tr
    for i = 1:nv
        xprev = x(N*(t-1)+n*(i-1)+1:N*(t-1)+n*i); % Previous state and inputs
        xcur = x(N*(t)+n*(i-1)+1:N*(t)+n*i); % Current state and inputs
        % Dynamic constraints on the three states
        Ceq(k:k+2) = xcur(1:3)-xprev(1:3)-dt*[cos(xprev(3))*xprev(4); sin(xprev(3))*xprev(4); xprev(5)];
        k = k+3;
    end
end

