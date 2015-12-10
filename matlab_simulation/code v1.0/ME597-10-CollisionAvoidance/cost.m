function f = cost(x)
% Returns the total cost for a given x, used by fmincon to evaluate
% possible solutions

global Tr nv N n xd beta % Requires these global variables (does not change them)

% Initialize cost to zero, and add all the contributions together with beta
% and 1-beta weighting on the two cost elements
f = 0;
for t = 2:Tr+1 % For each timestep after the initial
    for i = 1:nv % For each vehicle

        % current vehicle state and inputs
        xcur = x(N*(t-1)+n*(i-1)+1:N*(t-1)+n*(i-1)+n); 

        % Position error cost, quadratic distance from desired position
        f = f + beta*((xd(1,t-1,i) - xcur(1))^2+ (xd(2,t-1,i) - xcur(2))^2);
 
        % Turn rate control input cost, quadratic sum of turn rate inputs
        f = f + (1-beta)*xcur(5)^2;
    end
end

    