function f = cost(x)

global N T xd pF endonly

f=0;
for i=1:T
    xcur = x(N*(i-1)+1:N*i);

    % Position error cost
    if (~endonly)
        f = f + norm(xd(i,1:2) - xcur(1:2)');
    end
    
    % Heading error cost
    %f = f + (xd(i,3)-xcur(3))^2;
    
    % Velocity control input cost
    f = f + 0.01*xcur(4)^2;
    
    % Turn rate control input cost
    f = f + 0.01*xcur(5)^2;
end

if (endonly)
    xcur = x(N*(T-1)+1:N*T);
    f = f + norm(pF(1:2) - xcur(1:2)');
end    

    