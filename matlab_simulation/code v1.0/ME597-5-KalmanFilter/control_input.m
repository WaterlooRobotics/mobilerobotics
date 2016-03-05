function u = control_input(example,mu,t,old_u,t_num)

% This function provides control at a given time step based on certain
% conditions

% Provides control for example 1
if example == 1
    if (t_num>=1)
        u = old_u;
    end
    if (mu > 10)
        u = 0;
    elseif (mu < 2);
        u = 1;
    end
end

% Provides control for example 2
if example == 2
    u = 10*[sin(2*t);cos(t)];
end

% Provides control for example 3
if example == 3 
    u = 10*[sin(2*t);cos(t)];
end
