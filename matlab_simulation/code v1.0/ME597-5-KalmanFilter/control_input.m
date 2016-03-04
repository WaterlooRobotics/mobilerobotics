function control = control_input(example,mu,t,old_u)

% This function provides control at a given time step based on certain
% conditions

% Provides control for example 1
if example == 1
    if (t>1)
        control = old_u;
    end
    if (mu > 10)
        control = 0;
    elseif (mu < 2);
        control = 1;
    end
end

% Provides control for example 2
if example == 2
    control = 10*[sin(2*t);cos(t)];
end

% Provides control for example 3
if example ==3 
    control = 10*[sin(2*t);cos(t)];
end
