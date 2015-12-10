%% Rolling polygon approximation to walking

% Time
t = [0:0.01:3];

% Step rate
m = 2; % Hz

% Leg length
l = 1.0; % m

% Step length 
d = 1.0; % m

% Polygon sides
n = pi/asin(d/(2*l));

% Forward speed
v = d*m; % m/s

% Leg angle
a = asin((-d/2+mod(v*t,d))/(l));

% CG height
h = l*cos(a);

% Contact point
c = zeros(size(t)); 
c(1) = d/2;
for i=1:length(t)
    % New step counter
    if (i>1)
        if (a(i)-a(i-1)<0)
            c(i) = c(i-1)+d;
        else
            c(i) = c(i-1);
        end
    end
end

% Walking Figure
k = 1;
for i=1:2:length(t)
    figure(2); clf; hold on;
    % Center of motion
    cm = [v*t(i) h(i)];
    % Grounded leg
    leg = [c(i) 0; v*t(i) h(i)];
    foot = [leg(1,:); c(i)+0.2*l 0];
    % Other (swinging) leg
    oleg = [c(i)-d+mod(2*v*t(i),2*d) 0.03; v*t(i) h(i)];
    ofoot = [oleg(1,:); oleg(1,:)+[0.2*l 0]];
    % Body
    body = [cm; cm+[0 0.6*l]];
    % Head
    head = [cm+[0 0.6*l]];
    %Polygon
    s = 0:2*pi/n:2*pi;
    % Plotting
    plot(cm(1), cm(2), 'ro', 'MarkerSize', 8 , 'LineWidth', 2)
    plot(leg(:,1), leg(:,2),'b', 'LineWidth', 2);
    plot(foot(:,1), foot(:,2), 'b', 'LineWidth', 2);
    plot(oleg(:,1), oleg(:,2),'b', 'LineWidth', 2);
    plot(ofoot(:,1), ofoot(:,2), 'b', 'LineWidth', 2);
    plot(body(:,1), body(:,2),'b', 'LineWidth', 2);
    plot(head(1), head(2),'bo', 'MarkerSize', 18 , 'LineWidth', 2);
    plot(cm(1)+l*sin(s+a(i)),cm(2)+l*cos(s+a(i)),'g')
    axis equal
    axis([-1 6.5 0 2])
    F(k) = getframe; k = k+1;
end

% Save movie when ready
if(1)
   movie2avi(F,'walking.avi','fps',15)
end

% Known bug - polygon does not match motion if l not equal to d!