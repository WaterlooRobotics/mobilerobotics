%% Two-wheeled robot motion model


dt = 1; % Timestep
x0 = [0 0 0.1]'; % Initial State
v = 1; % Speed
w = 0.1; % Heading rate of change

% Noise Model (speed and heading)
R = [0.001 0;
     0 0.05];
[RE, Re] = eig (R);

n=1000; % Samples
for i=1:n
    % Disturbance
    E = RE*sqrt(Re)*randn(2,1);
    % Dynamics
    x(:,i) = x0 + [ dt*(v)*cos(x0(3)) ;  dt*(v)*sin(x0(3)); E(2)+ dt*w] + diag([0.05 0.05 0.1])*randn(3,1);
    %x(:,i) = x0 + [ dt*(v+E(1))*cos(x0(3)+E(2)) ;  dt*(v+E(1))*sin(x0(3)+E(2)); E(2)+ dt*w];
end
% Disturbance free dynamics
x1 = x0 + [ dt*v*cos(x0(3)) ;  dt*v*sin(x0(3)); dt*w];

% Plot
figure(1); clf; hold on;
plot( x0(1), x0(2), 'bo', 'MarkerSize',20, 'LineWidth', 3)
plot( x1(1), x1(2), 'bo', 'MarkerSize',20, 'LineWidth', 3)
plot( [x0(1) x1(1)], [x0(2) x1(2)],'b')
plot( x(1,:),x(2,:), 'm.', 'MarkerSize', 3)
title('Motion Model Distribution for two-wheeled robot')
xlabel('x (m)');
ylabel('y (m)');
axis equal