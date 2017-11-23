%% Two-wheeled robot motion model


dt = 1; % Timestep
x0 = [0 0 0.1]'; % Initial State
v = 1; % Speed
w = 0.1; % Heading rate of change

% Noise Model (x,y pos and heading)
Q = [0.001 0 0;
     0 0.001 0;
     0 0 0.05];
[QE, Qe] = eig (Q);

% Noise Model (speed and heading)
R = [0.001 0;
     0 0.05];
[RE, Re] = eig (R);

n=1000; % Samples


% disturbance free motion
x1 = dubins(x0,v,w,dt)

for i=2:n
    % Disturbance-driven motion
    D = QE*sqrt(Qe)*randn(3,1);
    E = RE*sqrt(Re)*randn(2,1);
    % Dynamics
    x_lin(:,i) = dubins(x0, v, w, dt) + D;
    x_nl(:,i) = dubins(x0+[0;0;dt*E(2)], v+E(1), w+E(2), dt); 
end

% Plot
figure(1); clf; hold on;
plot( x0(1), x0(2), 'bo', 'MarkerSize',20, 'LineWidth', 3)
plot( x1(1), x1(2), 'bo', 'MarkerSize',20, 'LineWidth', 3)
plot( [x0(1) x1(1)], [x0(2) x1(2)],'b')
plot( x_lin(1,:),x_lin(2,:), 'm.', 'MarkerSize', 3)
plot( x_nl(1,:),x_nl(2,:), 'c.', 'MarkerSize', 3)
title('Motion Model Distribution for two-wheeled robot')
xlabel('x (m)');
ylabel('y (m)');
axis equal