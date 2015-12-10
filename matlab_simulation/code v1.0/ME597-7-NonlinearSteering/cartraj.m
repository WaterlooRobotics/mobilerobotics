%% Trajectory tracking

% Fixed vehicle parameters
v = 5; % Speed
delta_max = 25*pi/180; % max steering angle
k = 2.5; % Gain
l = 1; % Car length

% Desired line state through 0,0 [theta]
xp = [0]; % Line heading

% Initial conditions in [e psi]
x0 = [5 0]; % translational offset
%x0 = [0 2]; % heading offset
%x0 = [5 2]; % both

% Simulation time
Tmax = 3;  % End point
dt =0.001; % Time step
T = 0:dt:Tmax; % Time vector

% Simulation setup
xd = zeros(length(T)-1,2); % Derivative of state ([edot psidot])
x = zeros(length(T),2);  % State ([e psi] 
x(1,:) = x0; % Initial condition
delta = zeros(length(T),1); % Steering angles
p = zeros(length(T),2); % Position in x,y plane
p(1,:) = (x0(1))*[sin(xp) cos(xp)]; % Initial position

for i=1:length(T)-1
   % Calculate steering angle
   delta(i) = max(-delta_max,min(delta_max,x(i,2) + atan2(k*x(i,1),v)));
   % State derivatives
   xd(i,1) = v*sin(x(i,2) - delta(i));
   xd(i,2) = -(v*sin(delta(i)))/(l);
   % State update
   x(i+1,1) = x(i,1)+dt*xd(i,1);
   x(i+1,2) = x(i,2)+dt*xd(i,2);
   % Position update
   p(i+1,1) = p(i,1) + dt*v*cos(x(i,2)-delta(i)-xp);
   p(i+1,2) = p(i,2) + dt*v*sin(x(i,2)-delta(i)-xp);
end

%% Plotting

% Trajectory
figure(1);clf; hold on;
plot([0 Tmax*v*cos(xp)], [0 Tmax*v*sin(xp)],'b--')
plot(x0(1)*sin(x0(2)),x0(1)*cos(x0(2)))
plot(p(:,1),p(:,2),'r');
for t=1:300:length(T)
      drawbox(p(t,1),p(t,2),x(t,2),.3,1);
end
xlabel('x (m)')
ylabel('y (m)')
axis equal

% Phase portrait
[e,psi] = meshgrid(-10:.5:10,-3:.2:3); % Create a grid over values of e and psi
delta = max(-delta_max,min(delta_max,psi+atan2(k*e,v)));  % Calculate steering angle at each point
ed = v*sin(psi-delta); % Find crosstrack derivative
psid = -(v*sin(delta))/(l); % Find heading derivative

psibplus = -atan2(k*e(1,:),v)+delta_max; % Find border of max region
psibminus = -atan2(k*e(1,:),v)-delta_max; % Find border of min region

figure(2);clf; hold on;
quiver(e,psi, ed, psid)
plot(e(1,:),psibplus,'r', 'LineWidth',2);
plot(e(1,:),psibminus,'r', 'LineWidth',2);
axis([-10 10 -3 3])
xlabel('e (m)')
ylabel('\psi (rad)')




