% Robot trajectories

% Time
Tmax = 10;
dt = 0.1;
T = 0:dt:Tmax;

% Spiral
x0 = [1 1 1]';
xddot = zeros(3,length(T));
xd = zeros(3,length(T)+1);
xd(:,1)= x0;
v = exp(-0.2*T);
w = ones(size(T));
for t=1:length(T)
    xddot(:,t) = [v(t)*cos(xd(3,t));
                  v(t)*sin(xd(3,t));
                  w(t)];
    xd(:,t+1) = xd(:,t)+dt*xddot(:,t);
end

figure(1);clf; hold on;
plot(T,xd(:,1:end-1));

figure(2);clf;hold on;
plot(xd(1,:),xd(2,:));
for t=1:3:length(T)
    drawcar(xd(1,t),xd(2,t),xd(3,t),.05,2);
end
title('Desired Trajectory');
axis equal;

% Squiggle
x0 = [1 1 1]';
xddot = zeros(3,length(T));
xd = zeros(3,length(T)+1);
xd(:,1)= x0;
for t=1:length(T)
    xddot(:,t) = [2*cos(xd(3,t));
                  1*sin(xd(3,t));
                  (xd(1,t))];
    xd(:,t+1) = xd(:,t)+dt*xddot(:,t);
end

figure(3);clf; hold on;
plot(xd(1,:),xd(2,:));
for t=1:3:length(T)
    drawcar(xd(1,t),xd(2,t),xd(3,t),.2,3);
end
title('Desired Trajectory');
axis equal;

% Motions
x0 = [1 1 1]';
xddot = zeros(3,length(T));
xd = zeros(3,length(T)+1);
xd(:,1)= x0;
v = 2*ones(size(T));
w = zeros(size(T));
c = floor(length(w)/8);

figure(4); clf;
figure(5); clf;
for i = 1:10
w(2*c+1:3*c) = (-5+i)/4;
w(3*c+1:4*c) = -(-5+i)/4;

for t=1:length(T)
    xddot(:,t) = [v(t)*cos(xd(3,t));
                  v(t)*sin(xd(3,t));
                  w(t)];
    xd(:,t+1) = xd(:,t)+dt*xddot(:,t);
end

figure(4); hold on;
plot(T,xd(:,1:end-1));

figure(5);hold on;
plot(xd(1,:),xd(2,:));
% for t=1:5:length(T)
%     drawcar(xd(1,t),xd(2,t),xd(3,t),.3,5);
% end
title('Desired Trajectory');
axis equal;
end