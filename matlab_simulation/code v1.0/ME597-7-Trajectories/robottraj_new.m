% Robot trajectories
clear;
close;
clc;

% Time
Tmax = 10;
dt = 0.1;
T = 0:dt:Tmax;

disp('What is the input trajectory?')
disp('1: Swerves; 2: Nudge; 3: Lane change; 4: Corner;')
disp('5: User defined motion model')
prompt = '';
index = input(prompt);

disp('Please give an initial condition, i.e. [1 1 1]')
prompt = '';
x0 = input(prompt)';

xddot = zeros(3,length(T));
xd = zeros(3,length(T)+1);
xd(:,1)= x0;


if index == 1
    % Swerves
    v = sin(0.2*T);
    w = ones(size(T));
    for t=1:length(T)
        xddot(:,t) = [v(t)*cos(xd(3,t));
                      v(t)*sin(xd(3,t));
                      w(t)];
        xd(:,t+1) = xd(:,t)+dt*xddot(:,t);
    end

    figure(1);hold on;
    plot(xd(1,:),xd(2,:));
    for t=1:3:length(T)
        drawcar(xd(1,t),xd(2,t),xd(3,t),.05,1);
    end
    title('Desired Trajectory: Swerves');
    axis equal;
    
elseif index == 2
    % Nudges
    v = 2*ones(size(T));
    w = zeros(size(T));
    c = floor(length(w)/8);
    w(2*c+1:3*c) = 1;
    w(3*c+1:4*c) = -1;
    w(4*c+1:5*c) = -1;
    w(5*c+1:6*c) = 1;

    for t=1:length(T)
        xddot(:,t) = [v(t)*cos(xd(3,t));
                      v(t)*sin(xd(3,t));
                      w(t)];
        xd(:,t+1) = xd(:,t)+dt*xddot(:,t);
    end
    
    figure(2);hold on;
    plot(xd(1,:),xd(2,:));
    for t=1:5:length(T)
        drawcar(xd(1,t),xd(2,t),xd(3,t),.3,2);
    end
    title('Desired Trajectory: Nudges');
    axis equal;

elseif index == 3
    % Lane change
    v = 2*ones(size(T));
    w = zeros(size(T));
    c = floor(length(w)/8);
    w(2*c+1:3*c) = 1;
    w(3*c+1:4*c) = -1;

    for t=1:length(T)
        xddot(:,t) = [v(t)*cos(xd(3,t));
                      v(t)*sin(xd(3,t));
                      w(t)];
        xd(:,t+1) = xd(:,t)+dt*xddot(:,t);
    end

    figure(3);hold on;
    plot(xd(1,:),xd(2,:));
    for t=1:5:length(T)
        drawcar(xd(1,t),xd(2,t),xd(3,t),.3,3);
    end
    title('Desired Trajectory: Lane change');
    axis equal;
    
elseif index == 4
    % Corner
    v = 2*ones(size(T));
    w = zeros(size(T));
    c = floor(length(w)/8);
    w(2*c+1:5*c) = -2/5;

    for t=1:length(T)
        xddot(:,t) = [v(t)*cos(xd(3,t));
                      v(t)*sin(xd(3,t));
                      w(t)];
        xd(:,t+1) = xd(:,t)+dt*xddot(:,t);
    end
    
    figure(4);hold on;
    plot(xd(1,:),xd(2,:));
    for t=1:5:length(T)
        drawcar(xd(1,t),xd(2,t),xd(3,t),.3,4);
    end
    title('Desired Trajectory: Corner');
    axis equal;
    
elseif index == 5
    % User defined motion model
    for t=1:length(T)
        xddot(:,t) = motion_model(t, T, xd);
        xd(:,t+1) = xd(:,t)+dt*xddot(:,t);
    end
    
    figure(5);hold on;
    plot(xd(1,:),xd(2,:));
    for t=1:5:length(T)
        drawcar(xd(1,t),xd(2,t),xd(3,t),.05,5);
    end
    title('Desired Trajectory: Corner');
    axis equal;
    
else
    disp('Please choose a proper input trajectory (1-4).')
end