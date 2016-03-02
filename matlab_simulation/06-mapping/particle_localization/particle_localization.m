addpath('./lib');

% Particle filter localization
clear;
clc;

%% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('particlelocalization.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

% Time
Tf = 20;
dt = 0.1;
T = 0:dt:Tf;

% Initial State
x0 = [0 0 0]';

% Control inputs
u = ones(2, length(T));
u(2, :) = 0.3 * u(2, :);

% Disturbance model
R = [0.0001 0 0;
     0 0.0001 0;
     0 0 0.001];
[RE, Re] = eig(R);

% Measurement type and noise
meas = 3; % 1 - range, 2 - bearing, 3 - both
switch(meas)
case 1
    Q = 0.01;
case 2
    Q = 0.01;
case 3
    Q = [0.01, 0; 0, 0.01];
end
[QE, Qe] = eig(Q);

% Number of particles
D = 100;
% Prior - uniform over -5 5 position, and 0 2*pi heading
X = zeros(3, D);
X(1:2,:) = 2 * rand(2, D) - 1;
X(3,:) = pi / 4 * rand(1, D) - pi / 8;
X0 = X;
Xp = X;
w = zeros(1, D);

% Feature Map
map = [5 5;  
       3 1;
       -4 5;
       -2 3;
       0 4];

% Simulation Initializations
n = length(x0);
x = zeros(n, length(T));
x(:, 1) = x0;
m = length(Q(:, 1));
y = zeros(m, length(T));
mf = zeros(2, length(T));

%plot_initial_state(1, map, x, X, D);


% % % Main loop
% for t=2:length(T)
%     % Simulation
%     % Select a motion disturbance
%     e = RE*sqrt(Re)*randn(n,1);
%     % Update state
%     x(:,t) = [x(1,t-1)+u(1,t)*cos(x(3,t-1))*dt;
%               x(2,t-1)+u(1,t)*sin(x(3,t-1))*dt;
%               x(3,t-1)+u(2,t)*dt] + e;
%     
%     % Take measurement
%     % Pick feature
%     mf(:,t) = closestfeature(map,x(:,t));
%     % Select a motion disturbance
%     d = QE*sqrt(Qe)*randn(m,1);
%     % Determine measurement
%     switch(meas) 
%         case 1
%             y(:,t) = [sqrt((mf(1,t)-x(1,t))^2 + (mf(2,t)-x(2,t))^2)] + d;
%         case 2 
%             y(:,t) = [mod(atan2(mf(2,t)-x(2,t),mf(1,t)-x(1,t))-x(3,t)+pi,2*pi)-pi] + d;
%         case 3
%             y(:,t) = [sqrt((mf(1,t)-x(1,t))^2 + (mf(2,t)-x(2,t))^2);
%                       mod(atan2(mf(2,t)-x(2,t),mf(1,t)-x(1,t))-x(3,t)+pi,2*pi)-pi] + d;
%     end
%     % Particle filter estimation
%     for dd=1:D
%         e = RE*sqrt(Re)*randn(n,1);
%         Xp(:,dd) = [X(1,dd)+u(1,t)*cos(X(3,dd))*dt;
%                     X(2,dd)+u(1,t)*sin(X(3,dd))*dt;
%                     X(3,dd)+u(2,t)*dt] + e;
%         switch(meas)
%             case 1
%                 hXp = [sqrt((mf(1,t)-Xp(1,dd))^2 + (mf(2,t)-Xp(2,dd))^2)] + d;
%             case 2
%                 hXp = [mod(atan2(mf(2,t)-Xp(2,dd),mf(1,t)-Xp(1,dd))-Xp(3,dd)+pi,2*pi)-pi] + d;
%             case 3
%                 hXp = [sqrt((mf(1,t)-Xp(1,dd))^2 + (mf(2,t)-Xp(2,dd))^2);
%                         mod(atan2(mf(2,t)-Xp(2,dd),mf(1,t)-Xp(1,dd))-Xp(3,dd)+pi,2*pi)-pi] + d;
%         end
%         w(dd) = max(1e-8,mvnpdf(y(:,t),hXp,Q));
%     end
%     W = cumsum(w);
%     for dd = 1:D
%         seed = max(W) * rand(1);
%         X(:, dd) = Xp(:, find(W > seed, 1));
%     end
%     muParticle = mean(X);
%     SParticle = var(X);
%     muParticle_S(:, dd) = muParticle;
%     
%     % Plot results
%     figure(1);
%     clf; 
%     hold on;
%     plot(map(:,1),map(:,2),'go', 'MarkerSize',10,'LineWidth',2);
%     plot(mf(1,t),mf(2,t),'mx', 'MarkerSize',10,'LineWidth',2)
%     plot(x(1,1:t),x(2,1:t), 'ro--')
%     if (meas == 1) 
%         circle(1,x(1:2,t), y(1,t)); 
%     end
%     if (meas == 2) 
%         plot([x(1,t) x(1,t)+10*cos(y(1,t)+x(3,t))], [ x(2,t) x(2,t)+10*sin(y(1,t)+x(3,t))], 'c');
%     end
%     if (meas == 3) 
%         plot([x(1,t) x(1,t)+y(1,t)*cos(y(2,t)+x(3,t))], [x(2,t) x(2,t)+y(1,t)*sin(y(2,t)+x(3,t))], 'c');
%     end
%     for dd = 1:D
%      plot(X(1, dd), X(2, dd), 'b.')
%     end
%     axis equal
%     axis([-5 7 -2 8]);
%     title('Particle Filter Localization')
%     
%     % record plot frame
%     if (makemovie) 
%         writeVideo(vidObj, getframe(gca)); 
%     end
% end
% 
% % close video object
% if (makemovie) 
%     close(vidObj); 
% end
