% Particle filter localization
clear;clc;

%% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('particlelocalization_lownoise_bothmeas_100_right disturbance.avi');
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
u(2,:)=0.3*u(2,:);

% Disturbance model
R = [1e-4 0 0; 
     0 1e-4 0; 
     0 0 1e-6];
[RE, Re] = eig(R);

% Measurement type and noise
meas = 3; % 1 - range, 2 - bearing, 3 - both

switch(meas)
    case 1
        Q = 0.02;
    case 2
        Q = 0.002;
    case 3
         Q = [0.002 0; 
              0 0.002];
end
[QE, Qe] = eig(Q);
% Sensor footprint
rmax = 10 ; % meters
thmax = pi/4; % rads

% Number of particles
D = 100;
% Prior - uniform over -1 1 position, and -pi/4 pi/4 heading
X = zeros(3,D);
X(1:2,:) = 4*rand(2,D) - 1;
X(3,:) = pi*rand(1,D)-pi/2;
X0 = X;
Xp = X;
w = zeros(1,D);

% Feature Map
map = [ 5 5;  
         3  1;
         -4  5;
         -2  3;
         0  4];
M = length(map(:,1));

% Simulation Initializations
n = length(x0);
x = zeros(n,length(T));
x(:,1) = x0;
m = length(Q(:,1));
y = zeros(2*M,length(T));
mf = zeros(2*M,2);
muParticle_S = zeros(n,length(T));

figure(1);clf; hold on;
plot(map(:,1),map(:,2),'go', 'MarkerSize',10,'LineWidth',2);
plot(x(1,1),x(2,1), 'ro--')
for dd=1:D
    plot(X(1,dd),X(2,dd),'b.')
end
axis equal
axis([-4 6 -1 7]);
title('Particle Filter Localization')
F(1) = getframe;


%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x(:,t) = [x(1,t-1)+u(1,t)*cos(x(3,t-1))*dt;
              x(2,t-1)+u(1,t)*sin(x(3,t-1))*dt;
              x(3,t-1)+u(2,t)*dt] + e;
    
    % Take measurement
    % Identify features that are in view
    nj = 0;
    for i=1:M
        % If feature is visible
        if (inview(map(i,:),x(:,t),rmax,thmax))
            nj = nj+1;
            mf(nj,:) = map(i,:);
            % Select a motion disturbance
            d = QE*sqrt(Qe)*randn(m,1);
            % Determine measurement
            switch(meas)
                case 1
                    y(nj,t) = max(0.001,sqrt((mf(nj,1)-x(1,t))^2 + (mf(nj,2)-x(2,t))^2) + d);
                case 2
                    y(nj,t) = [atan2(mf(nj,2)-x(2,t),mf(nj,1)-x(1,t))-x(3,t)] + d;
                    %y(nj,t) = mod(y(nj,t)+pi,2*pi)-pi;
                case 3
                    y(nj:nj+1,t) = [max(0.001, sqrt((mf(nj,1)-x(1,t))^2 + (mf(nj,2)-x(2,t))^2));
                        atan2(mf(nj,2)-x(2,t),mf(nj,1)-x(1,t))-x(3,t)] + d;
                    %y(nj+1,t) = mod(y(nj+1,t)+pi,2*pi)-pi;
                    nj = nj+1;
                    mf(nj,:) = map(i,:);

            end
        end
    end
    
    
    %% Particle filter estimation
    for dd=1:D
        e = RE*sqrt(Re)*randn(n,1);
        Xp(:,dd) = [X(1,dd)+u(1,t)*cos(X(3,dd))*dt;
                    X(2,dd)+u(1,t)*sin(X(3,dd))*dt;
                    X(3,dd)+u(2,t)*dt] + e;
    end
    if (nj > 0)
        for dd=1:D
            Qj = zeros(nj,nj);
            hXp =[];
            for j=1:nj
                switch(meas)
                    case 1
                        hXp(j) = [sqrt((mf(j,1)-Xp(1,dd))^2 + (mf(j,2)-Xp(2,dd))^2)];
                        Qj(j,j) = Q;
                    case 2
                        hXp(j) = [atan2(mf(j,2)-Xp(2,dd),mf(j,1)-Xp(1,dd))-Xp(3,dd)];
                        %hXp(j) = mod(hXp(j)+pi,2*pi)-pi;
                        Qj(j,j) = Q;
                    case 3
                        if (mod(j,2))
                            hXp(j) = [sqrt((mf(j,1)-Xp(1,dd))^2 + (mf(j,2)-Xp(2,dd))^2)];
                        else
                            hXp(j) = [atan2(mf(j,2)-Xp(2,dd),mf(j,1)-Xp(1,dd))-Xp(3,dd)];
                            %hXp(j) = mod(hXp(j)+pi,2*pi)-pi;
                            Qj(j-1:j,j-1:j) = Q;
                        end
                end
            end
            % Wraparound issue with heading must be resolved here
            % Switch heading measurement to be relative to hXp 
            % Use y-hXp and 0 instead of y, hXp, and then make sure y-hXp
            % falls between -pi and pi
            y_fixed = y(1:nj,t)';
            y_fixed(2:2:nj) = mod(y_fixed(2:2:nj) - hXp(2:2:nj) + pi,2*pi)-pi;
            hXp(2:2:nj) = 0;
            % Calculate weight for each particle using measurement model
            % (Additive gaussian noise on measurements)
            w(dd) = mvnpdf(y_fixed,hXp,Qj);
        end
        % Form cumulative distribution for sampling
        W = cumsum(w);
        % Perform importance sampling update
        for dd=1:D
            seed = max(W)*rand(1);
            X(:,dd) = Xp(:,find(W>seed,1));
        end
    else
        % If no measurements, simply propagate motion model
        X = Xp;
    end
    % Take mean and covariance of particle set as state estimate
    muParticle = mean(X');
    SParticle = var(X');
    muParticle_S(:,t) = muParticle;
    %% Plot results
    figure(1);clf; hold on;
    axis equal
    axis([-4 6 -1 7]);
    plot(map(:,1),map(:,2),'go', 'MarkerSize',10,'LineWidth',2);
    plot(x(1,1:t),x(2,1:t), 'ro--')
    for j = 1:nj
        plot(mf(j,1),mf(j,2),'mx', 'MarkerSize',10,'LineWidth',2)
        if (meas==1) circle(1,x(1:2,t), y(j,t)); end
        if (meas==2) plot([x(1,t) x(1,t)+10*cos(y(j,t)+x(3,t))], [ x(2,t) x(2,t)+10*sin(y(j,t)+x(3,t))], 'c');end
        if (meas==3)
            if (mod(j,2))
                plot([x(1,t) x(1,t)+y(j,t)*cos(y(j+1,t)+x(3,t))], [ x(2,t) x(2,t)+y(j,t)*sin(y(j+1,t)+x(3,t))], 'c');
            end
        end
    end
    for dd=1:D
        plot([X(1,dd) X(1,dd)+0.2*cos(X(3,dd))],[X(2,dd) X(2,dd)+0.2*sin(X(3,dd))],'b')
    end
    title('Particle Filter Localization')
    if (makemovie) writeVideo(vidObj, getframe(gca)); end
    
end
if (makemovie) close(vidObj); end

figure(2); clf; hold on;
plot(map(:,1),map(:,2),'go', 'MarkerSize',10,'LineWidth',2);
plot(x(1,1:t),x(2,1:t), 'ro--')
for t = 2:length(T)
    plot(muParticle_S(1,t),muParticle_S(2,t),'b.')
    plot([muParticle_S(1,t) muParticle_S(1,t)+0.2*cos(muParticle_S(3,t))],[muParticle_S(2,t) muParticle_S(2,t)+0.2*sin(muParticle_S(3,t))],'k')
end
axis equal
