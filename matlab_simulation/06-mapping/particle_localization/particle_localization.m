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
meas = 3;
switch(meas)
case 1  % 1 - range
    Q = 0.01;
case 2  % 2 - bearing
    Q = 0.01;
case 3  % 3 - both
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
muParticle_S = zeros(1, D, length(T));

% plot_initial_state(1, map, x, X, D);


% Main loop
for t = 2:length(T)    
    % update state
    e = RE * sqrt(Re) * randn(n, 1);
    x(:, t) = [x(1, t - 1) + u(1, t) * cos(x(3, t - 1)) * dt;
              x(2, t - 1) + u(1, t) * sin(x(3, t - 1)) * dt;
              x(3, t - 1) + u(2, t) * dt] + e;
    
    % take measurement
    % pick feature
    mf(:,t) = closestfeature(map,x(:,t));
    % Select a motion disturbance
    d = QE * sqrt(Qe) * randn(m, 1);
    % determine measurement
    switch(meas) 
        case 1  % 1 - range
            r = range(mf(1, t), x(1, t), mf(2, t), x(2, t));
            y(:, t) = r + d;
        case 2  % 2 - bearing
            b = bearing(mf(1, t), mf(2, t), x(1, t), x(2, t), x(3, t));
            y(:, t) = b + d;
        case 3  % 3 - both
            r = range(mf(1, t), x(1, t), mf(2, t), x(2, t));
            b = bearing(mf(1, t), mf(2, t), x(1, t), x(2, t), x(3, t));
            y(:, t) = [r; b] + d;
    end

    % particle filter estimation
    for dd = 1:D
        e = RE * sqrt(Re) * randn(n, 1);
        Xp(:, dd) = [X(1, dd) + u(1, t) * cos(X(3, dd)) * dt;
                    X(2, dd) + u(1, t) * sin(X(3, dd)) * dt;
                    X(3, dd) + u(2, t) * dt] + e;
        switch(meas)
            case 1  % 1 - range
                r = range(mf(1, t), Xp(1, dd), mf(2, t), Xp(2, dd));
                hXp = r + d;
            case 2  % 2 - bearing
                b = bearing(mf(1, t), mf(2, t), Xp(1, t), Xp(2, t), Xp(3, t));
                hXp = b + d;
            case 3  % 3 - both
                r = range(mf(1, t), Xp(1, dd), mf(2, t), Xp(2, dd));
                b = bearing(mf(1, t), mf(2, t), Xp(1, dd), Xp(2, dd), Xp(3, dd));
                hXp = [r; b] + d;
        end
        w(dd) = max(1e-8, mvnpdf(y(:, t), hXp, Q));
    end
    W = cumsum(w);

    for dd = 1:D
        seed = max(W) * rand(1);
        X(:, dd) = Xp(:, find(W > seed, 1));
    end

    muParticle = mean(X);
    SParticle = var(X);    muParticle_S(1, :, t) = muParticle;
    
    % record
    if (makemovie) 
        % plot results
        plot_results(1, meas, map, t, x, X, y, mf, D);
        
        % record plot frame
        writeVideo(vidObj, getframe(gca)); 
    end
end

% close video object
if (makemovie) 
    close(vidObj); 
end