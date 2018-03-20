
%% FastSLAM and FastSLAM v.2 example
clear;clc;close all

%% Create AVI object
makemovie = 0;
if(makemovie)
    vidObj = VideoWriter('fastSLAM.mp4');
    vidObj.Quality = 100;
    vidObj.FrameRate = 2;
    open(vidObj);
end


%% Select an example to run
% Example=1: robot travelling in a circular path between 2 lines of obstacles
% Example=2: robot travelling back and forth between 2 lines of obstacles
% Example=3: robot travelling in a circular path. 6 obstacles form a small circle
% Example=4: robot travelling in a circular path. 6 obstacles form a large circle
% Example=5: robot travelling in a circular path. 10 randomly placed
%            features
Example=1;


% The number of particles to use in the simulation
numParticles = 1000;

slamVersion = 2;
% Select the fast slam implementation to use (func_fastSLAM or func_fastSLAM2)
switch slamVersion
    case 1
        fastSlamFn = @func_fastSLAM;
    case 2
        fastSlamFn = @func_fastSLAM2;
    otherwise
        fastSlamFn = @func_fastSLAM;
end
        

%% Main Code
% Time
Tf = 25;
dt = 0.5;
T = 0:dt:Tf;

% Initial Robot State: (x,y,heading)
x0 = [0 0 0]';

% Generate control inputs and feature map based on example selection
switch Example
    case 1
        u = ones(2, length(T));
        u(2,:)=0.3*u(2,:);
        map = [-5:1:5 5:-1:-5; -2*ones(1,11) 8*ones(1,11)];
        axisDim = [ -6 6 -3 9 ]; 
    case 2
        turns=5;    % number of 180 deg turns to make
        u=ones(2,length(T));
        u(2,:)=0;
        for n=1:turns
            u(2,floor(length(T)./(turns+1)*n))=pi/dt;
        end
        map=[-1:1:8,8:-1:-1; 2*ones(1,10),-2*ones(1,10)];
        axisDim = [ -2 9 -3 3 ]; 
    case 3
        u = ones(2, length(T));
        u(2,:)=0.3*u(2,:);
        N=6; %number of features
        Radius=6;
        center=[1;3];
        theta=linspace(2*pi./N,2*pi,N);
        map=[center(1)+Radius.*cos(theta);
             center(2)+Radius.*sin(theta)];
        axisDim = [ -6 8 -4 10 ]; 
    case 4
        u = ones(2, length(T));
        u(2,:)=0.3*u(2,:);
        N=6; %number of features
        Radius=12;
        center=[1;3];
        theta=linspace(2*pi./N,2*pi,N);
        map=[center(1)+Radius.*cos(theta);
             center(2)+Radius.*sin(theta)];
        axisDim = [ -12 14 -10 16 ]; 
    case 5
        u = ones(2, length(T));
        u(2,:)=0.3*u(2,:);
        N=16; %number of features
        map = 10*rand(2,N);
        map(1,:) = map(1,:)-5; 
        map(2,:) = map(2,:)-2; 
        axisDim = [ -6 6 -3 9 ]; 

end
M = length(map);

% Motion Disturbance model
R = [0.01  0     0; 
     0     0.01  0; 
     0     0     0.001];
[RE, Re] = eig(R);

% Prior over robot state
mu0r = [0 0 0]'; % mean (mu)

% Prior over feature map
newFeatures = zeros(M,1);

%Measurement model
rMax = 20; % Max range
halfThetaMax = pi/4; % 1/2 Field of view

% Measurement noise
Qi = 0.01 * [0.02 0; 
       0    0.02];
[QiE, Qie] = eig(Qi);

% pre-compute noise multipliers
motionNoiseFactor = RE * sqrt(Re);
measurementNoiseFactor = QiE * sqrt(Qie);


% Simulation Initializations
n = size(x0,1); % Number of vehicle states
xr = zeros(n,length(T)); % Vehicle states 
xr(:,1) = x0;
m = length(Qi(:,1)); % Number of measurements per feature 
y = zeros(m*M,length(T)); % Measurements

particles(1:numParticles) = struct( ...
    'mu',    { zeros(size(x0,1),1) }, ...
    'f_mu',  { zeros(2, M) }, ...
    'f_cov', { zeros(2, 2 * M) }, ...
    'w',     { 0.00001 } );

mu_S = zeros(n,length(T)); % Belief

count=0;    % count of detected features in the map

%% Plot initial step
t=1;
% //figure(1);clf; 


if (makemovie) 
    writeVideo(vidObj, getframe(gcf)); 
end

%% Main loop
tic;
for t=2:length(T)
    %% Simulation
    
    % Select a motion disturbance
    e = motionNoiseFactor*randn(n,1);
    % Update robot state
    xr(:,t) = simpleRobotMotionModel(xr(:,t-1), u(:,t), dt)+e;

    % Find any features that are visible... (Each particle should have its own independent field of view...)
    % and collect their measurements (noisy of course)
    foundFeatures = searchForFeatures( xr(:,t), map, rMax, halfThetaMax, measurementNoiseFactor );
    
    [particles,newFeatures] = fastSlamFn(particles, foundFeatures, u(:,t), R, Qi, newFeatures, dt);

    % calculate the new belief by averaging the particles
    mu = sum( [particles.mu], 2 ) / numParticles;
    
    % Store results
    mu_S(:,t) = mu;

    %% Plot results
    figure(1);clf; hold on;
    axis equal
    axis(axisDim)
    
    %% Color Map Intialization
    cmap = colormap('jet');
    cmap = cmap(1:3:end,:);
    cn = length(cmap(:,1));

    title('FastSLAM with Range & Bearing Measurements');
    %% Plot the Map
    for j = 1:M
       plot(map(1,j),map(2,j),'o','Color', cmap(mod(j,cn)+1,:), 'MarkerSize',10,'LineWidth',2);
    end    
    
    s = max(1,t - 30);
    % plot the true motion
    plot(xr(1,s:t), xr(2,s:t), 'ro');

    % plot the lines of view
    for j=1:size(foundFeatures,2) 
        fp = locateFeature( xr(:,t), foundFeatures(1:2,j) );
        plot([xr(1,t) fp(1)], [xr(2,t) fp(2)], 'Color', cmap(mod(j,cn)+1,:) );
    end
    
    % plot the particles
    step = max(7,floor(numParticles) * 0.1);
    for p = 1:step:numParticles
        plot(particles(p).mu(1),particles(p).mu(2), 'b.');
        for i = 1:length(map)
            if newFeatures(i) == 1
                plot(particles(p).f_mu(1,i), particles(p).f_mu(2,i), 'b.', 'Color', cmap(mod(i,cn)+1,:));
            end
        end
    end
    
    % plot the belief (center of the particles)
    plot(mu_S(1,s:t),mu_S(2,s:t), 'k*')

    if (makemovie) 
        writeVideo(vidObj, getframe(gcf)); 
    end
end
toc

if (makemovie) 
    close(vidObj); 
end

%Error Between true pose and fast slam prediction
err = xr - mu_S;
figure(2);
subplot(3,1,1);
plot(err(1,:));hold on;title('Error Plots');xlabel('time');ylabel('X_{err}');
subplot(3,1,2);
plot(err(2,:));hold on;title('Error Plots');xlabel('time');ylabel('Y_{err}');
subplot(3,1,3);
plot(err(3,:));hold on;title('Error Plots');xlabel('time');ylabel('Theta_{err}');
fprintf('RMS error in X_pos: %f \n',rms(err(1,:)));
fprintf('RMS error in Y_pos: %f \n',rms(err(2,:)));
fprintf('RMS error in Orientation: %f \n',rms(err(3,:)));
