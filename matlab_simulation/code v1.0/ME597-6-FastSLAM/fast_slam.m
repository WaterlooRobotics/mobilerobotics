%% Fast SLAM example
clear;clc;

%% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('fastslam.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 5;
    open(vidObj);
end

% Time
Tf = 20;
dt = 0.5;
T = 0:dt:Tf;

% Initial Robot State
x0 = [0 0 0]';

% Motion Disturbance model
R = [0.001 0 0; 
     0 0.001 0; 
     0 0 0.0001];
[RE, Re] = eig(R);

% Control inputs
u = ones(2, length(T));
u(2,:)=0.3*u(2,:);

% Feature Map
M = 20;
map = 12*rand(2,M);
map(1,:) = map(1,:)-5.5; 
map(2,:) = map(2,:)-2; 


% Measurement model
rmax = 10;
thmax = pi/4;

% Feature initialization
newfeature = ones(M,1);

% Measurement noise
Qi = [0.02 0; 
     0 0.02];

[QiE, Qie] = eig(Qi);

% Simulation Initializations
n = length(R(:,1)); % Number of vehicle states
xr = zeros(n,length(T)); % Vehicle states 
xr(:,1) = x0;
m = length(Qi(:,1)); % Number of measurements per feature 
N = n+m*M;

% Particles
D = 100;% Number of particles
X = zeros(n,D); % Vehicle states
X0 = X; % Prior - known exactly
Xp = X; % Interim Vehicle states
mu = zeros(2,M,D); % Feature means
mup = mu; % Interim Feature means
S = zeros(2,2,M,D); % Feature covariances
Sp = S;  % Interim Feature covariances
% Initialization of Particle weights
w0 = 1/D; 
w = w0*ones(1,D);
    
%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update robot state
    xr(:,t) = [xr(1,t-1)+u(1,t)*cos(xr(3,t-1))*dt;
              xr(2,t-1)+u(1,t)*sin(xr(3,t-1))*dt;
              xr(3,t-1)+u(2,t)*dt] + e;

    % Define features that can be measured
    meas_ind = [];
    for i=1:M
        if(inview(map(:,i),xr(:,t),rmax,thmax))
            meas_ind=[meas_ind,i];
        end
    end
    
    % Take measurements
    y = [];
    for i = meas_ind
        % Select a measurement disturbance
        delta = QiE*sqrt(Qie)*randn(m,1);
        % Determine measurement, add to measurement vector
        y = [y, [sqrt((map(1,i)-xr(1,t))^2 + (map(2,i)-xr(2,t))^2);
            mod(atan2(map(2,i)-xr(2,t),map(1,i)-xr(1,t))-xr(3,t)+pi,2*pi)-pi] + delta];
    end
    
    
    %% Fast SLAM Filter Estimation

    % For each particle
    for d=1:D
        % Select a motion disturbance
        em = RE*sqrt(Re)*randn(n,1);
        % Update full state
        Xp(:,d) = [X(1,d)+u(1,t)*cos(X(3,d))*dt;
                  X(2,d)+u(1,t)*sin(X(3,d))*dt;
                  X(3,d)+u(2,t)*dt] + em;

        % For each feature measured
        clear yw hmuw Qw;
        for j=1:length(meas_ind)
            i = meas_ind(j);
            if (newfeature(i) == 1)
                % Feature initialization
                mup(1,i,d) = Xp(1,d)+y(1,j)*cos(y(2,j)+Xp(3,d));
                mup(2,i,d) = Xp(2,d)+y(1,j)*sin(y(2,j)+Xp(3,d));
                % Predicted range
                dx = mup(1,i,d)-Xp(1,d);
                dy = mup(2,i,d)-Xp(2,d);
                rp = sqrt((dx)^2+(dy)^2);
                % Calculate Jacobian
                Ht = [ dx/rp dy/rp; -dy/rp^2 dx/rp^2];
                Sp(:,:,i,d) = Ht\Qi*inv(Ht)';
                
            else
                % Measurement processing
                % Predicted range
                dx = mu(1,i,d)-Xp(1,d);
                dy = mu(2,i,d)-Xp(2,d);
                rp = sqrt((dx)^2+(dy)^2);
                % Calculate Jacobian
                Ht = [ dx/rp dy/rp; -dy/rp^2 dx/rp^2];
                % Calculate Innovation
                I = y(:,j)-[rp; mod(atan2(dy,dx)-Xp(3,d)+pi,2*pi)-pi];
                
                % Measurement update
                Q = Ht*S(:,:,i,d)*Ht' + Qi;
                K = S(:,:,i,d)*Ht'/Q;
                mup(:,i,d) = mu(:,i,d) + K*I;
                Sp(:,:,i,d) = (eye(2)-K*Ht)*S(:,:,i,d);

            end
            % Calculate predicted measurements and covariance for
            % weighting
            yw(2*(j-1)+1:2*j) = y(:,j);
            hmuw(2*(j-1)+1:2*j) = [rp; mod(atan2(dy,dx)-Xp(3,d)+pi,2*pi)-pi];
            Qw(2*(j-1)+1:2*j,2*(j-1)+1:2*j) = 2*Sp(:,:,i,d);
        end
        
        %Calculate weight
        %w(d) = w0;
        if (exist('Qw'))
            w(d) = max(0.000001,mvnpdf(yw,hmuw,Qw));
        end
    end
    
    % Eliminate initialization of previously measured features
    newfeature(meas_ind) = 0;
        
    % Resampling
    W = cumsum(w);

    % Resample and copy all data to new particle set
    for d=1:D
        seed = W(end)*rand(1);
        cur = find(W>seed,1);
        X(:,d) = Xp(:,cur);
        mu(:,:,d) = mup(:,:,cur);
        S(:,:,:,d) = Sp(:,:,:,cur);
    end
    
    muParticle = mean(X');
    mu_S(:,t) = muParticle;

    %% Plot results
    figure(1);clf; hold on;
    plot(xr(1,1:t),xr(2,1:t), 'ro--')
    plot([xr(1,t) xr(1,t)+1*cos(xr(3,t))],[xr(2,t) xr(2,t)+1*sin(xr(3,t))], 'r-')
    cmap = colormap('jet');
    cmap = cmap(1:3:end,:);
    cn = length(cmap(:,1));
    for j=1:length(meas_ind)
        plot([xr(1,t) xr(1,t)+y(1,j)*cos(y(2,j)+xr(3,t))], [xr(2,t) xr(2,t)+y(1,j)*sin(y(2,j)+xr(3,t))], 'Color', cmap(mod(meas_ind(j),cn)+1,:) );
    end
    for j = 1:M
        plot(map(1,j),map(2,j),'o','Color', cmap(mod(j,cn)+1,:), 'MarkerSize',10,'LineWidth',2);
    end
    for d=1:D
        plot(X(1,d),X(2,d),'b.');
        for j = 1:M
            if (~newfeature(j))
                plot(mu(1,j,d),mu(2,j,d),'.','Color', cmap(mod(j,cn)+1,:));
            end
        end
    end
    axis equal
    axis([-8 8 -2 10])
    title('FastSLAM with Range & Bearing Measurements')
    if (makemovie) writeVideo(vidObj, getframe(gca)); end

end
if (makemovie) close(vidObj); end


