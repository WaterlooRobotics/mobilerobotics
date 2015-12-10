% Extended Kalman Filter SLAM example
clear;clc;

rng('default')

% Time
Tf = 40;
dt = 0.5;
T = 0:dt:Tf;

% Initial Robot State
x0 = [0 0 0]';

% Control inputs
u = ones(2, length(T));
u(2,:)=0.3*u(2,:);

% Motion Disturbance model
R = [0.001 0 0; 
     0 0.001 0; 
     0 0 0.001];
[RE, Re] = eig(R);

% Prior over robot state
mu0r = [0 0 0]'; % mean (mu)
S0rr = 0.00000000001*eye(3);% covariance (Sigma)

% Feature Map
%M = 10;
%map = 8*rand(2,M);
map = [-5:0.5:5 5:-0.5:-5; -2*ones(1,21) 8*ones(1,21)];
M = length(map);
% Prior over feature map
mu0m = zeros(2*M,1);
S0mm = 100*eye(2*M);
newfeature = ones(M,1);

%Measurement model
rmax = 5; % Max range
thmax = pi/4; % 1/2 Field of view

% Measurement noise
Qi = [0.00001 0; 
     0 0.00001];

[QiE, Qie] = eig(Qi);
maxiters = 1;

% Simulation Initializations
n = length(R(:,1)); % Number of vehicle states
xr = zeros(n,length(T)); % Vehicle states 
xr(:,1) = x0;
N = n+2*M;
m = length(Qi(:,1)); % Number of measurements per feature 
y = zeros(m*M,length(T)); % Measurements

mu = [mu0r; mu0m];
S = [S0rr zeros(n,2*M);  zeros(2*M,n) S0mm];
%S = [S0rr 100*ones(n,2*M);  100*ones(2*M,n) S0mm];

mu_S = zeros(N,length(T)); % Belief
mu_S(:,1) = mu;

%% Plot results
t=1;
    figure(1);clf; 
    subplot(1,2,1); hold on;
    plot(map(1,:),map(2,:),'go', 'MarkerSize',10,'LineWidth',2);
    plot(xr(1,1:t),xr(2,1:t), 'ro--')
    plot([xr(1,t) xr(1,t)+1*cos(xr(3,t))],[xr(2,t) xr(2,t)+1*sin(xr(3,t))], 'r-')
    plot(mu_S(1,1:t),mu_S(2,1:t), 'bx--')
    plot([mu_S(1,t) mu_S(1,t)+1*cos(mu_S(3,t))],[mu_S(2,t) mu_S(2,t)+1*sin(mu_S(3,t))], 'b-')
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);

    for i=1:M
          if (~newfeature(i))
              fi = 2*(i-1)+1;
              fj = 2*i;
              plot([xr(1,t) xr(1,t)+y(fi,t)*cos(y(fj,t)+xr(3,t))], [xr(2,t) xr(2,t)+y(fi,t)*sin(y(fj,t)+xr(3,t))], 'c');
              plot(mu(3+fi),mu(3+fj), 'gx')
              mu_pos = [mu(3+fi) mu(3+fj)];
              S_pos = [S(3+fi,3+fi) S(3+fi,3+fj); S(3+fj,3+fi) S(3+fj,3+fj)];
              error_ellipse(S_pos,mu_pos,0.75);
          end
    end
    axis equal
%     axis([-4 6 -1 7])
    title('SLAM with Range & Bearing Measurements')
    
    subplot(1,2,2);
    image(10000*S);
    colormap('gray');
    title('Covariance matrix')
    F(t) = getframe(gcf);

    
%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update robot state
    xr(:,t) = [xr(1,t-1)+u(1,t)*cos(xr(3,t-1))*dt;
              xr(2,t-1)+u(1,t)*sin(xr(3,t-1))*dt;
              xr(3,t-1)+u(2,t)*dt] + e;

    % Take measurements
    % For each feature
    flist = zeros(M,1);
    for i=1:M
        % If feature is visible
        if (inview(map(:,i),xr(:,t),rmax,thmax))
            flist(i) = 1;
            % Select a motion disturbance
            d = QiE*sqrt(Qie)*randn(m,1);
            % Determine measurement
            y(2*(i-1)+1:2*i,t) = [sqrt((map(1,i)-xr(1,t))^2 + (map(2,i)-xr(2,t))^2);
                atan2(map(2,i)-xr(2,t),map(1,i)-xr(1,t))-xr(3,t)] + d;
        end
    end
    
    % Confuse a random pair of current measurements
    s1 = 0; s2= 0;% Indices to confused features
    if(1)
        ind = find(flist); % indices to features observed
        n_m = length(ind); % number of features
        % Remove new features to make task more manageable
        if (1)
            ranlist = find(flist & ~newfeature);
            ind = ranlist;
            n_m = length(ind);
        end
        % If more than 3 features measured, swap 2 of them
        if (n_m > 3)
            s = datasample(ind,2,'Replace',false);
            s1 = s(1); 
            s2 = s(2);
            tmp = y(2*(s1-1)+1:2*s1,t);
            y(2*(s1-1)+1:2*s1,t) = y(2*(s2-1)+1:2*s2,t);
            y(2*(s2-1)+1:2*s2,t) = tmp;
        end
    end
    %% Extended Kalman Filter Estimation
    % Prediction update
    mu(1:3) = [mu(1)+u(1,t)*cos(mu(3))*dt;
           mu(2)+u(1,t)*sin(mu(3))*dt;
           mu(3)+u(2,t)*dt];
    
    Gt = [ 1 0 -u(1,t)*sin(mu(3))*dt;
           0 1 u(1,t)*cos(mu(3))*dt;
           0 0 1];
    
    S(1:n,1:n) = Gt*S(1:n,1:n)*Gt' + R;

    % Measurement RANSAC
    % Get subset of previously measured features
    if(1)
    % RANSAC
    if (length(ranlist)>3)
        NR = 10;
        ran_eps = 0.2;
        best = [];
        for k = 1:NR
            inliers = [];
            curS = datasample(ranlist,1);  % select seed set
            curY =  y(2*(curS-1)+1:2*curS,t)
            curmu = EKF_meas(mu,S,curY,Qi); % update EKF
            for kk = 1:length(ranlist)
                i = ranlist(kk);
                dx = curmu(3+2*(i-1)+1)-curmu(1);
                dy = curmu(3+2*i)-curmu(2);
                rp = sqrt((dx)^2+(dy)^2);
                I = y(2*(i-1)+1:2*i,t)-[rp; (atan2(dy,dx) - mu(3))];
                if (norm(I) < ran_eps)
                    inliers = [inliers, i];
                end
            end
            if (length(inliers)>length(best))
                best = inliers;
            end
        end
        flist
        flist = zeros(M,1);
        flist(best) = 1;
        flist
    end
    end
    % Measurement update
    for i=1:M
        if (flist(i))
            % Feature initialization
            if (newfeature(i) == 1)
                mu(3+2*(i-1)+1) = mu(1)+y(2*(i-1)+1,t)*cos(y(2*i,t)+mu(3));
                mu(3+2*i) = mu(2)+y(2*(i-1)+1,t)*sin(y(2*i,t)+mu(3));
                newfeature(i) = 0;
            else
                for iter = 1:maxiters
                    % Linearization
                    % Predicted range
                    dx = mu(3+2*(i-1)+1)-mu(1);
                    dy = mu(3+2*i)-mu(2);
                    rp = sqrt((dx)^2+(dy)^2);
                    
                    Fi = zeros(5,N);
                    Fi(1:n,1:n) = eye(n);
                    Fi(4:5,3+2*(i-1)+1:3+2*i) = eye(2);
                    Ht = [ -dx/rp ...
                        -dy/rp ...
                        0 ...
                        dx/rp ...
                        dy/rp;
                        dy/rp^2 ...
                        -dx/rp^2 ...
                        -1 ...
                        -dy/rp^2 ...
                        dx/rp^2]*Fi;
                    
                    I = y(2*(i-1)+1:2*i,t)-[rp;
                        (atan2(dy,dx) - mu(3))];
                    I(2) = mod(I(2)+pi,2*pi)-pi;
                    
                    % Measurement update
                    K = S*Ht'*inv(Ht*S*Ht'+Qi);
                    mu = mu + K*I;
                    if (iter==maxiters)
                        S = (eye(n+2*M)-K*Ht)*S;
                    end
                end
            end
        end
    end
    pause(0.5);
    
    % Store results
    mu_S(:,t) = mu;
    
    
    %% Plot results
    figure(1);clf;
    subplot(1,2,1); hold on;
    plot(map(1,:),map(2,:),'go', 'MarkerSize',10,'LineWidth',2);
    plot(xr(1,1:t),xr(2,1:t), 'ro--')
    plot([xr(1,t) xr(1,t)+1*cos(xr(3,t))],[xr(2,t) xr(2,t)+1*sin(xr(3,t))], 'r-')
    plot(mu_S(1,1:t),mu_S(2,1:t), 'bx--')
    plot([mu_S(1,t) mu_S(1,t)+1*cos(mu_S(3,t))],[mu_S(2,t) mu_S(2,t)+1*sin(mu_S(3,t))], 'b-')
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    
    for i=1:M
        if (~newfeature(i))
            fi = 2*(i-1)+1;
            fj = 2*i;
            if (i == s1) || (i == s2)
                plot([xr(1,t) xr(1,t)+y(fi,t)*cos(y(fj,t)+xr(3,t))], [xr(2,t) xr(2,t)+y(fi,t)*sin(y(fj,t)+xr(3,t))], 'r', 'LineWidth',2);
            else
                plot([xr(1,t) xr(1,t)+y(fi,t)*cos(y(fj,t)+xr(3,t))], [xr(2,t) xr(2,t)+y(fi,t)*sin(y(fj,t)+xr(3,t))], 'c');
            end
            plot(mu(3+fi),mu(3+fj), 'gx')
            mu_pos = [mu(3+fi) mu(3+fj)];
            S_pos = [S(3+fi,3+fi) S(3+fi,3+fj); S(3+fj,3+fi) S(3+fj,3+fj)];
            error_ellipse(S_pos,mu_pos,0.75);
        end
    end
    axis equal
    axis([-10 10 -5 15])
    title('SLAM with Range & Bearing Measurements')
    
    subplot(1,2,2);
    image(10000*S);
    colormap('gray');
    title('Covariance matrix')
    
    F(t) = getframe(gcf);
end

figure(2);clf;hold on;
plot(T, xr(1,:)-mu_S(1,:) ,'b')
plot(T, xr(2,:)-mu_S(2,:) ,'r')
plot(T, xr(3,:)-mu_S(3,:) ,'g')
title('Estimation error plot')