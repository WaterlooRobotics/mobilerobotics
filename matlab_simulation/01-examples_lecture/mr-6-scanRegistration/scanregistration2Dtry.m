%% Scan Registration example
clear;clc;

%% Create AVI object
makemovie = 0;
if(makemovie)
    vidObj = VideoWriter('ICP2d.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 5;
    open(vidObj);
end

% Time
Tf = 50;
dt = 0.5;
T = 0:dt:Tf;

% True map
M = 200;
N = 200;
dx = 0.1;
I = imread('funnymap.jpg');
I = imresize(I, [M N]);
map = im2bw(I, 0.7); % Convert to 0-1 image
map = 1-flipud(map'); % Convert to 0 free, 1 occupied and flip.

% Map and vehicle path
figure(1);clf; hold on;
image(100*(1-map'));
colormap(gray);
axis equal
axis([0 M 0 N]);
if (makemovie) writeVideo(vidObj, getframe(gcf)); end

% Initial Robot State
x0 = [5 4 pi/2]';

% Motion Disturbance model
R = [0.001 0 0; 
     0 0.001 0; 
     0 0 0.0001];
[RE, Re] = eig(R);

% Control inputs
u = zeros(2, length(T));
u(1,:) = 1;
u(2,:) = -0.01;

% Measurement model
rmax = 20;
meas_phi = [-pi/1.1:0.05:pi/1.1];
Q = 0.0002;
d = sqrt(Q)*randn(1,1);
x0s = [x0(1)/dx; x0(2)/dx; x0(3)];
meas_r = getranges(map,x0s,meas_phi,rmax/dx);

% Make first measurement point cloud
kk=1;
for jj = 1:length(meas_phi)
    if(meas_r(jj) < rmax/dx)
        pp(kk,:) = [dx*meas_r(jj)*cos(meas_phi(jj)), dx*meas_r(jj)*sin(meas_phi(jj))];
        kk = kk+1;
    end
end

% Simulation Initializations
n = length(R(:,1)); % Number of vehicle states
xr = zeros(n,length(T)); % Vehicle states 
xr(:,1) = x0;
xICP = x0;

% Belief map
m = 0.5*ones(M,N);
L0 = log(m./(1-m));
L=L0;

% Motion planning look ahead
dxy1 = 5;
dxy2 = 15;

%% Main loop
for t=2:length(T)

    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update robot state
    xr(:,t) = [xr(1,t-1)+u(1,t)*cos(xr(3,t-1))*dt;
              xr(2,t-1)+u(1,t)*sin(xr(3,t-1))*dt;
              xr(3,t-1)+u(2,t)*dt] + e;

    % Jump through obstacles if not avoided
    xc = max(1,min(N,round(xr(1,t)/dx)));
    yc = max(1,min(M,round(xr(2,t)/dx)));
    if (map(xc,yc)) 
        xICP(:,t) = xr(:,t);
        continue; 
    end
    
    % Define measurements
    d = sqrt(Q)*randn(1,1);
    xrs = [xr(1,t)/dx; xr(2,t)/dx; xr(3,t)]
    meas_r(:,t) = getranges(map,xrs,meas_phi,rmax/dx);

    % Make current measurement point cloud
    kk = 1; ppcur = [];
    for jj = 1:length(meas_phi)
        if (meas_r(jj,t) < rmax/dx)
            ppcur(kk,:) = [dx*meas_r(jj,t)*cos(meas_phi(jj)),dx*meas_r(jj,t)*sin(meas_phi(jj))]; 
            kk=kk+1;
        end
    end
    
    % Control modification for collisions
    [min_range, minr_ind] = min(meas_r(:,t));
    u(2,t+1) = -sign(meas_phi(minr_ind))*0.25;

    % [max_range, maxr_ind] = max(meas_r(:,t));
    % u(2,t+1) = -sign(meas_phi(maxr_ind))*0.5;
    
%    front = find(meas_phi > 0,1);
%    if (meas_r(front,t)< 25/dx)
%        if (meas_r(front+5,t) > meas_r(front-5,t))
%            u(2,t+1) = 0.45;
%        else
%            u(2,t+1) = -0.45;
%        end
%    end
    
    %% ICP
    % Store previous and current point cloud
    qq = pp;
    pp = ppcur;
   
    % Predicted transformation
    Rp = [cos(u(2,t)) -sin(u(2,t)); sin(u(2,t)) cos(u(2,t))];
    Tp = [u(1,t)*dt; 0];
    Rp
    Tp
    
    % Perform ICP scan registration
    [Ricp Ticp] = icp2try(qq', pp', Rp, Tp, 5);
    Ricp 
    Ticp 
    
    % Store robot pose estimate
    x_cur = xICP(:,t-1);
    Rth = [cos(x_cur(3)) -sin(x_cur(3)); sin(x_cur(3)) cos(x_cur(3))];
    xICP(1:2,t) = x_cur(1:2) + Rth*Ticp;
    xICP(3,t) = x_cur(3) + asin(Ricp(2,1));
    
    % Transform pointcloud for display
    pp2 = [];
    for kk = 1:length(pp)
        pp2(kk,:) = (Ricp*pp(kk,:)' + Ticp)'; 
    end
    
    diffT(:,t) = (xr(1:2,t)-xr(1:2,t-1));
    diffICP(:,t) = Ticp;
    errICP(t) = norm(diffT(:,t)-diffICP(:,t));
    errICP(t)
    
    %% Map update
    measL = zeros(M,N);
    for i = 1:length(meas_phi)
        % Get inverse measurement model (with scan registration estimate or
        % motion prediction)
        invmod = inversescannerbres(M,N,xICP(1,t)/dx,xICP(2,t)/dx,meas_phi(i)+xICP(3,t),meas_r(i,t),rmax/dx);
        for j = 1:length(invmod(:,1));
            ix = invmod(j,1);
            iy = invmod(j,2);
            il = invmod(j,3);
            % Calculate updated log odds
            L(ix,iy) = L(ix,iy) +log(il./(1-il))-L0(ix,iy);
            measL(ix,iy)= measL(ix,iy) +log(il./(1-il))-L0(ix,iy);
        end
    end
    % Calculate probabilities
    m = exp(L)./(1+exp(L));
    invmod_T = exp(measL)./(1+exp(measL));

    % Map, scans and vehicle path
    figure(1);clf; 
    subplot(2,2,1); hold on;
    image(100*(1-map'));
    colormap(gray);
    plot(xr(1,1:t)/dx,xr(2,1:t)/dx,'bx-')
    plot(xICP(1,2:t)/dx,xICP(2,2:t)/dx,'go')
    RBI = [cos(xr(3,t-1)) -sin(xr(3,t-1)); sin(xr(3,t-1)) cos(xr(3,t-1))];
    for i=1:length(pp2)
        pp2I(i,:) = (RBI* pp2(i,:)'/dx + xr(1:2,t-1)/dx)';
    end
    for i=1:length(qq)
        qqI(i,:) = (RBI* qq(i,:)'/dx + xr(1:2,t-1)/dx);
    end
    plot(pp2I(:,1),pp2I(:,2),'bx');
    plot(qqI(:,1),qqI(:,2),'rx'); 
    legend('Robot pose', 'Scan reg transform', 'Transformed scan', 'Prior scan')
    title('Map, scans and robot poses')
    axis([0 M 0 N])
    axis equal

    % Plot scan registration results
    subplot(2,2,2); hold on;
    plot(qq(:,1), qq(:,2),'bo');
    plot(pp(:,1), pp(:,2),'ro');
    plot(pp2(:,1), pp2(:,2),'go');
    legend('Prior Scan','New scan','Transformed scan')
    title('Scan registration results')
    axis equal
    
    % Inverse measurement model
    subplot(2,2,3); hold on;
    image(100*(invmod_T'));
    colormap(gray);
    plot(xr(1,t)/dx,xr(2,t)/dx,'bx')
    for i=1:length(meas_r(:,t))
        plot(xr(1,t)/dx+meas_r(i,t)*cos(meas_phi(i) + xr(3,t)),xr(2,t)/dx+meas_r(i,t)*sin(meas_phi(i)+ xr(3,t)),'ko')
    end
    title('Inverse measurement model')
    axis([0 M 0 N])
    axis equal

    % Belief map
    subplot(2,2,4); hold on;
    image(100*(m'));
    colormap(gray);
    plot(xICP(1,max(1,t-10):t)/dx,xICP(2,max(1,t-10):t)/dx,'bx-')
    axis([0 M 0 N])
    %F3(t-1) = getframe;
    title('Current occupancy grid map')
    axis([0 M 0 N])
    axis equal
    if (makemovie) writeVideo(vidObj, getframe(gcf)); end
    drawnow;
end
if (makemovie) close(vidObj); end

figure(5);clf;hold on;
plot(errICP)
title('ICP Error 2-norm')
xlabel('Timestep')
ylabel('Error')
