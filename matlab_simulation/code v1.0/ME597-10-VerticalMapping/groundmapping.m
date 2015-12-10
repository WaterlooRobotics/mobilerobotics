%% Ground mapping

% Simulation time
Tmax = 100;
T = 0:Tmax;

%% True map
dx = 0.1;
rx = 10;
N = round(rx/dx);
map = ones(N,N);
xmap = 0:dx:rx;
zmap = (sin(xmap+1)+sin(xmap/2 + 0.3) + sin(xmap/3+0.5) + 3);
thmap = atan2(diff(zmap),diff(xmap));
thmap = [thmap, thmap(end)];  %Add final term to make vectors equal length
for i=1:N
    for j=1:N
        if (j > (zmap(i))/dx)
            map(i,j) = 0;
        end
    end
end

figure(1);clf; hold on;
plot(xmap,zmap);
axis([0 rx 0 rx])
figure(2);clf; hold on;
image(100*(map)');
colormap(gray);
axis([0 N 0 N])

%% Belief map
m = 0.5*ones(N,N);
L0 = log(m./(1-m));
L=L0;

%% Sensor model parameters
meas_phi = 3*pi/2 + pi/180*[15:10:65]; % Measurement headings
rmax = 10; % Max range
alpha = 0.1; % Width of an obstacle (Distance about measurement to fill in)
beta = pi/180*3; % Width of a beam (Angle beyond which to exclude) 

%% State Initialization
% Initial Robot location
x0 = [xmap(1) zmap(1)+1 thmap(1)];
x = zeros(3,length(T)+1);
x(:,1) = x0;


%% Main simulation
for t=2:length(T)
    % Robot motion defined by terrain
    x(:,t) = [xmap(t); zmap(t)+1; thmap(t)];
    
    % Generate a measurement data set
    meas_r = getranges(map,x(:,t),meas_phi,rmax,alpha);

    %% Map update
    % Get inverse measurement model
    invmod = inversescanner(N,N,x(1,t),x(2,t),x(3,t),meas_phi,meas_r,rmax,alpha,beta);
    
    % Calculate updated log odds
    L = L +log(invmod./(1-invmod))-L0;

    % Calculate probabilities
    m = 1./(1+exp(L));

    %% Plot results
    
    % Map and vehicle path
    figure(3);clf; hold on;
    image(100*(map)');
    colormap(gray);
    plot(x(1,1:t)/dx,x(2,1:t)/dx,'bx-')
    axis([0 N 0 N])
    F1(t-1) = getframe(gcf);
    
    % Inverse measurement model
    figure(4);clf; hold on;
    image(100*(1-invmod)');
    colormap(gray);
    plot(x(1,t)/dx,x(2,t)/dx,'bx')
    for i=1:length(meas_r)
        plot( (x(1,t)+meas_r(i)*cos(meas_phi(i) + x(3,t)))/dx,(x(2,t)+meas_r(i)*sin(meas_phi(i)+ x(3,t)))/dx,'ko')
    end
    axis([0 N 0 N])
    title('Measurements and inverse measurement model');
    F2(t-1) = getframe(gcf);
    
    % Belief map
    figure(5);clf; hold on;
    image(100*m');
    colormap(gray);
    plot(x(1,max(1,t-10):t)/dx,x(2,max(1,t-10):t)/dx,'bx-')
    plot(xmap*10,zmap*10, 'b');
    axis([0 N 0 N])
    title('Current occupancy grid map')
    F3(t-1) = getframe(gcf);
    
end

