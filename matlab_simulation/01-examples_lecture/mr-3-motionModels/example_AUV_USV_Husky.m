%Shared variables initlization
clear
clc
%time in seconds
t=50;
%time step
dt=0.1;
%number of interatins
n=t/dt;


%State initilization
%Husky
x_husky = zeros (3,n);

%USV
x_USV= zeros(6,n); %[x,y,thetha,vx,vy,vthetha]
USV_mass=28; %the mass of Clearpath's Heron in kg

%distance from thrusters to centroid, dimensions for Clearpath's Herons use
%seprate values for non symetrical vehicles
USV_L=0.72;

%AUV
x_AUV= zeros (6,n);
AUV_mass=10;



% Straight line motion

for i=2:n
    %Husky inputs
    omegal_husky=15;
    omegar_husky=15;
    x_husky(:,i)= Husky(x_husky(:,i-1),omegal_husky,omegar_husky,dt);
    
    %USV inputs for constant acceleration
    USV_thrust_l=100;
    USV_thrust_r=100;
    x_USV(:,i)= USV(x_USV(:,i-1),USV_thrust_l,USV_thrust_r, USV_mass, USV_L, USV_L, dt);
    
    %AUV planar motion
    AUV_thrust_x=10;
    AUV_thrust_y=10;
    AUV_thrust_z=0;
    
    x_AUV(:,i) = AUV(x_AUV(:,i-1),AUV_thrust_x, AUV_thrust_y, AUV_thrust_z, AUV_mass,dt);
    
end

figure(1); clf; hold on;
subplot(2,2,1)
plot(x_husky(1,:),x_husky(2,:))
title('Husky')

subplot(2,2,2)
plot(x_USV(1,:),x_USV(2,:))
title('USV')

subplot(2,2,3)
plot3(x_AUV(1,:),x_AUV(2,:),x_AUV(3,:))
title('AUV')

%Now with some more interesting inputs

for i=2:n
    %Husky inputs
    omegal_husky=15*cos(i*3.14159/100);
    omegar_husky=10;
    x_husky(:,i)= Husky(x_husky(:,i-1),omegal_husky,omegar_husky,dt);
    
    %USV inputs for constant acceleration
    USV_thrust_l=50;
    USV_thrust_r=25;
    x_USV(:,i)= USV(x_USV(:,i-1),USV_thrust_l,USV_thrust_r, USV_mass, USV_L, USV_L, dt);
    
    %AUV planar motion
    AUV_thrust_x=10*sin(i*3.14159/100);
    AUV_thrust_y=10*cos(i*3.14159/100);
    AUV_thrust_z=10;
    
    x_AUV(:,i) = AUV(x_AUV(:,i-1),AUV_thrust_x, AUV_thrust_y, AUV_thrust_z, AUV_mass,dt);
    
end

figure(2); clf; hold on;
subplot(2,2,1)
plot(x_husky(1,:),x_husky(2,:))
title('Husky')

subplot(2,2,2)
plot(x_USV(1,:),x_USV(2,:))
title('USV')

subplot(2,2,3)
plot3(x_AUV(1,:),x_AUV(2,:),x_AUV(3,:))
title('AUV')

