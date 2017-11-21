%Example on how to use the VoronoiDecomposition path planning function.
%The robot is an omni-directional wheeled robot with three wheels.
%The environment is an 50mX50m parking lot, filled with pylons at a known
%random location.
clear

%Start and End points
start = [5,5]; %Start at position x=5m, y=5m
finish = [45,40]; %End at postiion x=80m, y=90m

%Generate obstacles (pylons)
num_obstacles = 60;
x_o = rand(1,num_obstacles)*50;
y_o = rand(1,num_obstacles)*50;

%Generate path through random obstacles and show map of path
[path_coord,path_length] = voronoiDecomposition(start,finish,[x_o',y_o'],true);

%Select time to reach end of path to determine robot speed.
time = 120; %120 seconds to reach target
avg_vel = path_length/time;

%Motion planning
%With an omnibot, we can move in any direction without changing the head of
%the robot. The only parameter of the robot that needs to be specified is
%the radius of the wheels.
wheel_radius = 0.25;

%Calculate velocity and time for each move
segment_time = zeros(1,length(path_coord)-1);
v = zeros(length(path_coord)-1,2);

for i=1:length(path_coord)-1
    x_dist = path_coord(i+1,1)-path_coord(i,1);
    y_dist = path_coord(i+1,2)-path_coord(i,2);
    segment_time(i)=sqrt(x_dist^2+y_dist^2)/avg_vel;
    v(i,1) = sign(x_dist)*sqrt(avg_vel^2/(1+(y_dist/x_dist)^2));
    v(i,2) = v(i,1)*y_dist/x_dist;
end

%Convert the global velocity of the robot to local angular velocities of
%each wheel.
wv(:,3) = v(:,1)/sqrt(3)-v(:,2)/3;
wv(:,2) = wv(:,3) - 2*v(:,1)/sqrt(3);
wv(:,1) = -(wv(:,2)+wv(:,3));
w = wv./wheel_radius; %Angular velocity of wheel (rad/s)

%Resulting motion
motion_state =[w,segment_time'] %format:[wheel1,wheel2,wheel3,time]

%Prove motion using simulation
x(:,1) = [start,0]'; %Initial postion and direction
r = 0.25; %Wheel radius
l = 0.3; %Wheel to center distance

for j=1:length(motion_state)
    wv_rad(1) = motion_state(j,1);
    wv_rad(2) = motion_state(j,2);
    wv_rad(3) = motion_state(j,3);
    t = motion_state(j,4);
    wv = wv_rad.*wheel_radius; %Wheel edge speed in m/s
    vx_l = sqrt(3)/2 * (wv(3)-wv(2)); %Local speed in x
    vy_l = wv(1)-(wv(2)+wv(3))/2; %Local speed in y
    A = [vx_l*cos(x(3,j)) - vy_l*sin(x(3,j)); ...
        vx_l*sin(x(3,j)) + vy_l*cos(x(3,j));...
        (wv(1)+wv(2)+wv(3))/(3*l)];
    x(:,j+1) = x(:,j) + A*t; 
end
hold on
plot(x(1,:),x(2,:),'r:'); 








