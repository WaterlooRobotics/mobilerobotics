%% Occupancy grid mapping with collision avoidance and diagonal motion

% This example uses create_a_map.m to make an interesting environment, then
% uses udrl_robot_motion.m to produce the entire trajectory of the robot
% through the known environment.  It then uses inverse_scanner_bres.m to
% build an occupancy grid map along the way.  It uses the same spinning
% lidar as in previous examples for occupancy grid mapping

% TODO: It would of course be more interesting to see the robot reacting to 
% its own map and updating its motion based on where it thinks obstacles 
% are, and then crashing if it gets it wrong.  

clear;clc;
%% Create AVI object
makemovie1 = 0;
if(makemovie1)
    vidObj1 = VideoWriter('Scanning_bres.avi');
    vidObj1.Quality = 100;
    vidObj1.FrameRate = 2;
    open(vidObj1);
end

makemovie2 = 0;
if(makemovie2)
    vidObj2 = VideoWriter('Map_and_robot.avi');
    vidObj2.Quality = 100;
    vidObj2.FrameRate = 2;
    open(vidObj2);
end

makemovie3 = 0;
if(makemovie3)
    vidObj3 = VideoWriter('Occupancy_grid_map.avi');
    vidObj3.Quality = 100;
    vidObj3.FrameRate = 2;
    open(vidObj3);
end

%% Depiction of the map
% Draw a real map
M = 52; % Height of the map
N = 52; % Width of the map
map_real = create_a_map(M,N); % Create a random map with obstacles

% Generate the belief map
map_bel = 0.5 * ones(M,N); % Give 0.5 probability to uncertain cells
map_bel_logodds = log(map_bel./(1-map_bel)); % Transform the belief map into log-odds form
map_initial = map_bel_logodds; % Initial belif map

%% Robot Motion
T = 200; % 1000[sec] simulation time

% Robot State Initialization
x0= [ round(40 +2*randn(1)), round(40 + 2*randn(1)), 0]; % Initial states of robot
X = zeros(3,length(1:T));

% Produce Robot Motions for entire simulation
X = udlr_robot_motion(map_real, X, x0, T);

%% Sensor Parameters
meas_phi = -0.4:0.05:0.4; % Laser headings
rmax = 20; % Maximum range of laser
alpha = 1; % Distance about measurement to fill in
beta = 0.01; % Angle beyond which to exclude 

%% Inverse measurement model based on Bresenham ray-tracing algorithm
for i = 1 : length(X) % Each robot motion step
   % Actual range of measurement
   meas_r = getranges(map_real,X(:,i),meas_phi,rmax);
   % Initialization of inverse measurement model
   inv_mm_logodds = zeros(M,N);
   for j = 1 : length(meas_r) 
      % Compute inverse measurement model
      inv_meas_mod = inverse_scanner_bres(M, N, X(1,i), X(2,i), X(3,i) + meas_phi(j), meas_r(j), rmax);
      
      for k = 1 : length(inv_meas_mod(:,1)) 
          x_pos = inv_meas_mod(k,1); % x coordinate of the cell
          y_pos = inv_meas_mod(k,2); % y coordinate of the cell
          cell_prob = inv_meas_mod(k,3); % Probability of an obstacle of the cell
          
          % Updates of inverse measurement model    
          inv_mm_logodds(x_pos, y_pos) = inv_mm_logodds(x_pos, y_pos) + log(cell_prob ./ (1-cell_prob)) - map_initial(x_pos,y_pos);       
          % Updates of map
          map_bel_logodds(x_pos, y_pos) = map_bel_logodds(x_pos, y_pos) + log(cell_prob ./ (1-cell_prob)) - map_initial(x_pos,y_pos);
      end
   end
   
   % Transform inverse measurement model & map from log-odds to pobability
     inv_mm_probability = exp(inv_mm_logodds) ./ (1+exp(inv_mm_logodds));
     map_bel_probability = exp(map_bel_logodds) ./ (1+exp(map_bel_logodds));
     
   % Plot Inverse Measurement Model
     figure(1);clf;hold on;
     image(100 * (inv_mm_probability));
     colormap('gray');
     title('Inverse scanner');
     axis ([0 M 0 N]);
     
   % plot areas of have high possibility of obstacles (end points of laser beam)
     for j=1:length(meas_r) 
        plot( X(2,i) + meas_r(j) * sin (meas_phi(j) + X(3,i)) , X(1,i) + meas_r(j) * cos (meas_phi(j) + X(3,i)) ,'ko','MarkerSize',5);
     end  

     
   % Plot real map and robot position
     figure(2);clf;hold on;
     image(100 * (1-map_real));
     colormap('gray');
     plot(X(2,i),X(1,i),'*','Markersize',15);
     plot(X(2,1:i),X(1,1:i),'-','Linewidth',2);
     title('Real map & Robot positions');
     axis ([0 M 0 N]);
     
   % Plot occupancy grid map and robot position
     figure(3);clf;hold on;
     image(100 * (map_bel_probability));
     colormap('gray');
     plot(X(2,i),X(1,i),'w*','Markersize',15);
     title('Occupancy grid mapping');
     axis ([0 M 0 N]);
     drawnow;
     
     if (makemovie1) 
       writeVideo(vidObj1, getframe(gca)); 
     end   
     if (makemovie2) 
       writeVideo(vidObj2, getframe(gca)); 
     end
     if (makemovie3) 
       writeVideo(vidObj3, getframe(gca)); 
     end

end

     

if (makemovie1) 
    close(vidObj1); end
if (makemovie2) 
    close(vidObj2); end
if (makemovie3) 
    close(vidObj3); end
