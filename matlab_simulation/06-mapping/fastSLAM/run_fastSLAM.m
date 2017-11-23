%% RUN THE FAST SLAM FUNCTION
% INPUTS:(in order of call)
% -------
% None
% OUTPUTS:(in order of call)
% --------
% None
% INSTRUCTIONS:
%-------------
% 1.Settings can be changed in the 'config' structure
% 2.Change function name to fastSLAM2 to invoke fastSLAM2.
% 3.Set randMapFlag to 0 to load the static map.(staticMap.m)must
% exist.This is useful for comparision of fastSLAM1.0 and 2.0. Setting this
% value to 1 randomizes the map in each iteration.
% 4. Set the makemovie flag to 1 to record a movie
%
% NOTE: Advantages of the fastSLAM2.0 can be observed by reducing the no of
% features, increassing the motion noise, increasing total features
% COMMENTS:
% 1.TruePose(BLACK DELTA): here refers to the pose,we want the robot to undertake.But it
% reality it is corrupted by motion noise(RED).This is just for reference.
% The robot in reality executes the red trajectory which we have to track
% using fastSLAM.
% 2. The movie making is currently disabled.But can be added.
% 3. Q=Motion Noise and R=Measurement Noise, unlike the slides(flipped).
% 4. Error is plotted after the simulation is complete and RMS is displayed
% in prompt.
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initializations
clc; clear all; close all; 
% Initialize Configuration Structure
config=struct('MotNoise',{[0.001 0 0;
                           0 0.001 0;
                           0 0 0.0001]},...
              'MeasNoise',{[0.02 0;
                            0 0.02]},...
              'finalTime',{20},...
              'timeInterval',{0.1},...
              'noFeatures',{20},...
              'totalParticles',{100},...
              'maxRange',{10},...
              'maxFOV',{pi/4},...
              'randMapFlag',{0},...
              'makemovie',{1});% Set=1:to randomize Map. 0:to load static map.(staticMap.m must exist)
config.newFeat=ones(config.noFeatures,1);
Q=config.MotNoise;
R=config.MeasNoise;

% Primary Initializations           
pose(:,1)=[0,0,0]';%initial Pose
truePose(:,1)=pose(:,1);
u=[1;0.3];
%Map Initializations
if(config.randMapFlag==1)
map = 10*rand(2,noFeatures);
map(1,:) = map(1,:)-5; 
map(2,:) = map(2,:)-2; 
else
    load('staticMap.mat');
end

% Define the Particle Structure
particles_old(1:config.totalParticles)=struct('pose',{zeros(size(Q,1),1)},...
                     'meanFeat',{zeros(size(R,1),config.noFeatures)},...
                     'covFeat',{zeros(size(R,1),size(R,1),config.noFeatures)},...
                     'weight',{1/config.totalParticles});
measurements=struct('indices',{},...
                    'value',{});
particleSet.p=particles_old;
particles_new=particles_old;


%Initialize ProgessBar
h = waitbar(0,'Initializing waitbar...');
%% Fast SLAM1.0 LOOP

for t=2:config.finalTime/config.timeInterval
    oldPose=pose(:,t-1);
    [newPose,measurements(t),particles_new,newfeature]...
        =func_fastSLAM2(oldPose,u,config,map,particles_old);
    %STORE the Pose,Centroid and timestamp
    pose(:,t)=newPose;
    particleSet(t).p=particles_new;
    %calculate the centroid of particles
    centroid(:,t)=[0;0;0];
    for i=1:config.totalParticles
    centroid(:,t)=centroid(:,t)+particles_new(i).pose;
    end
    centroid(:,t)=centroid(:,t)./config.totalParticles;
    
    %Updates the true pose of the robot
    truePose(:,t)=motionUpdate_fs(truePose(:,t-1),u,zeros(3),config.timeInterval);
    
    % Swap Particles    
    particles_old=particles_new;    
    config.newFeat=newfeature;
    
%Show Progess
perC=t/(config.finalTime/config.timeInterval);
waitbar(perC,h,sprintf('Computing...%3.1f%% completed.',perC*100));
end
close(h);%Close the waitbar

%Plot the results
plot_fs;
