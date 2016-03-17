%% RUN THE FAST SLAM FUNCTION
% INPUTS:(in order of call)
% -------
% None
% OUTPUTS:(in order of call)
% --------
% None
% INSTRUCTIONS:
%-------------
% Changes can be made in the initialization section.(Straightforward)
% UNCOMMENT specified sections to run FAST SLAM1.0 or FAST SLAM2.0
% NOTE: Advantages of the fastSLAM2.0 can be observed by reducing the no of
% features, increassing the motion noise, increasing total features
% FUTURE IMPROVEMENTS
% 1. ADD true robot pose
% 2. Plot the errors
% 3. Figure out a cool way to compare both.
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initializations
clear all; close all; clc;
% Primary Initializations           
poseInitial=[0,0,0]';%initial Pose
dt=0.1; timeInterval=dt;%time interval
finalTime=20;%final Time in sec
MotionNoise=[0.001 0 0;0 0.001 0;0 0 0.0001];R=MotionNoise;
MeasurementNoise=[0.02 0;0 0.02];Q=MeasurementNoise;
noFeatures=20;% No of features in Map
totalParticles=100;%No of particles
rmax = 10;
thmax = pi/4;
u=[1;0.3];

%Fast Slam Specific Initializations
pose(:,1)=poseInitial;
particleSet=zeros(size(poseInitial,1),totalParticles);
muFeat=zeros(size(Q,1),noFeatures,totalParticles);
muFeatPred=muFeat;
covFeat=zeros(size(Q,1),size(Q,1),noFeatures,totalParticles);
covFeatPred=covFeat;
newfeature = ones(noFeatures,1);
w_initial=1/totalParticles;
w=w_initial*ones(1,totalParticles);
timeStamp(1)=timeInterval; 
truePose(:,1)=poseInitial;
%Map Initializations
map = 10*rand(2,noFeatures);
map(1,:) = map(1,:)-5; 
map(2,:) = map(2,:)-2; 

%% Fast SLAM LOOP
for t=2:finalTime/timeInterval
    oldPose=pose(:,t-1);
    if(t==2)
        oldParticleSet=particleSet;
        w_old=w;
        muFeatOld=muFeat;
        covFeatOld=covFeat;
     end
%% % %     UNCOMMENT to call the fast slam1.0 function
    [newPose,y,muParticleNew,covParticleNew,newParticleSet,meas_ind,newfeature,w_new]...
        =func_fastSLAM(oldPose,u,MotionNoise,MeasurementNoise,map,oldParticleSet,muFeatOld,covFeatOld,rmax,thmax,timeInterval,totalParticles,newfeature,w_old);
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     

%% % %     UNCOMMENT to call the fast2.0 slam function
%     [newPose,y,muParticleNew,covParticleNew,newParticleSet,meas_ind,newfeature,w_new]...
%         =func_fastSLAM2(oldPose,u,MotionNoise,MeasurementNoise,map,oldParticleSet,muFeatOld,covFeatOld,rmax,thmax,timeInterval,totalParticles,newfeature,w_old);
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

    %STORE the Pose,Centroid and timestamp
    timeStamp(t)=timeStamp(t-1)+timeInterval;
    pose(:,t)=newPose;
    centroid(:,t)=mean(newParticleSet');
    
    %Updates the true pose of the robot
    truePose(:,t)=motionUpdate_fs(truePose(:,t-1),u,zeros(3),timeInterval);
    %Plot the Results
    plot_fs(pose,map,y,muParticleNew,newParticleSet,t,meas_ind,totalParticles,noFeatures,centroid,truePose);
    
    % Swap for the next iteration        
    oldParticleSet=newParticleSet;
    muFeatOld=muParticleNew;
    covFeatOld=covParticleNew;
    w_old=w_new;
 end


  