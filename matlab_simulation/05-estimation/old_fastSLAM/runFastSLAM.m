%% Fast SLAM Example
% Author: Bismaya Sahoo.
%% Initializations
clear all; close all; clc;
% Primary Initializations           
poseInitial=[0,0,0]';%initial Pose
dt=0.1; timeInterval=dt;%time interval
finalTime=20;%final Time in sec
MotionNoise=[0.001 0 0;0 0.001 0;0 0 0.0001];R=MotionNoise;
MeasurementNoise=[0.02 0;0 0.02];Q=MeasurementNoise;
noFeatures=20;% No of features in Map
ParticleSize=100;%No of particles
rmax = 10;
thmax = pi/4;
u=[1;0.3];
%Fast Slam Specific Initializations
pose(:,1)=poseInitial;
totalParticles=ParticleSize;
particleSet=zeros(size(poseInitial,1),totalParticles);
muFeat=zeros(size(Q,1),noFeatures,totalParticles);
muFeatPred=muFeat;
covFeat=zeros(size(Q,1),size(Q,1),noFeatures,totalParticles);
covFeatPred=covFeat;
newfeature = ones(totalParticles,1);
w_initial=1/totalParticles;
w=w_initial*ones(1,totalParticles);
timeStamp(1)=timeInterval; 
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
    %call the fast slam function
    [newPose,map,y,muParticleNew,covParticleNew,newParticleSet,meas_ind,newfeature,centroid_particles,w_new]...
        =func_fastSLAM2(oldPose,u,MotionNoise,MeasurementNoise,map,oldParticleSet,muFeatOld,covFeatOld,w_old,rmax,thmax,timeInterval,totalParticles,newfeature);
    
% % %     UNCOMMENT to call the fast2.0 slam function
%     [newPose,map,y,muParticleNew,covParticleNew,newParticleSet,meas_ind,newfeature,centroid_particles,w_new]...
%         =fastSLAM2(oldPose,u,MotionNoise,MeasurementNoise,map,oldParticleSet,muFeatOld,covFeatOld,w_old,rmax,thmax,timeInterval,totalParticles,newfeature);
    
    %Update the Pose,Centroid and timestamp
    timeStamp(t)=timeStamp(t-1)+timeInterval;
    pose(:,t)=newPose;
    centroid(:,t)=centroid_particles';
    
    %Plot the Results
    plot_fs(pose,map,y,muParticleNew,newParticleSet,t,meas_ind,totalParticles,noFeatures,newfeature,centroid);
    
    % Swap for the next iteration        
    oldParticleSet=newParticleSet;
    muFeatOld=muParticleNew;
    covFeatOld=covParticleNew;
    w_old=w_new;
end


  