%% Fast SLAM Call Function. 
% This script is used to call the fastSLAM function after the due
% initializations.
% Author: Bismaya Sahoo. EMAIL:bsahoo@uwaterloo.ca


clear all; close all; clc;
PoseInitial=[0,0,0]';%initial Pose
dt=0.1;%time interval
FinalTime=20;%final Time in sec
MeasurementNoise=[0.001 0 0;0 0.001 0;0 0 0.0001];
MotionNoise=[0.02 0;0 0.02];
MapSize=20;% No of features in Map
ParticleSize=100;%No of particles
[pose,map,y,muFeat,particleSet,t,meas_ind,newfeature,centroid_particles]=fastSLAM_func(PoseInitial,MotionNoise,MeasurementNoise,MapSize,FinalTime,dt,ParticleSize);