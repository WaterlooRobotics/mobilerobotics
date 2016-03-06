clear all; close all; clc;
PoseInitial=[0,0,0]';
dt=0.1;
FinalTime=20;%final Time in sec
MeasurementNoise=[0.001 0 0;0 0.001 0;0 0 0.0001];
MotionNoise=[0.02 0;0 0.02];
MapSize=20;
ParticleSize=100;
fastSLAM_func_p(PoseInitial,MotionNoise,MeasurementNoise,MapSize,FinalTime,dt,ParticleSize);