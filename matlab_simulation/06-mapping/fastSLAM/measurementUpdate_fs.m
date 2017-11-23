%% Measurement Model
% INPUTS:(in order of call)
% -------
% Environment Map
% Robot's Current Pose
% Measurement Noise
% OUTPUTS:(in order of call)
% --------
% Measurement Matrix at the current time step
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Measurement Model
    function obsFeat=measurementUpdate_fs(map,poseRobot,MeasNoise)
     [QiE, Qie] = eig(MeasNoise);
     m = length(MeasNoise(:,1)); % Number of measurements per feature 
     delta = QiE*sqrt(Qie)*randn(m,1);
     obsFeat=[sqrt((map(1,1)-poseRobot(1,1))^2 + (map(2,1)-poseRobot(2,1))^2);
                mod(atan2(map(2,1)-poseRobot(2,1),map(1,1)-poseRobot(1,1))-poseRobot(3,1)+pi,2*pi)-pi] + delta;

    end