%% Motion Model
% INPUTS:(in order of call)
% -------
% Robot's Old Pose at last time step
% Control Input
% Motion Noise
% Time interval
% OUTPUTS:(in order of call)
% --------
% New Pose of the Robot
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%% Motion Model
    function newPose=motionUpdate_fs(oldPose1,controlInput,MotionNoise,dt)
    [QE, Qe] = eig(MotionNoise);n = length(MotionNoise(:,1)); 
     e = QE*sqrt(Qe)*randn(n,1);
    % Update robot state
        newPose = [oldPose1(1,1)+controlInput(1,1)*cos(oldPose1(3,1))*dt;% This means U is just a transla1ion and rotation
                  oldPose1(2,1)+controlInput(1,1)*sin(oldPose1(3,1))*dt;
                  oldPose1(3,1)+controlInput(2,1)*dt] + e;
    end