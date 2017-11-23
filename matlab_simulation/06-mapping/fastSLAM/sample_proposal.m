%% Sampling of the Proposal Distribution based on current measurement
%(unique to FAST SLAM2.0)
% INPUTS:(in order of call)
% -------
% Mean of the observed features
% Covariance of the observed features
% Robot's Current Pose
% Observations at the current timestep
% Measurement Noise
% Motion Noise
% OUTPUTS:(in order of call)
% --------
% Estimated Mean of the Pose of the robot represented by the curr particle
% Estimated Covariance of the pose of the robot of i-th particle
% Jacobian of the measurement wrt to robot's pose
% Jacobian of the measurement wrt to observed features
% Predicted Observation
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [mu_pose,cov_pose,H_pose,H_feat,pred_obs]=sample_proposal(muFeatOld,covFeatOld,rob_pose,act_obs,R,Q)
       dx = muFeatOld(1,1)-rob_pose(1,1);
       dy = muFeatOld(2,1)-rob_pose(2,1);
       rp = sqrt((dx)^2+(dy)^2);
       pred_obs=[rp; mod(atan2(dy,dx)-rob_pose(3,1)+pi,2*pi)-pi];
       H_feat = [ dx/rp dy/rp; -dy/rp^2 dx/rp^2];% Calculate Jacobian wrt features
       H_pose = [-dx/rp -dy/rp 0; -dy/rp^2 -dx/rp^2 -1];%Jacobian wrt Pose
       Q_mat=(R+H_feat*covFeatOld*H_feat');
       cov_pose=(H_pose'*((Q_mat)^-1)*H_pose+Q)^-1;
       mu_pose=cov_pose*H_pose'*(Q_mat^-1)*(act_obs-pred_obs)+rob_pose;
    end