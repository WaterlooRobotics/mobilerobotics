%% Prediction Step of the observed features.
% INPUTS:(in order of call)
% -------
% Mean of the Observed Features
% Robot's Current Pose
% OUTPUTS:(in order of call)
% --------
% predicted observation matrix
% jacobian of the measurement wrt features
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [H_feat,pred_obs]=predict_features(muFeatOld,rob_pose)
       dx = muFeatOld(1,1)-rob_pose(1,1);
       dy = muFeatOld(2,1)-rob_pose(2,1);
       rp = sqrt((dx)^2+(dy)^2);
       pred_obs=[rp; mod(atan2(dy,dx)-rob_pose(3,1)+pi,2*pi)-pi];
       H_feat = [ dx/rp dy/rp; -dy/rp^2 dx/rp^2];% Calculate Jacobian wrt features
    end