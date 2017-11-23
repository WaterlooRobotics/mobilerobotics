%% A function for the correction step of the EKF
% NOTE: THE prediction step has already been called, Meas Jac Calculated
% and new feature initialized, if at all. This is the next correction step
% 
% INPUTS:(in order of call)
% -------
% Mean of all observed Features
% covariance of all Observed Features
% Jacobian of the Measurement with respect to the features
% Predicted Observation from the prediction step of EKF
% Measurement Uncertainty
% OUTPUTS:(in order of call)
% --------
% Preicted mean of The feature
% Predicted covariance of the feature 
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [meanFeatP,covaFeatP]=ekf_correct(muFeat,covFeat,obsFeat,H_feat,pred_obs,measUncertainty)
        I = obsFeat(:,1)-pred_obs;
        Q2 = H_feat*covFeat*H_feat' + measUncertainty;
        K = covFeat*H_feat'/Q2;
        meanFeatP = muFeat + K*I;%new mean
        covaFeatP = (eye(2)-K*H_feat)*covFeat;% New covariance
    end