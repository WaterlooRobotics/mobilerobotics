%% A FUNCTION TO ADD UNOBSERVED FEATURES TO THE PARTICLES
% INPUTS:(in order of call)
% -------
% The prior mean of the feature
% The observation that led to the feature's discovery
% Uncertainty in that Measurement
% Jacobian Matrix of the Observation with respect to the current particle
% OUTPUTS:(in order of call)
% --------
% Predicted Mean of newly observed feature
% Predicted Covariance of the newly Observed Feature
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [meanFeatP,covaFeatP]=addFeature(prior,obsFeat,H_feat,measUncertainty)
        meanFeatP(1,1) = prior(1,1)+obsFeat(1,1)*cos(obsFeat(2,1)+prior(3,1));
        meanFeatP(2,1) = prior(2,1)+obsFeat(1,1)*sin(obsFeat(2,1)+prior(3,1));
        covaFeatP= H_feat\measUncertainty*inv(H_feat)';%calculate cov
    end