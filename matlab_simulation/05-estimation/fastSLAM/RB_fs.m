%% Rao Blackwellization Function for the Particle FILTER
% INPUTS:(in order of call)
% -------
% total no of Particles
% Weights calculated from Importance Sampling
% Old particle Set
% Mean of the Particle Set
% Covariance of the Particle Set
% OUTPUTS:(in order of call)
% --------
% new Particle Set after
% Mean of the Particle set after resampling
% Covariance of the Resampled particle set
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Rao Blackwellization
    function [newParticleSet,muParticleNew,covParticleNew]=RB_fs(totalParticles,w_new,particleSetP,muFeatPred,covFeatPred)
        W = cumsum(w_new);% Form the PDF and calculate the final cumulitive sum of all weights.
        % Resample and copy all data to new particle set
        for particle=1:totalParticles
            seed = W(end)*rand(1);
            cur = find(W>seed,1);
            newParticleSet(:,particle) = particleSetP(:,cur);
            muParticleNew(:,:,particle) = muFeatPred(:,:,cur);
            covParticleNew(:,:,:,particle) = covFeatPred(:,:,:,cur);
        end
    end