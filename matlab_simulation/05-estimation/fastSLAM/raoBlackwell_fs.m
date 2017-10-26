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
    function [particles_new]=raoBlackwell_fs(totalParticles,particles_old)
        W(1)=0;
        for i=2:totalParticles
        W(i) = W(i-1)+particles_old(i).weight;% Form the PDF and calculate the final cumulitive sum of all weights.
        end
        % Resample and copy all data to new particle set
        for particle=1:totalParticles
            seed = W(end)*rand(1);
            cur = find(W>seed,1);
            particles_new(particle).pose = particles_old(cur).pose;
            particles_new(particle).meanFeat = particles_old(cur).meanFeat;
            particles_new(particle).covFeat = particles_old(cur).covFeat;
        end
        for i=1:totalParticles
        particles_new(i).weight=particles_old(i).weight;
        end
        
        
    end