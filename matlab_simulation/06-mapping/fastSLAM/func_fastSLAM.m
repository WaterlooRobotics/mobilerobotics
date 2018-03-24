function [Pnew, featuresSeen] = func_fastSLAM( P, z, u, R, Q, featureState, dt )
    %% fastSLAM( P, z, u, R, Q, dt )
    % perform one iteration of the fastSLAM algorithm.
    %
    % P  -- particle vector - each particle has the following features:
    %        -- mu -- current estimated position (should be a distribution...)
    %        -- f_mu -- matrix of all estimated feature positions
    %        -- f_cov -- matrix of all feature covariances
    % z  -- measurement info for available features (range, bearing, id)
    % u  -- motion input
    % R  -- motion noise
    % Q  -- measurement noise
    % featureState -- a vector of the features that have been seen or not
    %                 (0 == unseen)
    % rMax -- max
    % dt -- time interval of motion
    %

    countParticles = size(P,2);
    w = zeros(countParticles,1);

    featuresSeen = featureState;

    [eV,ev] = eig(R);
    noiseFactor = eV * sqrt(ev);
    
    for p = 1:countParticles

        % move the particle forward in time
        P(p).mu = applyMotionModel( P(p).mu, u, noiseFactor, dt );

        countFeatures = size(z,2);
        if countFeatures == 0
            % no judgement info -- use the last weight
            w(p) = P(p).w;
        else
            % Allocate enough space to store the calculated co-variance, as well
            % as both the measured and predicted measurements
            zMeas = zeros(2 * size(z,2), 1);
            zPred = zeros(2 * size(z,2), 1);
            multiQm = zeros(2 * size(z,2), 2 * size(z,2));

            for f = 1:countFeatures

                % for each feature, perform an EKF update
                featureIndex = z(3,f);

                covStart = (featureIndex - 1) * 2 + 1;
                covEnd = covStart + 1;
                
                if featuresSeen( featureIndex ) == 0
                    % need an initial value
                    P(p).f_mu(:,featureIndex) = locateFeature( P(p).mu, z(1:2,f));
                    [zp, H] = calculateFeatureMeasurement( P(p).f_mu(:,featureIndex), P(p).mu );
                    H_inv = inv(H);
                    P(p).f_cov(:,covStart:covEnd) = H_inv * Q * H_inv';
                else 
                    [zp, H] = calculateFeatureMeasurement( P(p).f_mu(:,featureIndex), P(p).mu );
                end

                cov = P(p).f_cov(:,covStart:covEnd);
                Qm = H*cov*H'+Q;
                K = cov * H' * inv(Qm);
                P(p).f_mu(:,featureIndex) = P(p).f_mu(:,featureIndex) + K*(z(1:2,f) - zp);
                P(p).f_cov(:,covStart:covEnd) = (eye(2)-K*H)*cov;

                % Store the co-variance calculated, and the two related values of Z.  They'll be used
                % later to calculate the overall weight for this particle.
                zStart = 2 * (f-1) + 1;
                zEnd = zStart + 1;
                multiQm( zStart:zEnd, zStart:zEnd ) = round( Qm, 6 );  % prevent cholcov error
                zMeas( zStart:zEnd, 1 ) = z(1:2,f); 
                zPred( zStart:zEnd, 1 ) = zp(1:2,1); 
            end

            % if we saw anything, recalculate the final weight for the 
            % particle
            P(p).w = max( 0.00001, mvnpdf(zPred,zMeas,multiQm));
            w(p) = P(p).w;
        end
    
    end

    if exist( 'z', 'var' ) && size(z,1) == 3
        % mark features as seen
        featuresSeen(z(3,:)) = 1;
    end


    % draw a new set of particles
    W = cumsum(w);
    % Pnew = zeros( ... )  % preallocate new structure space
    for i = 1:countParticles
        seed = W(end) * rand(1);
        cur = find(W > seed,1);
        Pnew(i).mu = P(cur).mu;
        Pnew(i).f_mu = P(cur).f_mu;
        Pnew(i).f_cov = P(cur).f_cov;
        Pnew(i).w = P(cur).w;
    end

end
