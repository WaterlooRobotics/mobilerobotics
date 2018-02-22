function [Pnew, featuresSeen] = fSLAM( P, z, u, R, Q, featureState, dt )
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
    
    Rinv = inv(R);
    
    for p = 1:countParticles

        % move the particle forward in time
        mup = applyMotionModel( P(p).mu, u, noiseFactor, dt );

        countFeatures = size(z,2);
        if countFeatures == 0
            % no feature information -- use the original prediction
            P(p).mu = mup;
        else
            % Allocate enough space to store the calculated co-variance, as well
            % as both the measured and predicted measurements
            zMeas = zeros(2 * size(z,2), 1);
            zPred = zeros(2 * size(z,2), 1);
            multiQm = zeros(2 * size(z,2), 2 * size(z,2));

            mup_f = [0;0;0];
            for f = 1:countFeatures

                % for each feature, perform an EKF update
                featureIndex = z(3,f);

                covStart = (featureIndex - 1) * 2 + 1;
                covEnd = covStart + 1;

                if featuresSeen( featureIndex ) == 0
                    % need an initial value -- use the predicted position
                    P(p).f_mu(:,featureIndex) = locateFeature( mup, z(1:2,f));
                    [zp, Hm, Hx] = calculateMeasurementAndJacobians( P(p).f_mu(:,featureIndex), mup );
                    Hm_inv = inv(Hm);
                    P(p).f_cov(:,covStart:covEnd) = Hm_inv * Q * Hm_inv';
                else 
                    [zp, Hm, Hx] = calculateMeasurementAndJacobians( P(p).f_mu(:,featureIndex), P(p).mu );
                end

                cov = P(p).f_cov(:,covStart:covEnd);
                Qm = Hm*cov*Hm'+Q;   % Qt(k)
                Qm_inv = inv(Qm);

                % calculate the measurement error
                zErr = z(1:2,f) - zp;

                % calculate a possible belief for the particle's position
                covXt = inv(Hx' * Qm_inv * Hx + Rinv);
                mup_f = mup_f + covXt * Hx' * Qm_inv * zErr + mup;

                % update the Kalman gain and calculate the new feature
                % position and covariance.
                K = cov * Hm' * Qm_inv;
                P(p).f_mu(:,featureIndex) = P(p).f_mu(:,featureIndex) + K*zErr;
                P(p).f_cov(:,covStart:covEnd) = (eye(2)-K*Hm)*cov;

                % Store the co-variance caluculated, and the two related values of Z.  They'll be used
                % later to calculate the overall weight for this particle.
                qStart = size(Qm, 1) * (f - 1) + 1;
                qEnd = qStart + size(Qm,1) - 1;
                multiQm( qStart:qEnd, qStart:qEnd ) = round( Qm, 6 );  % prevent cholcov error

                zStart = 2 * (f-1) + 1;
                zEnd = zStart + 1;
                zMeas( zStart:zEnd, 1 ) = z(1:2,f); 
                zPred( zStart:zEnd, 1 ) = zp(1:2,1); 
            end
            
            % if we saw anything, recalculate the final weight for the
            % particle
            P(p).w = max( 0.00001, mvnpdf(zPred,zMeas,multiQm));
            
            % true belief is the average of all the guesses
            P(p).mu = mup_f / countFeatures;
        end
        w(p) = P(p).w;
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
        % Pnew(i) = P(cur);
        Pnew(i).mu = P(cur).mu;
        Pnew(i).f_mu = P(cur).f_mu;
        Pnew(i).f_cov = P(cur).f_cov;
        Pnew(i).w = P(cur).w;
    end

end
