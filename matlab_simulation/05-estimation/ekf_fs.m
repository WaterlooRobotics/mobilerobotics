function [meanFeatP,covaFeatP,Hmu,Ht]=ekf_fs(predFeatPrior,obsFeat,meanFeat,covaFeat,measUncertainty,checkNewFeat)
  if (checkNewFeat == 1) %nonobserved features are all 1;old features are 0;
                % If new feature, initialize the features.
                meanFeatP(1,1) = predFeatPrior(1,1)+obsFeat(1,1)*cos(obsFeat(2,1)+predFeatPrior(3,1));
                meanFeatP(2,1) = predFeatPrior(2,1)+obsFeat(1,1)*sin(obsFeat(2,1)+predFeatPrior(3,1));
                % Predicted range
                dx = meanFeatP(1,1)-predFeatPrior(1,1);
                dy = meanFeatP(2,1)-predFeatPrior(2,1);
                rp = sqrt((dx)^2+(dy)^2);
                Ht = [ dx/rp dy/rp; -dy/rp^2 dx/rp^2];% Calculate Jacobian
                covaFeatP= Ht\measUncertainty*inv(Ht)';%calculate cov
                
            else
                % IF old features, update the feature mean and covariance
                dx = meanFeat(1,1)-predFeatPrior(1,1);
                dy = meanFeat(2,1)-predFeatPrior(2,1);
                rp = sqrt((dx)^2+(dy)^2);
                Ht = [ dx/rp dy/rp; -dy/rp^2 dx/rp^2];% Calculate Jacobian
                %run EKF steps
                I = obsFeat(:,1)-[rp; mod(atan2(dy,dx)-predFeatPrior(3,1)+pi,2*pi)-pi];
                Q = Ht*covaFeat*Ht' + measUncertainty;
                K = covaFeat*Ht'/Q;
                meanFeatP = meanFeat + K*I;
                covaFeatP = (eye(2)-K*Ht)*covaFeat;

            end
        Hmu=[rp; mod(atan2(dy,dx)-predFeatPrior(3,1)+pi,2*pi)-pi];%used later for calculation of weights
end