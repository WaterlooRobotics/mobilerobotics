function [K_u,mu_u,S_u] = CovarianceUpdate(t,Sxy_u,Sy_u,Sp_u,mup_u,y,y_u)
%This function updates the covariance matrix for UKF
%t: current time step, Sxy_u: cross-covariance prediction, Sy_u:
%extracted covariance, Sp_u: covariance prediction updated, mup_u: mean
%prediction updated, y: measurment model, y_u: measurement model updated
    K_u = Sxy_u*inv(Sy_u);
    mu_u = mup_u + K_u*(y(:,t)-y_u);
    S_u = Sp_u - K_u*Sy_u*K_u';
end

