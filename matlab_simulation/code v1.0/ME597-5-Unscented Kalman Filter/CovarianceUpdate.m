function [K_u,mu_u,S_u] = CovarianceUpdate(t,Sxy_u,Sy_u,Sp_u,mup_u,y,y_u)
%Updates the mean and covaraince with the cross covariance values
    K_u = Sxy_u*inv(Sy_u);
    mu_u = mup_u + K_u*(y(:,t)-y_u);
    S_u = Sp_u - K_u*Sy_u*K_u';
end
