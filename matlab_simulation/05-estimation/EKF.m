function [muP,mu,sig,K] = EKF(A,B,C,D,dt,y,u,mu,sig,positionE,measureE,n)
%This function runs the Extended Kalman Filter with the following inputs:
%A: state matrix, B: input matrix, C: measurement matrix, D: input
%measurement matrix, y: measurment model, u: state inputs, n: number of
%states

%Prediction Update, muP equation based on example
muP = A*mu + B*u*dt;   %The addition of the input matrix requires the B matrix to be used
sigP = A*sig*A' + positionE;

%Jacobian matrix to linearize, change this equation for each example
Ht = [(muP(1))/(sqrt(muP(1)^2 + muP(2)^2)) (muP(2))/(sqrt(muP(1)^2 + muP(2)^2))];

% Measurement update
K = sigP*Ht'*inv(Ht*sigP*Ht'+ measureE);
mu = muP + K*(y - sqrt(muP(1)^2 + muP(2)^2)); %change h(mu) equations based on example
sig = (eye(n)-K*Ht)*sigP;
end

