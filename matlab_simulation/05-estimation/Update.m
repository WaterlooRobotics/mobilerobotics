function [prediction,model,sigUpdate] = Update(n,A,B,u,dt,lambda,S_u,mu,type)
%Updates the preidction model or measurement model with the weighted sigma
%points
%n: number of states, A: state matrix,B: input matrix, u: inputs, lambda:
%weighting, S_u: covariance updated
%type = 1 for prediction update, type = 2 for measurement update
    sigUpdate = sqrtm((n+lambda)*S_u);
    prediction(:,1) = mu;
    if type == 1
        model(:,1) = A*prediction(:,1) + B*u*dt; %need to change per example
    elseif type == 2
        model(:,1) = sqrt(prediction(1,1)^2+prediction(2,1)^2); %need to change per example
    end
end

