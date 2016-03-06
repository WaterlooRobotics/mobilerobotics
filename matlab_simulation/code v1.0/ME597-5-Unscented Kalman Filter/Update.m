function [prediction,model,sigUpdate] = Update(n,A,B,u,lambda,S_u,mu,type)
%Updates the preidction model or measurement model with the weighted sigma
%points
%type = 1 for prediction update, type = 2 for measurement update
    if type == 1
        sigUpdate = sqrtm((n+lambda)*S_u);
        prediction(:,1) = mu;
        model(:,1) = A*prediction(:,1) + B*u; %need to change per example
    elseif type == 2
        sigUpdate = sqrtm((n+lambda)*S_u);
        prediction(:,1) = mu;
        model(:,1) = sqrt(prediction(1,1)^2+prediction(2,1)^2); %need to change per example
    end
end
