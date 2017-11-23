function [xp,x] = SelectSigPoints(n,A,B,u,dt,mu,sig,xp,x,type)
%This function selects the sigma points used for both prediction and
%measurement
%n: number of states, A: state matrix, B: input matrix, u: inputs, xp:
%position prediction, x: position model
%type = 1 for position, type = 2 for measurement
for i=1:n
    % Sigma points prior to propagation
    xp(:,i+1) = mu + sig(:,i);
    xp(:,n+i+1) = mu - sig(:,i);
    if type == 1
        % Sigma points after propagation
        x(:,i+1) = A*xp(:,i+1) + B*u*dt;
        x(:,n+i+1) = A*xp(:,n+i+1) + B*u*dt;
    elseif type == 2
        % Measurement model applied to sigma points
        x(:,i+1) = sqrt(xp(1,i+1)^2 + xp(2,i+1)^2);
        x(:,n+i+1) = sqrt(xp(1,n+i+1)^2 + xp(2,n+i+1)^2);
    end
end

