function [xp,x] = SelectSigPoints(n,A,B,u,mu,sig,xp,x,type)
%This function selects the sigma points used for both prediction and
%measurement

%type = 1 for position, type = 2 for measurement
    if type == 1
        for i=1:n
            % Sigma points prior to propagation
            xp(:,i+1) = mu + sig(:,i);
            xp(:,n+i+1) = mu - sig(:,i);
            % Sigma points after propagation
        	x(:,i+1) = A*xp(:,i+1) + B*u;
            x(:,n+i+1) = A*xp(:,n+i+1) + B*u;
        end
    elseif type == 2
        for i=1:n
            % Sigma points prior to measurement
            xp(:,i+1) = mu + sig(i,:)';
            xp(:,n+i+1) = mu - sig(i,:)';
            % Measurement model applied to sigma points
            x(:,i+1) = sqrt(xp(1,i+1)^2 + xp(2,i+1)^2);
            x(:,n+i+1) = sqrt(xp(1,n+i+1)^2 + xp(2,n+i+1)^2);
        end

end
