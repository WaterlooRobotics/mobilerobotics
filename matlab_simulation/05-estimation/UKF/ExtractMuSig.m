function [mu,sig] = ExtractMuSig(n,mu,w_m,model,sig,w_c,error)
%Extracts mean and covariance from weighted points
%n: number of states, w_m: weighted mean, w_c: weighted covariance

    for i=1:2*n+1
        %Adds weighted sigma points to predicted mean
        mu = mu +w_m(i)*model(:,i);
    end
    for j=1:2*n+1
        %Adds weighted covariane to predicted covariance
        sig = sig + w_c(j)*((model(:,j)-mu)*(model(:,j)-mu)');
    end
    sig = sig + error;
end

