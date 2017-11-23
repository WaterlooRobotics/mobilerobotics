function [muParticle, SParticle, X] = pf_nonlinear(t, I, Ad, X, R, Q, y)
    [RE, Re] = eig(R);
    n = length(Ad(1,:));
    
    %  sample
    for i = 1:I
        ep = RE * sqrt(Re) * randn(n, 1);
        Xp(:, i) = Ad * X(:, i) + ep;
        w(i) = max(0.00001, normpdf(y(:, t), sqrt(Xp(1, i)^2 + Xp(3, i)^2), sqrt(Q)));
    end
    
    %  re-sample
    W = cumsum(w);
    for j = 1:I
         seed = W(end) * rand(1);
         X(:,j) = Xp(:, find(W > seed, 1));
    end
    muParticle = mean(X');
    SParticle = cov(X');
end

