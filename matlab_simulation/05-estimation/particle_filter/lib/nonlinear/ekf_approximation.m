function [muLN, SLN, yLN, pyLN] = ekf_approximation(L, S, mu)
    G = 1 / ((mu + 1 / 2)^2 + 1);
    muLN = atan(mu + 0.5);
    SLN = G * S * G';
    yLN = [muLN-L*sqrt(SLN):0.01:muLN+L*sqrt(SLN)];
    pyLN = normpdf(yLN, muLN, sqrt(SLN));
end

