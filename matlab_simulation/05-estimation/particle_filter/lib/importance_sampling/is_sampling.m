function [indP, xP, xPind] = is_sampling(M, Gx, x)
    indP = zeros(1, M);
    xP = zeros(1, M);
    
    draw = rand(1, M);
    for m = 1:M
        indP(m) = find(Gx >= draw(m), 1);
        xP(m) = x(indP(m));
    end
    xPind = hist(indP, M);
end

