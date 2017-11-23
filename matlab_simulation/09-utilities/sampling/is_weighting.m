function [wx] = is_weighting(M, x, gx, fx, xP)
    pgx = zeros(1, M);
    pfx = zeros(1, M);
    wx = zeros(1, M);

    for m = 1:M
        pgx(m) = gx(find(x >= xP(m), 1));
        pfx(m) = fx(find(x >= xP(m), 1));
        wx(m) = pfx(m) / pgx(m);
    end
end