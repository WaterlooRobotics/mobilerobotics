function [xPnew, xPindnew] = is_resampling(M, xP, wx)
    indPnew = zeros(1, M);
    xPnew = zeros(1, M);

    WX = cumsum(wx);
    WX = WX / max(WX);

    draw = rand(1, M);
    for m = 1:M
        indPnew(m) = find(WX >= draw(m), 1);
        xPnew(m) = xP(indPnew(m));
    end
    xPindnew = hist(indPnew, M);
end

