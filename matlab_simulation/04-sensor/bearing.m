function b = bearing(mx, my, x1, x2, x3)
    b = mod(atan2(my - x2, mx - x1) - x3 + pi, 2 * pi) - pi;
end