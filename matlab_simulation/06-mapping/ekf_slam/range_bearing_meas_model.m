function y = range_bearing_meas_model(xr,map)
% measurement model of range and bearing
% can be found P.12 of Mapping II slides
    y = [sqrt((map(1)-xr(1))^2 + (map(2)-xr(2))^2);
         atan2(map(2)-xr(2),map(1)-xr(1))-xr(3)];
end
