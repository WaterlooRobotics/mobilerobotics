function [ measurement_model ] = simpleRobotMeasurementModel( feature, MEASUREMENT_TYPE )
% Outputs measurement model from Mapping I slide 16 

switch(MEASUREMENT_TYPE) 
    case 1
        measurement_model = @ (mup,noInput) sqrt((feature(1)-mup(1))^2 + (feature(2)-mup(2))^2);
    case 2
        measurement_model = @ (mup,noInput) ...
            (atan2(feature(2)-mup(2),...
            feature(1)-mup(1)) - mup(3));
    case 3
        measurement_model = @ (mup,noInput) ...
            [sqrt((feature(1)-mup(1))^2 + (feature(2)-mup(2))^2);
            (atan2(feature(2)-mup(2),feature(1)-mup(1)) - mup(3))];
end
end

