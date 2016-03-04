function [C,D,Q] = measurement_model(example)

% This function provides the C and Q matrices for the measurement model

if example == 1
    C = 1;
    Q = 4;
    D = 0;
end

if example == 2
    C = zeros(2,4);
    C(1,1) = 1;
    C(2,3) = 1;
    D = zeros(2,2);
    Q = [.04 -0.01; -0.01 .01];
    %Q = [.004 -0.001; -0.001 .001];
    %Q = [.0004 -0.0001; -0.0001 .0001];
end

if example ==3
    Cp = eye(4);
    Cv = zeros(2,4);
    Cv(1,2) = 1;
    Cv(2,4) = 1;
    D = zeros(2,2);
    Qp = [.004 0 -0.001 0 ; 0 0.1 0 -0.01;  -0.001 0 .001 0; 0 -0.01 0 0.05];
    Qv = [.1 -0.01; -0.01 .05];
end