function [C,D,Q,m] = measurement_model(example)

% Provides discretized C(measurement), D(feed-forward) and Q(disturbance) 
% matrices for the measurement model

% Inputs:
%       example   : Example under considertaion [1,2 or 3]

% Outputs:
%       C         : Measurement
%       D         : Feed-forward
%       Q         : Disturbance
%       m         : Measurement dimension

% Example 1: Temperature
if example == 1
    C = 1;
    D = 0;
    Q.Q = 4;
    Q.QR = 4;
    m = length(C(:,1));
end

% Example 2: 2D Omnidirectional AUV
if example == 2
    C = zeros(2,4);
    C(1,1) = 1;
    C(2,3) = 1;
    D = zeros(2,2);
    Q.QR = [.4 -0.1; -0.1 .1];
    Q.Q = [.04 -0.01; -0.01 .01];
    %Q.Q = [.004 -0.001; -0.001 .001];
    %Q.Q = [.0004 -0.0001; -0.0001 .0001];
    m = length(C(:,1));
end

% Eample 3: Multirate 2D Omnidirectional AUV. Provides position and
% velocity measurements
% Cp : Measures position and velocity
% Cv : Measures only velocity
% Qp : Disturbances in position and velocity
% Qv : Disturbances in velocity
if example ==3
    C.Cp = eye(4);
    C.Cv = zeros(2,4);
    C.Cv(1,2) = 1;
    C.Cv(2,4) = 1;
    m.mp = length(C.Cp(:,1));
    m.mv = length(C.Cv(:,1));
    D = zeros(2,2);
    Q.Qp = [.004 0 -0.001 0 ; 0 0.1 0 -0.01;  -0.001 0 .001 0; 0 -0.01 0 0.05];
    Q.Qv = [.1 -0.01; -0.01 .05];
end