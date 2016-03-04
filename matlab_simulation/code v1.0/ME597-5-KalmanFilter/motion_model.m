function [A,B,R] = motion_model(example)

% This function provides the A,B and R matrices for motion prediction

if example == 1
    A = 0.8;
    B = 3;
    R = 2;
end

if example == 2
    b = 1;
    m = 2;
    A = [ 0 1 0 0; 0 -b/m 0 0; 0 0 0 1; 0 0 0 -b/m];
    B = [0 0 ;1/m 0; 0 0; 0 1/m];
    R = [.01 0 0 0; 0 .01 0 0; 0 0 .01 0; 0 0 0 .01];
    %R = [.001 0 0 0; 0 .001 0 0; 0 0 .001 0; 0 0 0 .001];
    %R = [.0001 0 0 0; 0 .0001 0 0; 0 0 .0001 0; 0 0 0 .0001];
end

if example == 3
    b = 1;
    m = 2;
    A = [ 0 1 0 0; 0 -b/m 0 0; 0 0 0 1; 0 0 0 -b/m];
    B = [0 0 ;1/m 0; 0 0; 0 1/m];
    R = [.0001 0 0 0; 0 .0001 0 0; 0 0 .0001 0; 0 0 0 .0001];
end