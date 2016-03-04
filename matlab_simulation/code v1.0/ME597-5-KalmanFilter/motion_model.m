function [Ad,Bd,R,n] = motion_model(example,dt)

% This function provides the A,B and R matrices for motion model

% For exmaple 1
if example == 1
    Ad = 0.8;
    Bd = 3;
    R = 2;
    n = length(A(1,:));
end

% For exmaple 2
if example == 2
    b = 1;
    m = 2;
    A = [ 0 1 0 0; 0 -b/m 0 0; 0 0 0 1; 0 0 0 -b/m];
    B = [0 0 ;1/m 0; 0 0; 0 1/m];
    C = [];
    D = [];
    R = [.01 0 0 0; 0 .01 0 0; 0 0 .01 0; 0 0 0 .01];
    %R = [.001 0 0 0; 0 .001 0 0; 0 0 .001 0; 0 0 0 .001];
    %R = [.0001 0 0 0; 0 .0001 0 0; 0 0 .0001 0; 0 0 0 .0001];
    
    % Form continuous system
    sysc=ss(A,B,C,D);

    % zoh discretization
    sysd = c2d(sysc,dt,'zoh');
    Ad = sysd.A;
    Bd = sysd.B;
    
    n = length(A(1,:));
end

% For exmaple 3
if example == 3
    b = 1;
    m = 2;
    A = [ 0 1 0 0; 0 -b/m 0 0; 0 0 0 1; 0 0 0 -b/m];
    B = [0 0 ;1/m 0; 0 0; 0 1/m];
    R = [.0001 0 0 0; 0 .0001 0 0; 0 0 .0001 0; 0 0 0 .0001];
    
    n = length(A(1,:));
    
    % Form continuous system
    sysc=ss(A,B,C,D);

    % zoh discretization
    sysd = c2d(sysc,dt,'zoh');
    Ad = sysd.A;
    Bd = sysd.B;
end