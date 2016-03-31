%% Simple Bayesian Filtering Example
% Rickey Wang
% A door opening robot with a very noisy sensor tries to open a door

% This example uses the MATLAB command window. 
% See Thrun's textboook on Bayes Filters for explanation
clear; clc; 

%% Parameters
% Motion Model
%          Xt = open   closed
%                +-    -+  Xt-1 =
% ut = none -->  | 1  0 |  Open
%                | 0  1 |  Closed
%                +-    -+
%                +-        -+  
% ut = open -->  | 1    0   | 
%                | 0.8  0.2 |  
%                +-        -+
motion_m = zeros(2,2,2);
motion_m(:,:,1) = [1 0; 0 1];
motion_m(:,:,2) = [1 0; 0.8 0.2];
% Measurement Model
%  measurements = {sens_open,sens_closed}
%  p(y|x)
%             x = open  closed
%                +-        -+ y =
%                | 0.6  0.2 | sens_open
%                | 0.4  0.8 | sens_closed
%                +-        -+
meas_m = [0.6 0.2; 0.4, 0.8];
% State Vector (boolean door open or closed)
X = [];
% Input vector
u = [];
% Measurement vector
y = [];

%% Initial Set-up
% At beginning, assume the door is equally likely to be open or closed
% This forms our initial prior belief
bel_open = [];
bel_open = [bel_open; 0.5]; % Concatenate

%% Running the model
while(true)
    prompt = '0: none, 1: open   u = ';
    in = input(prompt);
    while ~(~isempty(in) ...
            && isnumeric(in) ...
            && isreal(in) ...
            && isfinite(in) ...
            && (in == fix(in)) ...
            && (in >= 0) ...
            && (in <=1))
        in = input(prompt);
    end
    
    u = [u; in];
    prompt = '0: sens_open, 1: sens_closed   y = ';
    in = input(prompt);
    while ~(~isempty(in) ...
        && isnumeric(in) ...
        && isreal(in) ...
        && isfinite(in) ...
        && (in == fix(in)) ...
        && (in >= 0) ...
        && (in <=1))
    in = input(prompt);
    end
    y = [y; in];
    
    bel_open_pri = [bel_open(end), 1-bel_open(end)];
    % The biggest challenge is figuring out what to put here:
    bf_open = bayes_filter(motion_m(:,:,u(end)+1)', meas_m(y(end)+1,:)', bel_open_pri');
    bel_open = [bel_open; 1/(sum(bf_open(:,2)))*bf_open(1,2)];
    
    %prompt = sprintf('bel(open)=%f',bel_open(end));
    %disp(prompt)
    
    fprintf('%3s %3s %10s\r\n','u','y','bel_open');
    fprintf('%3i %3i %10.5f\r\n',u,y,bel_open(2:end));
end



