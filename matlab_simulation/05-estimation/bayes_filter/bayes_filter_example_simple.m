%% Simple Bayesian Filtering Example
% Rickey Wang
% A door opening robot with a very noisy sensor tries to open the door
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
%         y = sens_open sens_closed
%                +-        -+ x =
%                | 0.6  0.2 | open
%                | 0.4  0.8 | closed
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
bel_open_pri = [];
bel_closed_pri = [];

bel_open_pri = [bel_open_pri; 0.5]; % Concatenate
bel_closed_pri = [bel_closed_pri; 0.5];
bel_open = bel_open_pri;
bel_closed = bel_closed_pri;

%% Running the model
while(true)
    prompt = '0: none, 1: open   u = ';
    in = input(prompt);
    u = [u; in];
    prompt = '0: sens_open, 1: sens_closed   y = ';
    in = input(prompt);
    y = [y; in];
    
    bf_open = bayes_filter(motion_m, meas_m, bel_open(end))
    bf_closed = bayes_filter(motion_m, meas_m, bel_closed(end))
    
    bel_open = [bel_open; bf_open(:,2)];
    bel_closed = [bel_closed; bf_closed(:,2)];
end



