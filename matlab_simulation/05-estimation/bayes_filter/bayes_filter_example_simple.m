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
    u = [u; in];
    prompt = '0: sens_open, 1: sens_closed   y = ';
    in = input(prompt);
    y = [y; in];
    
    bel_open_pri = [bel_open(end); 1-bel_open(end)];
%     bf_open = bayes_f(motion_m(:,:,u(end)+1), meas_m, bel_open_pri)
    
    prob_motion = motion_m(:,:,u(end)+1);
    prob_meas = meas_m;
    pred_pri = diag(bel_open_pri);
    
    % Prediction update
    pred_upd = prob_motion * pred_pri;
    % Measurement update
    meas_upd = prob_meas.*pred_upd;
    % Normalize
    meas_upd = meas_upd/norm(meas_upd);
    bf_open = [pred_upd meas_upd]
    
  
    bel_open = [bel_open; bf_open(y(end)+1,3)];
end



