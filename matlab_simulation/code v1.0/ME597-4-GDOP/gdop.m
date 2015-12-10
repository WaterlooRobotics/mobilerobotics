%% GDOP example
clear; clc;

% Satellite positions in lla (deg, deg, mA)
sats = [0 -80 20000000;
        90 -20 20000000;
        45 -140 20000000;
        60 -60 22000000;
        40 -80 21000000;
        50 -40 19000000;
        70 -100 21000000;
        30 -120 23000000;
        15  -100 20000000;]

% Current receiver position    
cur = [43 -80 337]; 

% Number of available satellites
num = 4;

for i=1:num
    % Convert satellite position to ENU 
    satsenu(i,:) = Wgslla2enu(sats(i,1), sats(i,2), sats(i,3), cur(1), cur(2), cur(3));
    % Find range to satellite from receiver
    r = sqrt(satsenu(i,1)^2+satsenu(i,2)^2+satsenu(i,3)^2);
    % Calculate A matrix
    A(i,:) = [(satsenu(i,1)-cur(1))/r (satsenu(i,2)-cur(2))/r (satsenu(i,3)-cur(3))/r 1];
end

% Find Q matrix
Q = inv(A'*A);
% Calculate GDOP
GDOP = sqrt(trace(Q));

% Plot results
figure(1); clf; hold on;
plot3(cur(1),cur(2),cur(3), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
for i=1:num
    plot3(satsenu(i,1),satsenu(i,2),satsenu(i,3), 'ro', 'MarkerSize', 10,'LineWidth', 2);
end
grid on;