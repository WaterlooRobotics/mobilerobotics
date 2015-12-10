function drawbox(x,y,h,scale,fig)
% function drawbox(x,y,h,scale,fig)
% This function plots a box at position x,y heading h and size scale on
% figure number fig.


% Car outline
box = [-1 -0.5; 1 -0.5; 1 0.5; -1 0.5; -1 -0.5];

%Size scaling
box = scale*box;

% Rotation matrix
R = [cos(h) -sin(h); sin(h) cos(h)];
box = (R*box')';

% Centre
box(:,1) = box(:,1)+x;
box(:,2) = box(:,2)+y;

% Plot
figure(fig);
plot(box(:,1), box(:,2), 'b','LineWidth', 2);
axis equal
end