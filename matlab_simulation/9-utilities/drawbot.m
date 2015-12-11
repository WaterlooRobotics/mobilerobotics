function drawbot(x,y,h,scale,fig)
% function drawbot(x,y,h,scale,fig)
% This function plots a two wheeled robot at position x,y heading h and size scale on
% figure number fig.
% The default x,y = (0,0), h = 0 points to the right, and scale=1 plots a
% robot with a body of radius of 1 units.

% Make a circle for the body
t=0:0.01:2*pi;
len = length(t);
bx = sin(t);
by = cos(t);

% Wheel locations on body
wh1 = round(len/4);
wh2 = round(3*len/4);

% Define the wheels
wwidth= 0.2;
wheight = 0.4;
w = [0 -wheight; wwidth -wheight; wwidth wheight; 0 wheight; 0 0];

% Body top
top = round(len/2);

% Top pointer
pwidth = 0.1;
pheight = 0.2;
tp = [pwidth/2 0; 0 -pheight; -pwidth/2 0; pwidth/2 0];

% Robot outline
bot = [bx(1:wh1)' by(1:wh1)';
    bx(wh1)+w(:,1) by(wh1)+w(:,2);
    bx(wh1:wh2)' by(wh1:wh2)';
    bx(wh2)-w(:,1) by(wh2)-w(:,2);
    bx(wh2:end)' by(wh2:end)'];

point = [bx(top)+tp(:,1) by(top)+tp(:,2)];

%Size scaling
bot = scale*bot;
point = scale*point;

% Rotation matrix
R = rot(h+pi/2,3);
R = R(1:2,1:2); 
bot = (R'*bot')';
point = (R'*point')';

% Centres
bot(:,1) = bot(:,1)+x;
bot(:,2) = bot(:,2)+y;
point(:,1) = point(:,1)+x;
point(:,2) = point(:,2)+y;

% Plot
figure(fig); hold on;
plot(bot(:,1), bot(:,2), 'b');
plot(point(:,1), point(:,2), 'r');
axis equal
end