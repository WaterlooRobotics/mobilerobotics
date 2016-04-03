% A simple exmaple of using Animation_GIF function to genrate animation file and
% save the result as .gif file

clc;
clear;
syms t;
filename='sample.gif';

dt=0.1;
Time=0:dt:10;
x=t;            % random function
y=sin(t);       % random function

for i=1:numel(Time)
    clf
    ti=Time(i);
plot(subs(x,t,ti),subs(y,t,ti),'ro','MarkerEdgeColor','k',...
                'MarkerFaceColor',[.49 1 .63],...
                'MarkerSize',10);
title('This result will be saved as an .gif animation file')
axis([-2 10 -4 4 ]);
grid on;
Animation_GIF( filename,i,gcf )
end