function circle(fig,pos,r)
% circle(fig, pos, r)
% args: figure number, position in x,y, radius r
figure(fig);
rectangle('Position',[pos(1)-r,pos(2)-r,2*r,2*r], 'Curvature', [1 1], 'LineWidth',1, 'EdgeColor','b');