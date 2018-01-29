%% Test drawquadrotor.m

   load('quad_starmac.mat')
   figure(1); clf; hold on;
   drawquadrotor(qm_fnum, qm_xyz, 1, 0, 0, 0, 0, 0,[0.6 0.1 0.1]);
   plot3(0,0,0, 'go', 'MarkerSize', 6, 'LineWidth', 2); % Draw object
   xlabel('X')
   ylabel('Y')
   zlabel('Z')
   axis([-5 5 -5 5 -5 5])
   view(0, 90);
   grid on