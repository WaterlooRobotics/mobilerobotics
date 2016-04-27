function  Animation_GIF( filename,i,h )
% This function generates animation file of the simulation and saves it to a .gif file. 
% h: is the figure handle
% i: is the iteration index
% filename: filename.gif

    getframe(h);
    
    
    % saving animation file in .gif file
      im = frame2im(getframe(1));
      [imind,cm] = rgb2ind(im,256);
      if i == 1;
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
      else
          imwrite(imind,cm,filename,'gif','WriteMode','append');
      end
      
end

