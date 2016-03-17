%% FUNC to search of features in Field of VIEW
% INPUTS:(in order of call)
% -------
% No of features
% Map of the Environment
% Robot's pose at the current time step
% Maximum Range
% Maximum FOV
% OUTPUTS:(in order of call)
% --------
% A vector containing all the measured indexs
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Feature Search
    function meas_ind=featureSearch_fs(noFeatures,map,robot_pose,rmax,thmax)
        meas_ind = [];
        for i=1:noFeatures
                if(inview(map(:,i),robot_pose,rmax,thmax))
                    meas_ind=[meas_ind,i];%concatenate the indices
                end
        end
            function yes = inview(f,x, rmax, thmax)
            % Checks for features currently in robot's view
            yes = 0;
            dx = f(1)-x(1);
            dy = f(2)-x(2);
            r = sqrt(dx^2+dy^2);
            th = mod(atan2(dy,dx)-x(3),2*pi);
            if (th > pi)
                th = th-2*pi;
            end
            if ((r<rmax) && (abs(th)<thmax))
                yes = 1;
            end
        end
    end