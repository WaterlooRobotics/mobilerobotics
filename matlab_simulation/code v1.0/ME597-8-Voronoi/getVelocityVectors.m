function [ wheelVelocity ] = getVelocityVectors( path, V, l, r )
%GETVELOCITYVECTORS Obtains the wheel velocity vectors for an omniwheel robot.


i=1;
wheelVelocity = [];
while i<=(length(path)-1)
    worldVelocity = V(path(i+1),:) - V(path(i),:);
    wheelVelocity = [wheelVelocity; transpose(IKOmniDirectionRobot(worldVelocity, l, r))];
    i = i+1;
end;

end

