function [ ] = erasePath( pathLineObjects )
%ERASEPATH Erases already drawn path line objects
%   Detailed explanation goes here
for i = 1:length(pathLineObjects)
    delete(pathLineObjects(i));
end

