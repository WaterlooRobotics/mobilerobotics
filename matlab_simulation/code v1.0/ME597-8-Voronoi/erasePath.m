function [ ] = erasePath( pathLineObjects )
%ERASEPATH Erases already drawn path line objects
for i = 1:length(pathLineObjects)
    delete(pathLineObjects(i));
end

