function [new_path] = delete_abnormal_w(path)
new_path = [];
k = size(path,2);
for i = 1:k-3
    if (abs(path(i+1)-path(i)) > 0.001)
        path(i+1) = 0.5*(path(i+3)+path(i));
    end
    new_path = [new_path,path(i)];
end
new_path = [new_path,path(k-2),path(k-1),path(k)];
end