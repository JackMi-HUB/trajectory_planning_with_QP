function [new_path] = delete_abnormal_v(path)
new_path = [];
k = size(path,2);
for i = 2:k-1
    if (path(i+1)/path(i) > 1.015)
        path(i) = 0.5*(path(i+1)+path(i-1));
    end
    new_path = [new_path,path(i)];
end
new_path = [path(1),new_path,path(k)];
end