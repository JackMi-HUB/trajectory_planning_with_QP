function [new_path] = ResamplePath(path,step)%对A*_path重新采样，path为n*3矩阵，step为分数:(0,1]
k=size(path,1);%计算A*_path的点数，目前为126
new_path=[];
step=1/step;%step为采集step，分母越大，点数越多；采样点数很重要：关系到QP求解器是否有解

for i=1:step:k
    j=i;
    if(j>=k-2)
        j = k-2;
    end
    %反正切用的是atan2；返回[-pi,pi]；而atan，返回的是[-pi/2,pi/2]
    theta = atan2((path(j+2,2)-path(j,2)),(path(j+2,1)-path(j,1)));%计算反正切角的step暂时用2；后面考虑再改
    new_path=[new_path;path(i,1:2),theta];
end

