function [BVr, BVf, xr, yr, xf, yf] = SpecifySafeCorridor(x, y, theta)%x,y,theta为A*的采样点集
global vehicle_geometrics_
%对小车简化为两个圆盘模型;原model给定的是四轮小车，这里我们做简化，采用双轮的，所以简化为一个圆盘（待简化）
xr = x + vehicle_geometrics_.r2x .* cos(theta);%Rear_disc的x&y坐标
yr = y + vehicle_geometrics_.r2x .* sin(theta);
% xf = x + vehicle_geometrics_.f2x .* cos(theta);%Front_disc的x&y坐标
% yf = y + vehicle_geometrics_.f2x .* sin(theta);
xf = x;
yf = y;
BVr = zeros(length(x),4); % xmin, xmax, ymin, ymax
BVf = BVr;%box_vertexs_front & box_vertexs_rear
for ii = 1 : length(xr)
    x = xr(ii); y = yr(ii);
    lb = GetBoxVertexes(x, y);
    if (~any(lb))
        counter = 0;
        is_lb_nonzero = 0;
        while (~is_lb_nonzero)
            counter = counter + 1;
            for jj = 1 : 4
                switch jj
                    case 1
                        x_nudge = x + counter * 0.01;
                        y_nudge = y;
                    case 2
                        x_nudge = x - counter * 0.01;
                        y_nudge = y;
                    case 3
                        x_nudge = x;
                        y_nudge = y + counter * 0.01;
                    case 4
                        x_nudge = x;
                        y_nudge = y - counter * 0.01;
                end
                lb = GetBoxVertexes(x_nudge, y_nudge);
                if (any(lb))
                    is_lb_nonzero = 1;
                    x = x_nudge;
                    y = y_nudge;
                    break;
                end
            end
        end
    end
    BVr(ii,:) = [x - lb(2), x + lb(4), y - lb(3), y + lb(1)];
    xr(ii) = x; yr(ii) = y;
    
    x = xf(ii); y = yf(ii);
    lb = GetBoxVertexes(x, y);
    if (~any(lb))
        counter = 0;
        is_lb_nonzero = 0;
        while (~is_lb_nonzero)
            counter = counter + 1;
            for jj = 1 : 4
                switch jj
                    case 1
                        x_nudge = x + counter * 0.01;
                        y_nudge = y;
                    case 2
                        x_nudge = x - counter * 0.01;
                        y_nudge = y;
                    case 3
                        x_nudge = x;
                        y_nudge = y + counter * 0.01;
                    case 4
                        x_nudge = x;
                        y_nudge = y - counter * 0.01;
                end
                lb = GetBoxVertexes(x_nudge, y_nudge);
                if (any(lb))
                    is_lb_nonzero = 1;
                    x = x_nudge;
                    y = y_nudge;
                    break;
                end
            end
        end
    end
    BVf(ii,:) = [x - lb(2), x + lb(4), y - lb(3), y + lb(1)];
    xf(ii) = x; yf(ii) = y;
    
end
end
%%
function lb = GetBoxVertexes(x,y)%x,y是将要扩展的box的起点坐标
global optimization_
unit_step = optimization_.unit_step;%init:0.03——每次扩展的长度；这两个参数要根据实际的地图尺寸来设定
max_step = optimization_.max_step;%init:10——最大扩展长度
% up left down right
lb = zeros(1,4);
is_completed = zeros(1,4);
while (sum(is_completed) < 4)
    for ind = 1 : 4%ind：1~4分别代表上左下右四个方向；lb的index也是一样
        if (is_completed(ind))%某一方向扩展完成则停止该方向的扩展
            continue;
        end
        test = lb;
        if (test(ind) + unit_step > max_step)%若扩展时一直没遇到障碍或边界，则在规定步进max_step后不在扩展
            is_completed(ind) = 1;%扩展完成标志位
            continue;
        end
        test(ind) = test(ind) + unit_step;
        if (IsCurrentEnlargementValid(x, y, test, lb, ind))%test代表代检测的扩展（有效性未知）；lb代表确定有效的扩展
            lb = test;
        else
            is_completed(ind) = 1;
        end
    end
end
end
%%
function is_valid = IsCurrentEnlargementValid(x, y, test, lb, ind)%test代表代检测的扩展（有效性未知）；lb代表确定有效的扩展
%% 检测是否碰到地图边界
switch ind
    case 1
        A = [x - lb(2), y + lb(1)];
        B = [x + lb(4), y + lb(1)];
        EA = [x - test(2), y + test(1)];
        EB = [x + test(4), y + test(1)];
        V_check = [A; B; EB; EA];
    case 2
        A = [x - lb(2), y + lb(1)];
        D = [x - lb(2), y - lb(3)];
        EA = [x - test(2), y + test(1)];
        ED = [x - test(2), y - test(3)];
        V_check = [A; D; ED; EA];
    case 3
        C = [x + lb(4), y - lb(3)];
        D = [x - lb(2), y - lb(3)];
        EC = [x + test(4), y - test(3)];
        ED = [x - test(2), y - test(3)];
        V_check = [C; D; ED; EC];
    case 4
        B = [x + lb(4), y + lb(1)];
        C = [x + lb(4), y - lb(3)];
        EB = [x + test(4), y + test(1)];
        EC = [x + test(4), y - test(3)];
        V_check = [C; B; EB; EC];
    otherwise
        is_valid = 0;
        return;
end
x_min = min(V_check(:,1));
x_max = max(V_check(:,1));
y_min = min(V_check(:,2));
y_max = max(V_check(:,2));

%global environment_scale_
if ((x_min < 0)||(x_max > 26)||(y_min < 0)||(y_max > 26)) %这里其实已经把地图scale从(-26,26)--->(0,26)中来
    is_valid = 0;
    return;
end
%% 检测是否碰到障碍物
ind_x_min = floor(x_min / 0.1) + 1;%将xy —> grid
ind_y_min = floor(y_min / 0.1) + 1;
ind_x_max = floor(x_max / 0.1) + 1;
ind_y_max = floor(y_max / 0.1) + 1;
ind_x = ind_x_min : ind_x_max;
ind_y = ind_y_min : ind_y_max;

global costmap_
if (any(costmap_(ind_x, ind_y)))%any检测到非零元素，返回1；这里为什么要加两个any？
    is_valid = 0;
else
    is_valid = 1;
end
end