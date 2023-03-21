clear all; close all; clc
global obstacle_vertexes_
global path_Position_
global path_Pos_Angel_
global path_Vel_
load('Case1.mat');
InitParam();
figure(1)
axis equal; box on; grid on; axis([-26 26 -26 26]);
set(gcf,'outerposition',get(0,'screensize'));
hold on;
for ii = 1 : Nobs
    fill(obstacle_vertexes_{ii}.x, obstacle_vertexes_{ii}.y, [125, 125, 125] ./ 255);%填充灰色
end
h=a_star();%a_star得到的为n行三列的矩阵[x,y,cost]
new_Path=ResamplePath(h,1/2);%采样频率选择：1/2*总样点数
[BVr, BVf, ~, ~, ~, ~] = SpecifySafeCorridor(new_Path(:,1) * 0.1, new_Path(:,2) * 0.1, new_Path(:,3));
path_o=get_Box_hart(BVf);%Box中心坐标
plotclosePolygon(floor(BVf/0.1));

%%在安全走廊内构造Bezier曲线
color = ['r', 'g', 'b', 'c', 'm', 'y', 'k'];%定义颜色index
n_order = 7;   % snap:8 control points n_order = 2*d_order - 1
path = new_Path(:,1:2);
n_seg = size(BVf,1);
corridor = zeros(4, n_seg);
for i = 1:n_seg
    % 对于每一列而言，前面两个是坐标，后面两个是走廊长宽
    corridor(:, i) = [path_o(i, 1), path_o(i, 2), path_o(i, 3), path_o(i, 4)]';
end
%每一段轨迹的时间分配为1s
ts = zeros(n_seg, 1);
% t_p = [1,1,1,1,1];
for i = 1:n_seg
    ts(i,1) = 1;
end

% 独立对各轴求解
% 采样点的设置一定要合理，不然quadprog函数会出现无解的情况；什么时候无解？有待研究
global bezier_
poly_coef_x = MinimumSnapCorridorBezierSolver(1, path(:, 1), corridor, ts, n_seg, n_order, bezier_.v_max, bezier_.a_max, bezier_.j_max);
disp("x axle solved!")
poly_coef_y = MinimumSnapCorridorBezierSolver(2, path(:, 2), corridor, ts, n_seg, n_order, bezier_.v_max, bezier_.a_max, bezier_.j_max);
disp("y axle solved!")

%% #####################################################
% draw bezier curve
x_pos = [];y_pos = [];
x_vel = [];y_vel = [];
x_acc = [];y_acc = [];
x_jerk = [];y_jerk = [];
idx = 1;
for j = 1:n_seg
    start_pos_id = idx;
    for t = 0:0.01:1
        x_pos(idx) = 0.0;y_pos(idx) = 0.0;
        x_vel(idx) = 0.0;y_vel(idx) = 0.0;
        x_acc(idx) = 0.0;y_acc(idx) = 0.0;
        x_jerk(idx) = 0.0;y_jerk(idx) = 0.0;
        for i = 0:n_order
            start_idx = (j-1)*(n_order+1)+i;
            basis_p = nchoosek(n_order, i) * t^i * (1-t)^(n_order-i);%nchoosek为组合数C_n取k
            x_pos(idx) = x_pos(idx) + poly_coef_x(start_idx+1)*basis_p*ts(j);
            y_pos(idx) = y_pos(idx) + poly_coef_y(start_idx+1)*basis_p*ts(j);
            if i < n_order
                basis_v = nchoosek(n_order-1, i) * t^i *(1-t)^(n_order-1-i);
                x_vel(idx) = x_vel(idx) + (n_order+1) * (poly_coef_x(start_idx+2)-poly_coef_x(start_idx+1))*basis_v;
                y_vel(idx) = y_vel(idx) + (n_order+1) * (poly_coef_y(start_idx+2)-poly_coef_y(start_idx+1))*basis_v;
            end
            if i < n_order-1
                basis_a = nchoosek(n_order-2, i) * t^i *(1-t)^(n_order-2-i);
                x_acc(idx) = x_acc(idx) + (n_order+1) * n_order * (poly_coef_x(start_idx+3) - 2*poly_coef_x(start_idx+2) + poly_coef_x(start_idx+1))*basis_a/ts(j);
                y_acc(idx) = y_acc(idx) + (n_order+1) * n_order * (poly_coef_y(start_idx+3) - 2*poly_coef_y(start_idx+2) + poly_coef_y(start_idx+1))*basis_a/ts(j);
            end
            if i < n_order-2
                basis_j = nchoosek(n_order-3, i) * t^i *(1-t)^(n_order-3-i);
                x_jerk(idx) = x_jerk(idx) + (n_order+1) * n_order * (n_order-1) * ...
                    (poly_coef_x(start_idx+4) - 3*poly_coef_x(start_idx+3) + 3*poly_coef_x(start_idx+2) - poly_coef_x(start_idx+1)) * basis_j/ts(j)^2;
                y_jerk(idx) = y_jerk(idx) + (n_order+1) * n_order * (n_order-1) * ...
                    (poly_coef_y(start_idx+4) - 3*poly_coef_y(start_idx+3) + 3*poly_coef_y(start_idx+2) - poly_coef_y(start_idx+1)) * basis_j/ts(j)^2;
            end
        end
        idx = idx + 1;
    end
    end_pos_id = idx - 1;
    %scatter(ts(j)*poly_coef_x((j-1)*(n_order+1)+1:(j-1)*(n_order+1)+1+n_order), ts(j)*poly_coef_y((j-1)*(n_order+1)+1:(j-1)*(n_order+1)+1+n_order), 'filled',color(mod(j,7)+1),'LineWidth', 5);hold on;
    plot(x_pos(start_pos_id:end_pos_id), y_pos(start_pos_id:end_pos_id), color(mod(j,7)+1),'LineWidth', 2);hold on;%mod函数为求余数
    %作图函数先不修改，看看实际效果如何
end

figure(3) %p,v.a.j--t待画
subplot(4,2,1)
plot(x_pos, 'Color', 'r');title('x position');
subplot(4,2,2)
plot(y_pos, 'Color', 'g');title('y position');
subplot(4,2,3)
plot(x_vel, 'Color', 'r');title('x velocity');
subplot(4,2,4)
plot(y_vel, 'Color', 'g');title('y velocity');
subplot(4,2,5)
plot(x_acc, 'Color', 'r');title('x acceleration');
subplot(4,2,6)
plot(y_acc, 'Color', 'g');title('y acceleration');
subplot(4,2,7)
plot(x_jerk, 'Color', 'r');title('x jerk');
subplot(4,2,8)
plot(y_jerk, 'Color', 'g');title('y jerk');

%% 对最后的轨迹进行重采样，用于做轨迹跟踪，减少计算量
path_Position_ = [x_pos',y_pos'];
path_Vel_ = [x_vel',y_vel'];
path_Pos_Angel_ = ResamplePath(path_Position_,1/10);
path_Vel_ = ResamplePath(path_Vel_,1/10);
MPC_Tracking(path_Pos_Angel_,path_Vel_);
figure(8);
plot(path_Pos_Angel_(:,3),'k-');