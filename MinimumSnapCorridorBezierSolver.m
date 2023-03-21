function poly_coef = MinimumSnapCorridorBezierSolver(axis, waypoints, corridor, ts, n_seg, n_order, v_max, a_max, j_max)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond   = [waypoints(end), 0, 0, 0];   
    
    %% #####################################################
    % STEP 1: compute Q_0 of c'Q_0c
    [Q, M]  = getQM(n_seg, n_order, ts);
    Q_0 = M'*Q*M;
    % 返回和Q矩阵距离最近的一个对称正定(Symmetric Positive Definite)矩阵Q'.
    % 目的是在把目标函数微调为一个凸函数,保证得到的解为全局最优解.
    Q_0 = nearestSPD(Q_0);
    
    %% #####################################################
    % STEP 2: get Aeq and beq
    [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond);
    
    %% #####################################################
    % STEP 3: get corridor_range, Aieq and bieq
    
    % STEP 3.1: get corridor_range of x-axis or y-axis,
    % you can define corridor_range as [p1_min, p1_max;
    %                                   p2_min, p2_max;
    %                                   ...,
    %                                   pn_min, pn_max ];
    corridor_range = zeros(n_seg,2);
    for k = 1:n_seg
        % 横坐标/纵坐标减去/加上corridor宽度得到x/y轴边界
        corridor_range(k,:) = [corridor(axis,k) - corridor(axis+2,k), corridor(axis,k) + corridor(axis+2,k)];
    end
    
    % STEP 3.2: get Aieq and bieq
    [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max, j_max);
    
    f = zeros(size(Q_0,1),1);
    % quadprog函数参数介绍：https://blog.csdn.net/tianzy16/article/details/87916128
    poly_coef = quadprog(Q_0,f,Aieq, bieq, Aeq, beq);
end

%% 
%  p4-------p3
%   |       |
%   |       |
%  p1-------p2
%%
function plot_rect(center, x_r, y_r) 
    p1 = center+[-x_r;-y_r];
    p2 = center+[-x_r;y_r];
    p3 = center+[x_r;y_r];
    p4 = center+[x_r;-y_r];
    plot_line(p1,p2);
    plot_line(p2,p3);
    plot_line(p3,p4);
    plot_line(p4,p1);
end

function plot_line(p1,p2)
    a = [p1(:),p2(:)];
    plot(a(1,:),a(2,:),'b');
end
