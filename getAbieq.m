function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max, j_max)%这里曲线段数等于控制点个数
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 3.2.1 p constraint
    % 安全性约束
    coeff_p = ones(n_all_poly,1);
    bieq_p = [];
    for k = 1:n_seg % max
        % 这个没有完全理解为什么要这么构造？
        coeff_p(1 + (k-1)*(n_order+1):k*(n_order+1)) = coeff_p(1 + (k-1)*(n_order+1):k*(n_order+1)) * ts(k)^(1);
        % 放入上边界
        bieq_p = [bieq_p;ones(n_order+1,1)*corridor_range(k,2)];
    end
    for k = 1:n_seg % -min
        % 放入下边界
        bieq_p = [bieq_p;ones(n_order+1,1)*corridor_range(k,1)*(-1)];
    end
    Aieq_p = diag(coeff_p,0);
    Aieq_p = [Aieq_p;-Aieq_p];

    %#####################################################
    % STEP 3.2.2 v constraint  
    % 动力学约束: v
    n_ctr = n_order;      % the number of control posints after first deferention: n_order ;less one than the p_constrain's 
    n_eq = n_ctr*n_seg*2; % number of equations (max and min constraints)
    Aieq_v = zeros(n_eq/2,n_all_poly);

    for k = 1:n_seg
        for n = 1:n_ctr
            index_col = k*(n_order+1)-1;
            index_row = n+(k-1)*n_ctr;
            Aieq_v(index_row,index_col:index_col+1) = n_order * [-1, 1] * ts(k)^(0);
        end
    end
    Aieq_v = [Aieq_v;-Aieq_v];
    bieq_v = ones(n_eq,1)* v_max;%这里的v_max和v_min一致
    
%     bieq_v = ones(n_eq/2,1)* v_max;

    %#####################################################
    % STEP 3.2.3 a constraint 
    % 动力学约束: a
    n_ctr = n_order-1;    % the number of control posints after second deferention: n_order - 1 ,less one than the v_constrain's
    n_eq = n_ctr*n_seg*2; % number of equations (max and min constraints)
    Aieq_a = zeros(n_eq/2, n_all_poly);
    
    for k = 1:n_seg
        for n = 1:n_ctr
            index_col = k*(n_order+1)-2;
            index_row = n+(k-1)*n_ctr;
            Aieq_a(index_row,index_col:index_col+2) = n_order * (n_order-1) * [1, -2, 1] * ts(k)^(-1);
        end
    end
    Aieq_a = [Aieq_a;-Aieq_a];
    bieq_a = ones(n_eq,1)*a_max;%这里的a_max和a_min一致

%     bieq_a = ones(n_eq/2,1)*a_max;

    % STEP 3.2.4 j constraint 
    n_ctr = n_order-2;    % the number of control posints after third deferention: n_order - 2
    n_eq = n_ctr*n_seg*2; % number of equations (max and min constraints)
    Aieq_j = zeros(n_eq/2, n_all_poly);
    
    for k = 1:n_seg
        for n = 1:n_ctr
            index_col = k*(n_order+1)-3;
            index_row = n+(k-1)*n_ctr;
            Aieq_j(index_row,index_col:index_col+3) = n_order * (n_order-1) * (n_order-2) * [1, -3, 3, -1] * ts(k)^(-2);
        end
    end
    Aieq_j = [Aieq_j;-Aieq_j];
    bieq_j = ones(n_eq,1)*j_max;%这里的j_max和j_min也一致


    %#####################################################
     % combine all components to form Aieq and bieq  

    Aieq = [Aieq_p; Aieq_v; Aieq_a; Aieq_j];
    bieq = [bieq_p; bieq_v; bieq_a; bieq_j];
%     Aieq = Aieq_p;
%     bieq = bieq_p;
end
